"""
Voice Controller Node
---------------------
Runs alongside llm_controller_node. Listens to ReSpeaker mic continuously, waits
for wake word "Hey Jarvis", records the next utterance, sends it through the
existing /chat endpoint (which already does LLM + tool execution), then speaks
the reply through the robot's speaker via Piper TTS.

Flow:
  ReSpeaker mic -> openWakeWord (local) -> ding -> record (max 8s, stop on 1.5s silence)
  -> OpenAI Whisper (cloud) -> POST /chat -> Piper TTS (local) -> aplay -> speaker

The LLM and tool-calling logic are unchanged: we just add audio I/O around them.
"""

import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import wave
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

import requests
import sounddevice as sd
from openai import OpenAI
from openwakeword.model import Model as WakeWordModel
from dotenv import load_dotenv


# ---- Audio settings ----
SAMPLE_RATE = 16000          # openwakeword + Whisper expect 16k
CHANNELS_REC = 1             # mono - we'll take channel 0 of the 6-channel ReSpeaker stream
CHUNK_SAMPLES = 1280         # 80 ms at 16k - openwakeword frame size

# ReSpeaker is "ArrayUAC10" in `aplay -l` (card 1)
INPUT_DEVICE_HINT = "ReSpeaker"
OUTPUT_DEVICE_HW = "plughw:1,0"   # for aplay

# ---- Wake word ----
HEY_JARVIS_PATH = "/home/pi/.local/lib/python3.12/site-packages/openwakeword/resources/models/hey_jarvis_v0.1.onnx"
WAKE_THRESHOLD = 0.5

# ---- Recording after wake ----
MAX_RECORD_SEC = 8.0
SILENCE_THRESHOLD_RMS = 0.012   # tweak if env is noisy
SILENCE_HANGOVER_SEC = 1.5      # stop after this much continuous silence
MIN_UTTERANCE_SEC = 0.5         # ignore wakes that produce < this

# ---- Piper ----
PIPER_MODEL = "/home/pi/voice_models/piper/zh_CN-huayan-medium.onnx"

# ---- LLM endpoint (running in same process? no — it's the other node) ----
LLM_ENDPOINT = "http://127.0.0.1:5000/chat"
EVENT_ENDPOINT = "http://127.0.0.1:5000/event"
LISTENING_ENDPOINT = "http://127.0.0.1:5000/listening"


def find_input_device():
    """Find ReSpeaker by name; fall back to default."""
    devs = sd.query_devices()
    for i, d in enumerate(devs):
        if INPUT_DEVICE_HINT.lower() in d["name"].lower() and d["max_input_channels"] > 0:
            return i
    return None  # default


def rms(audio_np):
    if len(audio_np) == 0:
        return 0.0
    return float(np.sqrt(np.mean(audio_np.astype(np.float32) ** 2)) / 32768.0)


def play_wav(path):
    """Blocking playback via aplay on the ReSpeaker output."""
    try:
        subprocess.run(
            ["aplay", "-q", "-D", OUTPUT_DEVICE_HW, path],
            check=False, timeout=30,
        )
    except Exception as e:
        print(f"[voice] aplay error: {e}", file=sys.stderr)


def beep_ding():
    """Quick acknowledgement beep using sounddevice (no file needed)."""
    try:
        t = np.linspace(0, 0.15, int(SAMPLE_RATE * 0.15), False)
        tone = 0.3 * np.sin(2 * np.pi * 880 * t)  # A5
        sd.play(tone.astype(np.float32), SAMPLE_RATE, device=None, blocking=True)
    except Exception as e:
        print(f"[voice] beep error: {e}", file=sys.stderr)


class VoiceControllerNode(Node):
    def __init__(self):
        super().__init__("voice_controller")

        # Load API key (only used for Whisper)
        env_path = Path.home() / "desktoprobo" / ".env"
        if env_path.exists():
            load_dotenv(env_path)
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY not set")
            raise RuntimeError("missing OPENAI_API_KEY")
        self.openai = OpenAI(api_key=api_key)

        # Wake word model
        self.get_logger().info("Loading wake word model (hey_jarvis)...")
        self.ww_model = WakeWordModel(wakeword_model_paths=[HEY_JARVIS_PATH])
        self.get_logger().info("Wake word ready.")

        # Audio in stream
        self.in_dev = find_input_device()
        self.get_logger().info(f"Using input device #{self.in_dev}")

        self._audio_q = queue.Queue()
        self._stop_flag = threading.Event()
        self._listening_cached = True
        self._listening_cached_until = 0.0

        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

        self.get_logger().info('Voice controller online. Say "Hey Jarvis" to wake.')

    # ---------- Mic streaming ----------
    def _audio_cb(self, indata, frames, time_info, status):
        if status:
            pass
        chan0 = indata[:, 0].copy()
        self._audio_q.put(chan0)

    def _listen_loop(self):
        try:
            with sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=6,                # ReSpeaker provides 6
                dtype="int16",
                blocksize=CHUNK_SAMPLES,
                device=self.in_dev,
                callback=self._audio_cb,
            ):
                while not self._stop_flag.is_set():
                    try:
                        chunk = self._audio_q.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    if len(chunk) != CHUNK_SAMPLES:
                        continue
                    scores = self.ww_model.predict(chunk)
                    score = scores.get("hey_jarvis_v0.1", 0.0)
                    if score >= WAKE_THRESHOLD:
                        self.get_logger().info(f"WAKE detected (score={score:.2f})")
                        if not self._is_listening():
                            self._post_event("system", f"(忽略唤醒，监听已暂停 score={score:.2f})")
                            self.ww_model.reset()
                            continue
                        self._post_event("wake", f"唤醒 (score={score:.2f})", score=float(score))
                        self._handle_interaction()
                        with self._audio_q.mutex:
                            self._audio_q.queue.clear()
                        self.ww_model.reset()
        except Exception as e:
            self.get_logger().error(f"listen loop crashed: {e}")

    # ---------- Per-utterance pipeline ----------
    def _handle_interaction(self):
        try:
            beep_ding()
            self._post_event("recording", "录音中...")
            audio = self._record_until_silence()
            if audio is None or len(audio) < SAMPLE_RATE * MIN_UTTERANCE_SEC:
                self.get_logger().info("(too short, ignoring)")
                self._post_event("system", "(语音过短，已忽略)")
                return

            duration = len(audio) / SAMPLE_RATE
            self.get_logger().info(f"recorded {duration:.1f}s, transcribing...")
            self._post_event("transcribing", f"已录 {duration:.1f}s，识别中...")
            text = self._transcribe(audio)
            if not text.strip():
                self.get_logger().info("(empty transcript)")
                self._post_event("system", "(语音被过滤或识别为空)")
                return
            self.get_logger().info(f"USER said: {text}")

            reply = self._ask_llm(text)
            self.get_logger().info(f"LLM reply: {reply}")

            self._post_event("speaking", "合成语音中...")
            self._speak(reply)
            self._post_event("speaking_done", "")
        except Exception as e:
            self.get_logger().error(f"interaction error: {e}")
            self._post_event("error", f"interaction error: {e}")

    def _record_until_silence(self):
        with self._audio_q.mutex:
            self._audio_q.queue.clear()

        buf = []
        silent_for = 0.0
        start = time.time()
        while True:
            try:
                chunk = self._audio_q.get(timeout=0.3)
            except queue.Empty:
                continue
            buf.append(chunk)
            chunk_dur = len(chunk) / SAMPLE_RATE
            if rms(chunk) < SILENCE_THRESHOLD_RMS:
                silent_for += chunk_dur
            else:
                silent_for = 0.0
            elapsed = time.time() - start
            if silent_for >= SILENCE_HANGOVER_SEC and elapsed > 0.8:
                break
            if elapsed > MAX_RECORD_SEC:
                break
        if not buf:
            return None
        return np.concatenate(buf)

    # Known Whisper hallucinations on silent / unclear audio (esp. for Chinese).
    _WHISPER_HALLUCINATIONS = (
        "请不吝", "点赞", "订阅", "转发", "打赏",
        "明镜与点点", "字幕由", "字幕志愿者",
        "Thank you for watching", "Subscribe to my channel",
    )

    def _transcribe(self, audio_int16):
        """Send wav to Whisper. Returns "" on hallucination / silence."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
        with wave.open(wav_path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(audio_int16.tobytes())
        try:
            with open(wav_path, "rb") as f:
                resp = self.openai.audio.transcriptions.create(
                    model="whisper-1",
                    file=f,
                    prompt=(
                        "This is a short voice command to a small wheeled "
                        "desktop robot. Possible commands: forward, backward, "
                        "stop, turn left, turn right, spin, faster, slower. "
                        "中文：前进、后退、停止、左转、右转、转一圈、再快一点、慢一点。"
                    ),
                    temperature=0.0,
                )
            text = (resp.text or "").strip()
            for bad in self._WHISPER_HALLUCINATIONS:
                if bad in text:
                    self.get_logger().info(f"(hallucination filtered: {text[:50]!r})")
                    return ""
            return text
        finally:
            try:
                os.unlink(wav_path)
            except OSError:
                pass

    def _is_listening(self) -> bool:
        """Cached check: query /listening every 3s; default to True if unreachable."""
        now = time.time()
        if now < self._listening_cached_until:
            return self._listening_cached
        try:
            r = requests.get(LISTENING_ENDPOINT, timeout=1.5)
            self._listening_cached = bool(r.json().get("enabled", True))
        except Exception:
            self._listening_cached = True   # fail open
        self._listening_cached_until = now + 3.0
        return self._listening_cached

    def _post_event(self, kind, text, **extra):
        """Best-effort push to the LLM node's event log so the web UI sees voice activity."""
        try:
            payload = {"kind": kind, "text": text}
            payload.update(extra)
            requests.post(EVENT_ENDPOINT, json=payload, timeout=2)
        except Exception as e:
            self.get_logger().warn(f"event post failed: {e}")

    def _ask_llm(self, user_text: str) -> str:
        try:
            r = requests.post(LLM_ENDPOINT,
                              json={"message": user_text, "source": "voice"},
                              timeout=120)
            data = r.json()
            return data.get("reply", "(no reply)")
        except Exception as e:
            return f"LLM 调用失败: {e}"

    def _speak(self, text: str):
        if not text.strip():
            return
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
        try:
            proc = subprocess.run(
                ["python3", "-m", "piper",
                 "--model", PIPER_MODEL,
                 "--output_file", wav_path],
                input=text.encode("utf-8"),
                capture_output=True, timeout=20,
            )
            if proc.returncode != 0:
                self.get_logger().error(f"piper failed: {proc.stderr.decode()[:200]}")
                return
            play_wav(wav_path)
        finally:
            try:
                os.unlink(wav_path)
            except OSError:
                pass

    def shutdown(self):
        self._stop_flag.set()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
