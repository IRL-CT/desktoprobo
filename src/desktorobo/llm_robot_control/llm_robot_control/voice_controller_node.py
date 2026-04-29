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
import webrtcvad
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

import requests
import sounddevice as sd
from openai import OpenAI
from openwakeword.model import Model as WakeWordModel
from piper.voice import PiperVoice
from dotenv import load_dotenv


# ---- Audio settings ----
SAMPLE_RATE = 16000          # openwakeword + Whisper expect 16k
CHANNELS_REC = 1             # mono - we'll take channel 0 of the 6-channel ReSpeaker stream
CHUNK_SAMPLES = 1280         # 80 ms at 16k - openwakeword frame size

# ReSpeaker is "ArrayUAC10" in `aplay -l` (card 1)
INPUT_DEVICE_HINT = "ReSpeaker"
OUTPUT_DEVICE_HW = "plughw:CARD=ArrayUAC10,DEV=0"   # name-based, survives card-number changes

# ---- Wake word ----
HEY_JARVIS_PATH = "/home/pi/.local/lib/python3.12/site-packages/openwakeword/resources/models/hey_jarvis_v0.1.onnx"
WAKE_THRESHOLD = 0.5

# ---- Recording after wake ----
MAX_RECORD_SEC = 12.0           # safety bound
MIN_INITIAL_WAIT_SEC = 4.0      # if user never starts speaking after wake, give up after this
VAD_FRAME_MS = 30               # webrtcvad supports 10/20/30 ms frames
VAD_AGGRESSIVENESS = 2          # 0=lenient ... 3=very strict; 2 is a good default
VAD_END_SILENCE_MS = 700        # end recording after this much non-speech after speech started
FOLLOWUP_WINDOW_SEC = 10.0      # after a successful interaction, listen for follow-up commands without wake word
MIN_UTTERANCE_SEC = 0.5         # ignore wakes that produce < this

# ---- Piper ----
PIPER_MODEL = "/home/pi/voice_models/piper/en_US-amy-low.onnx"
TTS_GAIN = 2.0   # software amp factor; ReSpeaker has no hardware volume

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
    """Tiny non-blocking acknowledgement chirp. Doesn't delay the recording start."""
    try:
        dur = 0.05  # 50ms — much shorter than before
        t = np.linspace(0, dur, int(SAMPLE_RATE * dur), False)
        # rising chirp 660->1320 Hz: more "hi!" feel
        freq = 660 + (1320 - 660) * (t / dur)
        tone = 0.12 * np.sin(2 * np.pi * freq * t)
        # quick fade in/out to avoid clicks
        envelope = np.ones_like(t)
        fade = int(SAMPLE_RATE * 0.005)
        envelope[:fade] = np.linspace(0, 1, fade)
        envelope[-fade:] = np.linspace(1, 0, fade)
        tone *= envelope
        sd.play(tone.astype(np.float32), SAMPLE_RATE, device=None, blocking=False)
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

        # Piper TTS voice — load once, reuse on every reply (saves 1-2s onnxruntime startup per call)
        self.get_logger().info(f"Loading Piper voice from {PIPER_MODEL}...")
        self.piper_voice = PiperVoice.load(PIPER_MODEL)
        self.get_logger().info("Piper voice ready.")

        # Audio in stream
        self.in_dev = find_input_device()
        self.get_logger().info(f"Using input device #{self.in_dev}")

        self._audio_q = queue.Queue()
        self._stop_flag = threading.Event()
        self._listening_cached = True
        self._listening_cached_until = 0.0
        self._vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
        self._followup_active_until = 0.0   # follow-up listening expires at this time
        self._is_speaking = False           # true while Piper TTS plays back

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
                    # Path A: explicit wake word
                    if score >= WAKE_THRESHOLD:
                        self.get_logger().info(f"WAKE detected (score={score:.2f})")
                        if not self._is_listening():
                            self._post_event("system", f"(wake ignored, listening paused, score={score:.2f})")
                            self.ww_model.reset()
                            continue
                        self._post_event("wake", f"Wake (score={score:.2f})", score=float(score))
                        self._handle_interaction(from_followup=False)
                        with self._audio_q.mutex:
                            self._audio_q.queue.clear()
                        self.ww_model.reset()
                        continue

                    # Path B: inside follow-up window, VAD-detected speech also triggers
                    in_followup = time.time() < self._followup_active_until
                    if (in_followup
                            and not self._is_speaking
                            and self._is_listening()
                            and self._chunk_has_speech(chunk)):
                        self._post_event("wake", "Follow-up speech detected (no wake word needed)", followup=True)
                        self._handle_interaction(from_followup=True)
                        with self._audio_q.mutex:
                            self._audio_q.queue.clear()
                        self.ww_model.reset()
                        continue
        except Exception as e:
            self.get_logger().error(f"listen loop crashed: {e}")

    # ---------- Per-utterance pipeline ----------
    def _handle_interaction(self, from_followup: bool = False):
        try:
            if not from_followup:
                beep_ding()
            mode_label = "(follow-up) " if from_followup else ""
            self._post_event("recording", f"{mode_label}Recording...")
            audio = self._record_until_silence()
            if audio is None or len(audio) < SAMPLE_RATE * MIN_UTTERANCE_SEC:
                self.get_logger().info("(too short, ignoring)")
                self._post_event("system", "(too short, ignored)")
                return

            duration = len(audio) / SAMPLE_RATE
            self.get_logger().info(f"recorded {duration:.1f}s, transcribing...")
            self._post_event("transcribing", f"Recorded {duration:.1f}s, transcribing...")
            text = self._transcribe(audio)
            if not text.strip():
                self.get_logger().info("(empty transcript)")
                self._post_event("system", "(empty transcript)")
                return
            self.get_logger().info(f"USER said: {text}")

            reply = self._ask_llm(text)
            self.get_logger().info(f"LLM reply: {reply}")

            # Decide: short ack (just play a beep) vs full TTS
            if self._is_short_ack(reply):
                self._post_event("speaking", f"(ack-only beep, no TTS) reply='{reply}'")
                self._play_ack_beep()
                self._post_event("speaking_done", "")
            else:
                self._post_event("speaking", "Synthesizing...")
                self._is_speaking = True
                try:
                    self._speak(reply)
                finally:
                    self._is_speaking = False
                self._post_event("speaking_done", "")

            # Open the follow-up window so the next command doesn't need a wake word.
            self._followup_active_until = time.time() + FOLLOWUP_WINDOW_SEC
            self._post_event("system",
                f"Listening for follow-up commands for {FOLLOWUP_WINDOW_SEC:.0f}s — "
                "no need to say 'Hey Jarvis' again.")
        except Exception as e:
            self.get_logger().error(f"interaction error: {e}")
            self._post_event("error", f"interaction error: {e}")

    def _record_until_silence(self):
        """Record using WebRTC VAD: stop ~700ms after the user stops speaking."""
        with self._audio_q.mutex:
            self._audio_q.queue.clear()

        FRAME_SAMPLES = int(SAMPLE_RATE * VAD_FRAME_MS / 1000)   # 480 @ 16k for 30ms
        END_FRAMES = max(1, int(VAD_END_SILENCE_MS / VAD_FRAME_MS))

        captured = []                # full audio chunks (np.int16 arrays)
        leftover = np.zeros(0, dtype=np.int16)   # bytes not yet processed by VAD
        silence_run = 0              # contiguous non-speech VAD frames
        speech_seen = False
        start = time.time()

        while True:
            try:
                chunk = self._audio_q.get(timeout=0.3)
            except queue.Empty:
                if time.time() - start > MAX_RECORD_SEC:
                    break
                continue

            captured.append(chunk)
            leftover = np.concatenate([leftover, chunk])

            # consume leftover in 30ms windows
            while len(leftover) >= FRAME_SAMPLES:
                frame = leftover[:FRAME_SAMPLES]
                leftover = leftover[FRAME_SAMPLES:]
                try:
                    is_speech = self._vad.is_speech(frame.tobytes(), SAMPLE_RATE)
                except Exception:
                    is_speech = False
                if is_speech:
                    speech_seen = True
                    silence_run = 0
                else:
                    silence_run += 1
                # speech ended?
                if speech_seen and silence_run >= END_FRAMES:
                    return np.concatenate(captured) if captured else None

            elapsed = time.time() - start
            if elapsed > MAX_RECORD_SEC:
                break
            if not speech_seen and elapsed > MIN_INITIAL_WAIT_SEC:
                break
        return np.concatenate(captured) if captured else None

    # English Whisper hallucinations on silent / unintelligible audio.
    _WHISPER_HALLUCINATIONS = (
        "Thank you for watching", "Subscribe to my channel",
        "Thanks for watching", "Please subscribe",
        "Thanks for listening", "Don't forget to subscribe",
        "MBC ╓ ╗", "[Music]", "[music]",
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
                        "This is a short English voice command to a small "
                        "wheeled desktop robot. Possible commands: forward, "
                        "backward, stop, turn left, turn right, spin, "
                        "faster, slower, drift left, drift right, go home, "
                        "where are you, status."
                    ),
                    language="en",
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

    @staticmethod
    def _is_short_ack(reply: str) -> bool:
        """Heuristic: treat short, generic acknowledgements as 'no need to TTS'."""
        if not reply:
            return True
        t = reply.strip().lower().rstrip(".!,?")
        SHORT_ACKS = {
            "ok", "okay", "okey", "k", "kk",
            "done", "got it", "sure", "yes", "alright", "yep", "yup",
            "moving", "moved", "stopped", "stopping",
            "turning", "turned", "spinning", "spun",
            "waiting", "going", "rolling",
        }
        if t in SHORT_ACKS:
            return True
        # Also: <= 4 words and no informative numbers/punctuation marks
        words = t.split()
        if len(words) <= 4 and not any(ch.isdigit() for ch in t):
            return True
        return False

    @staticmethod
    def _play_ack_beep():
        """Soft two-tone descending chirp (1100 -> 880 Hz, 80ms) — the 'done' sound."""
        try:
            dur = 0.08
            t = np.linspace(0, dur, int(SAMPLE_RATE * dur), False)
            freq = 1100 + (880 - 1100) * (t / dur)
            tone = 0.15 * np.sin(2 * np.pi * freq * t)
            envelope = np.ones_like(t)
            fade = int(SAMPLE_RATE * 0.005)
            envelope[:fade] = np.linspace(0, 1, fade)
            envelope[-fade:] = np.linspace(1, 0, fade)
            sd.play((tone * envelope).astype(np.float32), SAMPLE_RATE, device=None, blocking=False)
        except Exception:
            pass

    def _chunk_has_speech(self, chunk):
        """Quick VAD check on the leading 30ms of a chunk (used for follow-up trigger)."""
        FRAME_SAMPLES = int(SAMPLE_RATE * VAD_FRAME_MS / 1000)
        if len(chunk) < FRAME_SAMPLES:
            return False
        try:
            return self._vad.is_speech(chunk[:FRAME_SAMPLES].tobytes(), SAMPLE_RATE)
        except Exception:
            return False

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
            return f"LLM call failed: {e}"

    def _speak(self, text: str):
        if not text.strip():
            return
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
        try:
            with wave.open(wav_path, "wb") as wf:
                self.piper_voice.synthesize_wav(text, wf)
            self._amplify_wav(wav_path, TTS_GAIN)
            play_wav(wav_path)
        except Exception as e:
            self.get_logger().error(f"piper error: {e}")
        finally:
            try:
                os.unlink(wav_path)
            except OSError:
                pass

    @staticmethod
    def _amplify_wav(path, gain):
        """Multiply 16-bit PCM samples by gain, clipping to int16 range. In-place."""
        try:
            with wave.open(path, 'rb') as wf:
                params = wf.getparams()
                frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32)
            audio = np.clip(audio * gain, -32768, 32767).astype(np.int16)
            with wave.open(path, 'wb') as wf:
                wf.setparams(params)
                wf.writeframes(audio.tobytes())
        except Exception as e:
            print(f"[voice] amplify error: {e}")

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
