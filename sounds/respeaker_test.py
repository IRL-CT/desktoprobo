#!/usr/bin/env python3
"""
respeaker_test.py
Minimal end-to-end test for the ReSpeaker Mic Array v2.0 on RPi5.

  1. Lists every PortAudio device.
  2. Auto-detects the ReSpeaker for input AND output.
  3. Records `--duration` seconds of audio at 16 kHz from the array.
  4. Plays the recording back through the ReSpeaker's 3.5mm jack
     by opening an output stream pinned to the ReSpeaker output device.

Run:
    python3 respeaker_test.py
    python3 respeaker_test.py --channels 6 --duration 8
"""

import argparse
import sys
import wave

import pyaudio

RESPEAKER_NAME_HINTS = ("respeaker", "seeed", "mic array", "uac1.0")
RATE = 16000           # native rate of the v2.0 firmware
CHUNK = 1024
FORMAT = pyaudio.paInt16
SAMPLE_WIDTH = 2       # bytes per sample for paInt16
OUTPUT_FILE = "/tmp/respeaker_test.wav"


def find_respeaker_indices(pa: pyaudio.PyAudio):
    """Return (input_idx, output_idx) for the ReSpeaker, or raise."""
    in_idx, out_idx = None, None
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        name = info["name"].lower()
        if not any(h in name for h in RESPEAKER_NAME_HINTS):
            continue
        if in_idx is None and info["maxInputChannels"] > 0:
            in_idx = i
        if out_idx is None and info["maxOutputChannels"] > 0:
            out_idx = i
    if in_idx is None:
        raise RuntimeError(
            "ReSpeaker input not found. Run with no args to see device list, "
            "then add the matching string to RESPEAKER_NAME_HINTS."
        )
    if out_idx is None:
        raise RuntimeError(
            "ReSpeaker output not found. The board exposes a USB UAC1.0 "
            "playback endpoint that drives the 3.5mm jack — confirm with `aplay -l`."
        )
    return in_idx, out_idx


def list_devices(pa: pyaudio.PyAudio) -> None:
    print("--- PortAudio devices ---")
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        print(
            f"  [{i:2d}] {info['name']:50s}  "
            f"in={info['maxInputChannels']}  out={info['maxOutputChannels']}  "
            f"default_rate={int(info['defaultSampleRate'])}"
        )
    print("-------------------------")


def record(pa: pyaudio.PyAudio, device_index: int, channels: int, duration: float) -> bytes:
    stream = pa.open(
        format=FORMAT,
        channels=channels,
        rate=RATE,
        input=True,
        input_device_index=device_index,
        frames_per_buffer=CHUNK,
    )
    n_chunks = int(RATE / CHUNK * duration)
    print(f"Recording {duration}s ({channels}ch @ {RATE}Hz) — speak now...")
    frames = []
    for _ in range(n_chunks):
        frames.append(stream.read(CHUNK, exception_on_overflow=False))
    stream.stop_stream()
    stream.close()
    print("Recording done.")
    return b"".join(frames)


def save_wav(pcm: bytes, channels: int, path: str) -> None:
    with wave.open(path, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(RATE)
        wf.writeframes(pcm)
    print(f"Saved -> {path}")


def play_through_respeaker(pa: pyaudio.PyAudio, output_index: int, path: str) -> None:
    """Play wav through the ReSpeaker output -> 3.5mm jack.

    The ReSpeaker output endpoint is stereo; if we recorded mono / 6-ch we
    let pyaudio's plug layer (or the file's own header) drive channel count
    by reopening the wav and adapting.
    """
    with wave.open(path, "rb") as wf:
        # ReSpeaker output is 2-channel; if the wav is mono we duplicate,
        # if it's 6ch we keep ch0 only (the processed channel) and duplicate.
        in_channels = wf.getnchannels()
        out_channels = 2
        sample_width = wf.getsampwidth()
        framerate = wf.getframerate()
        raw = wf.readframes(wf.getnframes())

    pcm_out = _to_stereo(raw, in_channels, sample_width)

    stream = pa.open(
        format=pa.get_format_from_width(sample_width),
        channels=out_channels,
        rate=framerate,
        output=True,
        output_device_index=output_index,
    )
    print(f"Playing back through ReSpeaker output (device {output_index}) -> 3.5mm jack...")
    # Write in chunks so audio actually starts streaming promptly.
    bytes_per_frame = out_channels * sample_width
    chunk_bytes = CHUNK * bytes_per_frame
    for offset in range(0, len(pcm_out), chunk_bytes):
        stream.write(pcm_out[offset : offset + chunk_bytes])
    stream.stop_stream()
    stream.close()
    print("Playback done.")


def _to_stereo(raw: bytes, in_channels: int, sample_width: int) -> bytes:
    """Down/upmix raw PCM bytes to interleaved stereo (int16 only)."""
    if sample_width != 2:
        # Keep this script focused; we always record paInt16 above.
        return raw
    import array
    samples = array.array("h")
    samples.frombytes(raw)
    n_frames = len(samples) // in_channels
    out = array.array("h", [0] * (n_frames * 2))
    if in_channels == 1:
        for i in range(n_frames):
            v = samples[i]
            out[2 * i] = v
            out[2 * i + 1] = v
    elif in_channels == 2:
        return raw
    else:
        # Multi-channel (e.g. 6ch from ReSpeaker): take channel 0 (processed).
        for i in range(n_frames):
            v = samples[i * in_channels]
            out[2 * i] = v
            out[2 * i + 1] = v
    return out.tobytes()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--channels", type=int, default=1,
                    help="Capture channels: 1 (processed mono, default) or 6 (raw multichannel).")
    ap.add_argument("--duration", type=float, default=5.0, help="Seconds to record.")
    ap.add_argument("--list-only", action="store_true", help="Just list devices and exit.")
    args = ap.parse_args()

    pa = pyaudio.PyAudio()
    try:
        list_devices(pa)
        if args.list_only:
            return 0

        in_idx, out_idx = find_respeaker_indices(pa)
        print(f"ReSpeaker: input={in_idx}, output={out_idx}")

        pcm = record(pa, in_idx, args.channels, args.duration)
        save_wav(pcm, args.channels, OUTPUT_FILE)
        play_through_respeaker(pa, out_idx, OUTPUT_FILE)
        print("OK.")
        return 0
    except Exception as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1
    finally:
        pa.terminate()


if __name__ == "__main__":
    sys.exit(main())
