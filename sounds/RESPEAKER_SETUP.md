# ReSpeaker Mic Array v2.0 — Setup on RPi5 (Ubuntu 24.04)

This documents enabling the **ReSpeaker Mic Array v2.0** as both:
- **Audio input** (4 mics + AEC reference, 6 channels via USB UAC)
- **Audio output** routed out of the ReSpeaker's onboard **3.5mm headphone jack**

The Pi 5 has no analog audio jack of its own (only HDMI), so the ReSpeaker's 3.5mm jack is the cleanest way to get analog audio output for TTS / robot speech without adding another USB audio card.

---

## 1. Hardware Check

Plug the ReSpeaker into one of the RPi5's USB ports (USB 3.0 is fine, USB 2.0 also works) using a **data-capable** USB cable. Verify it enumerates:

```bash
lsusb | grep -i seeed
# Expect: Bus 00x Device 00x: ID 2886:0018 SEEED Technology Co., Ltd ReSpeaker 4 Mic Array (UAC1.0)
```

Then check ALSA sees it as both a capture and playback device:

```bash
arecord -l   # should list "ReSpeaker 4 Mic Array" as a capture card
aplay   -l   # should list "ReSpeaker 4 Mic Array" as a playback card
```

If `aplay -l` only shows `vc4hdmi0` / `vc4hdmi1`, the ReSpeaker isn't enumerating — try a different USB cable / port and re-check `dmesg | tail`.

---

## 2. Run the Setup Script

```bash
chmod +x setup_respeaker.sh
./setup_respeaker.sh
```

What it does:
1. Installs `portaudio19-dev`, `python3-pyaudio`, `alsa-utils`, `pulseaudio-utils`.
2. Detects the ReSpeaker card index from `aplay -l`.
3. Writes `~/.asoundrc` so the ReSpeaker becomes the **default ALSA playback + capture device** — meaning `aplay file.wav` and any pyaudio default-device call will go to/from the ReSpeaker (and audio out exits the 3.5mm jack).
4. If PulseAudio/PipeWire is running, also sets the ReSpeaker as the default sink + source via `pactl`.

After it finishes, plug headphones or a powered speaker into the ReSpeaker's 3.5mm jack.

---

## 3. Smoke Test (no Python needed)

```bash
# Plays a stereo sine sweep — you should hear it through the 3.5mm jack on the ReSpeaker
speaker-test -D default -c 2 -t sine -l 1

# Record 5 s from the ReSpeaker's processed mono channel
arecord -D default -r 16000 -c 1 -f S16_LE -d 5 /tmp/test.wav

# Play it back through the 3.5mm jack
aplay /tmp/test.wav
```

If you hear yourself, you're done with the system layer.

---

## 4. Run the Python Test

```bash
python3 respeaker_test.py
```

What it does:
1. Lists every PortAudio device (good for sanity-checking what pyaudio sees).
2. Auto-detects the ReSpeaker by name for **both** input and output indices.
3. Records 5 s of mono 16 kHz audio from the array.
4. Plays the recording back **through the ReSpeaker's 3.5mm jack** by explicitly opening an output stream on the ReSpeaker output device — *not* through `aplay` defaults — so it works regardless of whether `~/.asoundrc` was written.

Pass `--channels 6` if you want the raw multichannel capture (4 mics + playback ref + processed). Pass `--duration 10` for a longer clip.

---

## 5. Channel Map (v2.0)

When you record with `CHANNELS=6`, the layout is:

| Channel | Content |
|--------:|---------|
| 0 | Processed audio (AEC + beamformed, what you usually want) |
| 1 | Mic 1 (raw) |
| 2 | Mic 2 (raw) |
| 3 | Mic 3 (raw) |
| 4 | Mic 4 (raw) |
| 5 | Playback / AEC reference |

For ASR / wake-word, channel 0 is almost always the right choice. The raw channels are useful for DOA, beamforming experiments, or feeding a custom front-end.

---

## 6. Troubleshooting

**`pyaudio` errors with `Invalid sample rate`** — the device exposes 16000 Hz natively. Don't request 44100 directly on the hardware device; use the `default` plug device or stick to 16 kHz.

**No sound from the 3.5mm jack but `aplay -l` shows the card** — check `alsamixer` (press `F6`, pick the ReSpeaker, raise master volume, unmute with `M`). The XMOS firmware boots with playback muted on some firmware versions.

**Sound goes to HDMI instead of the jack** — `~/.asoundrc` wasn't picked up (sometimes shadowed by a system-wide PipeWire config). Run:
```bash
pactl list short sinks
pactl set-default-sink alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo
```
(Tab-completion will fill in the exact sink name.)

**`arecord` works but pyaudio can't find the device** — pyaudio matches by substring on the device name. Confirm with:
```bash
python3 -c "import pyaudio; p=pyaudio.PyAudio(); [print(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count())]"
```
and adjust the `RESPEAKER_NAME_HINTS` list in `respeaker_test.py` if your kernel reports a different name string.

**Latency on playback** — the v2.0's USB UAC1.0 implementation has ~30–60 ms output latency. Acceptable for TTS, not great for tight real-time loops. If that matters, use a separate USB DAC for output and keep the ReSpeaker as input-only.

---

## 7. Files in This Folder

| File | Purpose |
|------|---------|
| `RESPEAKER_SETUP.md` | This document. |
| `setup_respeaker.sh` | One-shot install + ALSA/Pulse default routing. |
| `respeaker_test.py`  | Records from ReSpeaker, plays out its 3.5mm jack. |
