# desktoprobo

A differential-drive desktop robot (evolved from [TiltyBot](https://github.com/imandel/tiltybot)) for Human-Robot Interaction (HRI) research. The platform runs ROS 2 Jazzy on a Raspberry Pi 5 with Dynamixel XL330 motors and is targeted at the ICRA 2027 HRI track.

---

## 1. Hardware Requirements

| Component | Notes |
|---|---|
| **Raspberry Pi 5** | 64-bit, running Ubuntu 24.04 LTS (Noble Numbat). Verify with `uname -m` — must report `aarch64`. The Pi 5 has **no analog audio jack**; analog audio output goes through the ReSpeaker's 3.5mm jack instead. |
| **2× Dynamixel XL330-M288-T (or similar)** | Factory baud defaults to **57600**, not 115200. Reconfigured to 115200 for this project (control table address 8, value 2). |
| **Robotis U2D2** | USB-to-TTL adapter. Uses FTDI FT232H (USB VID:PID `0x0403:0x6014`). Enumerates as `/dev/ttyUSB0`. |
| **PS4 DualShock 4 controller** | Used for teleop. Works over USB (plug-and-play) or Bluetooth (pair + **trust** + connect). |
| **ReSpeaker Mic Array v2.0** | 4 mics + AEC reference (6 channels total) for the multimodal sensing layer. Also exposes a USB UAC1.0 stereo playback endpoint that drives a **3.5mm headphone jack on the board** — used as the system's analog audio output (TTS, alerts, robot speech). |
| **Power** | **Do NOT share a single power-bank port between the RPi5 and the motors.** Voltage sag under combined load causes motor hardware errors and brownouts. Use separate high-current ports or separate banks. |
| **USB cable for U2D2** | Must be a **data-capable** cable, not charge-only. Charge-only cables are the #1 cause of the U2D2 not appearing in `lsusb`/`dmesg`. |
| **JST-EH cables** | Loose JST seating mimics baud-rate and firmware bugs with "Incorrect status packet" errors. Always reseat before deep debugging. |

---

## 2. Installing Ubuntu 24.04 on the RPi5

Flash a 64-bit Ubuntu 24.04 image (Raspberry Pi Imager → Ubuntu Server 24.04 LTS or Desktop 24.04). After first boot:

```bash
sudo apt update && sudo apt upgrade -y
```

Enable SSH if you're running headless:

```bash
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
hostname -I    # find the Pi's IP to connect from another machine
```

From the laptop: `ssh pi@<ip-address>`.

---

## 3. Installing ROS 2 Jazzy on Ubuntu 24.04

ROS 2 Jazzy Jalisco is the only tier-1 ROS 2 release for Ubuntu 24.04 Noble.

### 3.1 Set the locale

ROS 2 needs a UTF-8 locale.

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify
```

### 3.2 Enable the Universe repo and install the ROS 2 apt source

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 3.3 Install ROS 2 Jazzy and development tools

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-dev-tools
```

### 3.4 Install the project-specific ROS packages

```bash
sudo apt install -y \
    ros-jazzy-joy \
    ros-jazzy-joy-teleop \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs
```

If `tf_transformations` still can't be imported, also install its Python backend:

```bash
pip install transforms3d --break-system-packages
```

### 3.5 Source the ROS 2 environment

Add this to `~/.bashrc` so every new shell sees `ros2`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 --help   # sanity check
```

> **Note on `~/ros2_jazzy`:** If you previously built ROS 2 from source into `~/ros2_jazzy`, source both — the apt install first, then the local workspace:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/ros2_jazzy/install/setup.bash
> ```

---

## 4. Python / System Dependencies

Ubuntu 24.04 ships Python 3.12 with PEP-668 "externally managed" protection. For a dedicated robotics machine, the simplest path is `--break-system-packages`:

```bash
pip install dynamixel-sdk --break-system-packages
pip install evdev          --break-system-packages
pip install pyaudio        --break-system-packages
```

System packages the above depend on (motor + controller + audio stack):

```bash
sudo apt install -y \
    python3-pip \
    python3-pyaudio \
    portaudio19-dev \
    alsa-utils \
    pulseaudio-utils \
    libasound2-plugins \
    git
```

If you prefer a venv:

```bash
python3 -m venv ~/robot-env
source ~/robot-env/bin/activate
pip install dynamixel-sdk evdev pyaudio
```

---

## 5. Hardware Setup & Persistent Fixes

These are one-time configurations that have bitten this project before. Do them up front on every new Pi.

### 5.1 U2D2 latency timer (critical)

The FTDI USB serial latency timer defaults to **16 ms**, which corrupts Dynamixel status packets at 115200 and above (`[TxRxResult] Incorrect status packet!`). It must be set to **1 ms**.

The permanent fix is a udev rule — no `sudo` at runtime, set automatically on every plug-in:

```bash
sudo tee /etc/udev/rules.d/99-u2d2.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the U2D2, then verify:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # should print 1
```

> Do **not** write the latency timer at runtime from inside a ROS node — if the port is already open, the write returns `Input/output error`.

### 5.2 `input` group for headless controller access

On a GUI session, `logind` grants ACL access to `/dev/input/js*` automatically. In a **headless** setup it does not, so `joy_node` silently publishes nothing on `/joy`. Add your user to the `input` group:

```bash
sudo usermod -aG input $USER
sudo reboot    # full reboot required — re-login alone is not enough in headless mode
```

Verify:

```bash
groups                  # should include 'input'
ls -la /dev/input/js0   # should be readable by 'input' group
```

### 5.3 `dialout` group for serial port access (if needed)

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

### 5.4 FTDI driver & finding the U2D2 port

The FTDI driver is loaded by default on Ubuntu. Verify:

```bash
lsusb                  # should show 'Future Technology Devices International ... FT232'
lsmod | grep ftdi      # should list ftdi_sio
ls /dev/ttyUSB*        # U2D2 enumerates as /dev/ttyUSB0
```

If nothing shows up after plugging in the U2D2: try a different USB cable (data-capable), and watch `sudo dmesg -w` during replug.

---

## 6. Bluetooth Pairing for the PS4 Controller (headless)

```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# scan on
# hold PS + SHARE on the controller until the light bar double-flashes
# wait for the MAC address (AA:BB:CC:DD:EE:FF) to appear
[bluetooth]# pair    AA:BB:CC:DD:EE:FF
[bluetooth]# trust   AA:BB:CC:DD:EE:FF     # <-- this step is critical; GUI does it automatically
[bluetooth]# connect AA:BB:CC:DD:EE:FF
[bluetooth]# exit
```

Confirm:

```bash
ls /dev/input/js0
jstest /dev/input/js0
```

---

## 7. Audio / ReSpeaker Mic Array v2.0

The ReSpeaker provides both **audio input** (4 mics + AEC reference, 6 channels via USB UAC1.0) and **audio output** through the 3.5mm jack on the board. Because the Pi 5 has no built-in analog audio jack, the ReSpeaker is also the system's analog audio output for TTS, alerts, and robot speech.

All audio scripts and configuration live in `~/desktoprobo/sounds/`:

| File | Purpose |
|---|---|
| `RESPEAKER_SETUP.md` | Detailed setup notes, channel map, and troubleshooting. |
| `setup_respeaker.sh` | One-shot installer + ALSA/PulseAudio default-device routing. |
| `respeaker_test.py`  | End-to-end test: records 5 s from the array, plays it back through the 3.5mm jack. |

### 7.1 Verify the device enumerates

Plug the ReSpeaker into a USB port with a **data-capable** cable, then:

```bash
lsusb | grep -i seeed
# Expect: ID 2886:0018 SEEED Technology Co., Ltd ReSpeaker 4 Mic Array (UAC1.0)

arecord -l   # should list "ReSpeaker 4 Mic Array" as a capture card
aplay   -l   # should list "ReSpeaker 4 Mic Array" as a playback card
```

If `aplay -l` only shows `vc4hdmi0` / `vc4hdmi1`, the ReSpeaker isn't enumerating. Try a different cable / port and check `dmesg | tail`.

### 7.2 Run the setup script

```bash
cd ~/desktoprobo/sounds
chmod +x setup_respeaker.sh
./setup_respeaker.sh
```

What it does:

1. Installs `portaudio19-dev`, `python3-pyaudio`, `alsa-utils`, `pulseaudio-utils`, `libasound2-plugins`.
2. Auto-detects the ReSpeaker's ALSA card index from `aplay -l`.
3. Writes `~/.asoundrc` so the ReSpeaker becomes the default ALSA playback + capture device.
4. If PulseAudio/PipeWire is running, sets the ReSpeaker as the default sink + source via `pactl`.

After it finishes, plug headphones or a powered speaker into the ReSpeaker's 3.5mm jack.

### 7.3 Smoke test (no Python)

```bash
# Hear a stereo sine via the 3.5mm jack
speaker-test -D default -c 2 -t sine -l 1

# Record 5 s mono, then play it back
arecord -D default -r 16000 -c 1 -f S16_LE -d 5 /tmp/test.wav
aplay /tmp/test.wav
```

If you hear yourself, the system layer is good.

### 7.4 Python test

```bash
cd ~/desktoprobo/sounds
python3 respeaker_test.py                  # default: 1 ch, 5 s
python3 respeaker_test.py --channels 6     # raw multi-channel capture
python3 respeaker_test.py --list-only      # just dump device list and exit
```

The script auto-detects the ReSpeaker for both input and output, records, and pins playback to the ReSpeaker output device explicitly so it works regardless of whether `~/.asoundrc` took effect.

### 7.5 v2.0 channel map

When recording with `--channels 6`, the layout is:

| Channel | Content |
|--------:|---------|
| 0 | Processed audio (AEC + beamformed) — **use this for ASR / wake-word** |
| 1 | Mic 1 (raw) |
| 2 | Mic 2 (raw) |
| 3 | Mic 3 (raw) |
| 4 | Mic 4 (raw) |
| 5 | Playback / AEC reference |

---

## 8. Building the `desktoprobo` Workspace

Clone the workspace onto the Pi:

```bash
cd ~
git clone https://github.com/imandel/tiltybot.git desktoprobo
# or use the desktoprobo-master layout that already contains src/desktorobo/
```

Expected layout:

```
~/desktoprobo/
├── build/          (generated)
├── install/        (generated)
├── log/            (generated)
├── sounds/
│   ├── RESPEAKER_SETUP.md
│   ├── setup_respeaker.sh
│   └── respeaker_test.py
└── src/
    └── desktorobo/
        ├── joy_teleop_keymapping/
        │   ├── joy_teleop_keymapping/
        │   │   └── keymapping_node.py
        │   ├── launch/mapping_launch.py
        │   ├── package.xml
        │   └── setup.py
        ├── mobile_robot_control/
        │   ├── launch/mobile_robot_launch.py
        │   ├── mobile_robot_control/
        │   │   ├── mobile_robot_control_node.py
        │   │   └── odrive_command.py
        │   ├── package.xml
        │   └── setup.py
        └── README.md
```

Build with colcon **from the workspace root** (not from `src/`):

```bash
cd ~/desktoprobo
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means future edits to the `.py` files don't require a rebuild — just restart the node.

---

## 9. Running the Robot

Every new terminal needs the two `source` lines. Bake them into a startup script:

```bash
cat > ~/start_controller.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/desktoprobo/install/setup.bash
ros2 launch mobile_robot_control mobile_robot_launch.py
EOF
chmod +x ~/start_controller.sh
```

Launch everything (robot control + joy + keymapping) with one command:

```bash
~/start_controller.sh
```

Or manually:

```bash
ros2 launch mobile_robot_control mobile_robot_launch.py
```

### Controller mapping (from `keymapping_node.py`)

| Input | Action |
|---|---|
| **L1** (button 4) — hold | Safety deadman. Required for motion. Left stick Y = linear, right stick (axes[3]) = angular. |
| **R1** (button 5) — hold | Auto forward-burst ramp. |
| Neither held | Publishes zero velocity (stops). |

### Useful topics for debugging

```bash
ros2 topic echo /joy          # raw controller input
ros2 topic echo /cmd_vel      # after keymapping
ros2 node list
ros2 param get /joy device_id
```

---

## 10. Motor Configuration Reference

| Parameter | Value | Control table address |
|---|---|---|
| Protocol | 2.0 | — |
| Baud rate | 115200 (reconfigured from factory 57600) | 8 |
| Operating mode | 1 (velocity) | 11 |
| Torque enable | — | 64 |
| Goal velocity | — | 104 |
| Present position | — | 132 |
| `MAX_SPEED` (safety cap) | 300 | — |
| Left motor ID | 1 | — |
| Right motor ID | 2 | — |
| Differential drive mixing | `M1 = linear + angular`, `M2 = linear − angular` | — |

Order of operations when initializing a motor (critical — operating-mode writes are silently ignored while torque is on):

1. Torque OFF (`write1ByteTxRx(..., ADDR_TORQUE_ENABLE, 0)`)
2. Set operating mode (`write1ByteTxRx(..., ADDR_OPERATING_MODE, 1)`)
3. Torque ON
4. Ping each motor before configuring — skip missing ones with a warning instead of crashing the port.

---

## 11. Known Gotchas (Learned the Hard Way)

- **`rclpy.time.Time` ≠ `builtin_interfaces.msg.Time`.** When assigning to `odom_msg.header.stamp` or `tf_msg.header.stamp`, call `.to_msg()` on the `rclpy.time.Time` object. Direct assignment triggers a fatal `builtin_interfaces/msg/_time_s.c` assertion and kills the node.
- **Headless vs GUI input devices.** GUI sessions get automatic ACL grants on `/dev/input/js*` via `logind`. Headless mode requires explicit `input` group membership + full reboot.
- **U2D2 latency timer can't be written at runtime** if a process already has the port open (returns `Input/output error`). Use the udev rule.
- **Motor factory baud is 57600.** If you've never run the Arduino/ESP32 motor-setup sketch, the motors are still at 57600 — always probe at the factory default before assuming 115200.
- **JST cable seating** causes corrupted packets that look identical to firmware or baud-rate bugs. Reseat before deep debugging.
- **Session-scoped Claude files.** Files generated in one Claude session are not persisted to the next. Keep `odrive_command.py` and other node source files under git version control.
- **`colcon build` must run from the workspace root** (`~/desktoprobo`), not from `src/`.
- **Bluetooth `trust` is not optional.** `pair` + `connect` alone leave the controller half-registered and `/dev/input/js0` never appears.
- **Audio routes to HDMI instead of the 3.5mm jack.** PipeWire/PulseAudio can shadow the `~/.asoundrc` defaults. Re-run the relevant block of `setup_respeaker.sh`, or manually: `pactl set-default-sink alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo` (tab-completion finds the exact sink name).
- **ReSpeaker plays no sound even though `aplay -l` shows the card.** XMOS firmware boots with playback muted on some versions. Open `alsamixer`, press `F6`, select the ReSpeaker, raise master, unmute with `M`.
- **pyaudio can't find the ReSpeaker by name.** Kernel name strings vary by firmware. Run `python3 -c "import pyaudio; p=pyaudio.PyAudio(); [print(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count())]"` and update `RESPEAKER_NAME_HINTS` in `respeaker_test.py` accordingly.

---

## 12. Research Context

This platform supports a graduated-autonomy HRI framework where the robot adapts to user intent over time through multimodal signals. The pipeline has two layers:

- **Signal recognition:** ASR, OpenPose, OpenFace, LiDAR, RGB-D
- **Intention inference:** Mixture-of-Experts + RAG

Study design: Wizard-of-Oz across three subtasks, then a deployed interactive system for a within-subjects user study (target n=16).

| Milestone | Target |
|---|---|
| Complete WoZ study | ~May 2026 |
| Deploy interactive system & run user study | ~Aug 2026 |
| ICRA 2026 HRII Workshop (stepping stone) | April 2026 |
| ICRA 2027 HRI track submission | ~Sep 2026 |
| ICRA 2027, Seoul | May 2027 |

---

## 13. Quick Reference: Useful Commands

```bash
# Verify ROS 2 install
ros2 --help
printenv | grep ROS

# U2D2 / motor sanity
ls /dev/ttyUSB*
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # should be 1
python3 -c "import dynamixel_sdk; print('OK')"

# Controller sanity
ls /dev/input/js*
jstest /dev/input/js0
groups                                                   # should include 'input'

# Audio / ReSpeaker
arecord -l                                               # list capture devices
aplay -l                                                 # list playback devices
pactl list short sinks                                   # list PulseAudio sinks
speaker-test -D default -c 2 -t sine -l 1                # test 3.5mm jack
cd ~/desktoprobo/sounds && python3 respeaker_test.py     # full mic+jack round-trip

# Kernel diagnostics
dmesg | grep -iE "ftdi|usb|tty" | tail -20
sudo dmesg -w                                            # live kernel log during replug

# ROS 2 debugging
ros2 node list
ros2 topic list
ros2 topic echo /joy
ros2 topic echo /cmd_vel

# Build/rebuild
cd ~/desktoprobo && colcon build --symlink-install
source install/setup.bash
```

---

## License & Attribution

Built on the TiltyBot platform: https://github.com/imandel/tiltybot
