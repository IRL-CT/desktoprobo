#!/usr/bin/env python3
"""button_speak.py — press X on PS4 controller, robot speaks."""
import subprocess
import time
from inputs import get_gamepad

SOUND_FILE = "/home/pi/desktoprobo/sounds/hello.wav"
TRIGGER_BUTTON = "BTN_SOUTH"   # X on PS4 (Circle=EAST, Triangle=NORTH, Square=WEST)
COOLDOWN = 1.0                 # seconds between triggers

last_played = 0.0
print(f"Press X to speak. Ctrl+C to quit.")

while True:
    events = get_gamepad()
    for e in events:
        if e.code == TRIGGER_BUTTON and e.state == 1:   # 1 = press, 0 = release
            now = time.time()
            if now - last_played >= COOLDOWN:
                subprocess.Popen(["paplay", SOUND_FILE])
                last_played = now
                print("[SPEAK] hello")
