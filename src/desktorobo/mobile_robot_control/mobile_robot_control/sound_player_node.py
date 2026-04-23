#!/usr/bin/env python3
import subprocess
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# PS4 button indices in the standard joy mapping
BTN_X        = 0
BTN_CIRCLE   = 1
BTN_TRIANGLE = 2
BTN_SQUARE   = 3

SOUND_MAP = {
    BTN_X: "/home/pi/desktoprobo/sounds/hello.wav",
    # add more: BTN_CIRCLE: "/home/pi/desktoprobo/sounds/forward.wav",
}

COOLDOWN = 1.0  # seconds

class SoundPlayer(Node):
    def __init__(self):
        super().__init__("sound_player")
        self.sub = self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.last_buttons = []
        self.last_played = 0.0
        self.get_logger().info("sound_player ready. Listening on /joy.")

    def on_joy(self, msg: Joy):
        # detect rising edges (0 -> 1) so we only fire once per press
        if len(self.last_buttons) != len(msg.buttons):
            self.last_buttons = [0] * len(msg.buttons)

        now = time.time()
        for idx, pressed in enumerate(msg.buttons):
            was_pressed = self.last_buttons[idx]
            if pressed and not was_pressed and idx in SOUND_MAP:
                if now - self.last_played >= COOLDOWN:
                    subprocess.Popen(["paplay", SOUND_MAP[idx]])
                    self.last_played = now
                    self.get_logger().info(f"[SPEAK] button {idx}")

        self.last_buttons = list(msg.buttons)

def main():
    rclpy.init()
    node = SoundPlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
