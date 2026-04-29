"""
LLM Robot Control Node
- Publishes geometry_msgs/Twist to /cmd_vel (coexists with joystick stack)
- Hosts Flask web UI on 0.0.0.0:5000
- Routes user text through OpenAI tool-use; tool calls -> /cmd_vel
- Server-side clamping prevents unsafe velocities regardless of LLM output
"""

import json
import math
import os
import collections
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from flask import Flask, jsonify, request, send_from_directory
from openai import OpenAI
from dotenv import load_dotenv


MAX_LINEAR = 0.3
MAX_ANGULAR = 1.5
MAX_DURATION = 10.0
MIN_DURATION = 0.05

MODEL = "gpt-4o-mini"
WEB_PORT = 5000
MAX_HISTORY_MESSAGES = 20
EVENT_LOG_MAX = 200    # max events kept in ring buffer for the UI

# Pricing for gpt-4o-mini (USD per token, as of 2026)
PRICE_PER_INPUT_TOKEN  = 0.15  / 1_000_000
PRICE_PER_OUTPUT_TOKEN = 0.60  / 1_000_000

SYSTEM_PROMPT = """You are controlling a small two-wheeled differential-drive desktop robot.

You have these tools:
- drive(linear, angular, duration): move with given velocities for `duration` seconds, then auto-stop.
    linear  (m/s): +forward, -backward. Reasonable range: -0.25 to 0.25.
    angular (rad/s): +LEFT (counter-clockwise), -RIGHT. Reasonable range: -1.2 to 1.2.
    duration (s): 0.1 to 10.
- stop(): immediately cut velocity to zero.
- wait(seconds): pause without moving.
- get_status(): return odometry pose (x, y, theta_deg).

Rules:
- Start slow. First move: linear ~ 0.15, duration ~ 1-2 s.
- 90-deg turn at angular=1.0 rad/s -> duration ~ 1.57 s.
- 180-deg turn -> ~ 3.14 s at the same angular.
- Never combine linear >= 0.2 with angular >= 1.0 (skidding).
- If user says "stop" or anything urgent, call stop() first.
- Chain tool calls for multi-step instructions before replying.
- Reply in English (no Chinese characters).
- For motion commands (drive/stop/wait), reply with JUST a short acknowledgement: "ok" or "done" or "stopping" — under 5 words, no narration.
- For questions or status (get_status, "where are you", "what can you do"), give one short informative sentence.
- The shorter the better — assume the user is impatient and watching the robot move.
"""


TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "drive",
            "description": "Drive with given linear/angular velocity for duration seconds, then auto-stop.",
            "parameters": {
                "type": "object",
                "properties": {
                    "linear": {"type": "number", "description": "m/s, +forward/-backward"},
                    "angular": {"type": "number", "description": "rad/s, +left/-right"},
                    "duration": {"type": "number", "description": "seconds 0.1-10"},
                },
                "required": ["linear", "angular", "duration"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop",
            "description": "Immediately stop the robot.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "wait",
            "description": "Wait N seconds without moving.",
            "parameters": {
                "type": "object",
                "properties": {"seconds": {"type": "number"}},
                "required": ["seconds"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_status",
            "description": "Read current odometry (x, y, theta_deg).",
            "parameters": {"type": "object", "properties": {}},
        },
    },
]


def _clamp(val, lo, hi):
    return max(lo, min(hi, float(val)))


class LLMControllerNode(Node):
    def __init__(self):
        super().__init__("llm_controller")

        env_path = Path.home() / "desktoprobo" / ".env"
        if env_path.exists():
            load_dotenv(env_path)
            self.get_logger().info(f"Loaded API key from {env_path}")
        else:
            load_dotenv()
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY not set. Put it in ~/desktoprobo/.env")
            raise RuntimeError("missing OPENAI_API_KEY")
        self.openai = OpenAI(api_key=api_key)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self._last_odom = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}

        self.history = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._chat_lock = threading.Lock()
        self._cmd_lock = threading.Lock()

        # Live event log for the UI (wake / user / tool_call / tool_result / assistant / system)
        self.events = collections.deque(maxlen=EVENT_LOG_MAX)
        self._event_lock = threading.Lock()
        self._next_event_id = 0

        # Voice listening can be paused from the UI
        self._listening_enabled = True

        # Cost / token accounting
        self._tokens_in = 0
        self._tokens_out = 0
        self._cost_usd = 0.0

        self._start_web_server()
        self.get_logger().info(f"LLM controller ready. Open http://<pi-ip>:{WEB_PORT}")

    def _log_event(self, kind, text, **extra):
        """Append an event to the live event log for the UI."""
        with self._event_lock:
            ev = {
                "id": self._next_event_id,
                "ts": time.time(),
                "kind": kind,
                "text": str(text),
            }
            ev.update(extra)
            self._next_event_id += 1
            self.events.append(ev)

    def _odom_cb(self, msg: Odometry):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        self._last_odom = {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "theta_deg": round(math.degrees(theta), 1),
        }

    def _publish_twist(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        with self._cmd_lock:
            self.cmd_pub.publish(t)

    def _drive_for(self, linear, angular, duration):
        linear = _clamp(linear, -MAX_LINEAR, MAX_LINEAR)
        angular = _clamp(angular, -MAX_ANGULAR, MAX_ANGULAR)
        duration = _clamp(duration, MIN_DURATION, MAX_DURATION)
        end = time.time() + duration
        while time.time() < end:
            self._publish_twist(linear, angular)
            time.sleep(0.1)
        self._publish_twist(0.0, 0.0)
        return {"ok": True, "linear": linear, "angular": angular, "duration": round(duration, 2)}

    def _stop(self):
        for _ in range(3):
            self._publish_twist(0.0, 0.0)
            time.sleep(0.05)
        return {"ok": True, "stopped": True}

    def _exec_tool(self, name, args):
        try:
            if name == "drive":
                return self._drive_for(args.get("linear", 0.0), args.get("angular", 0.0), args.get("duration", 1.0))
            if name == "stop":
                return self._stop()
            if name == "wait":
                secs = _clamp(args.get("seconds", 0.5), 0.0, 30.0)
                time.sleep(secs)
                return {"ok": True, "waited": secs}
            if name == "get_status":
                return {"ok": True, **self._last_odom}
            return {"ok": False, "error": f"unknown tool {name}"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def handle_user_message(self, user_text, source="text"):
        with self._chat_lock:
            turn_start = time.time()
            self.history.append({"role": "user", "content": user_text})
            self._log_event("user", user_text, source=source)
            if len(self.history) > MAX_HISTORY_MESSAGES + 1:
                self.history = [self.history[0]] + self.history[-MAX_HISTORY_MESSAGES:]

            for round_idx in range(6):
                resp = self.openai.chat.completions.create(
                    model=MODEL, messages=self.history, tools=TOOLS,
                )
                # accumulate token usage
                try:
                    usage = resp.usage
                    self._tokens_in += usage.prompt_tokens
                    self._tokens_out += usage.completion_tokens
                    self._cost_usd += (usage.prompt_tokens * PRICE_PER_INPUT_TOKEN
                                       + usage.completion_tokens * PRICE_PER_OUTPUT_TOKEN)
                except Exception:
                    pass
                msg = resp.choices[0].message
                self.history.append(msg.model_dump(exclude_none=True))

                if not msg.tool_calls:
                    reply = msg.content or "(no reply)"
                    self._log_event("assistant", reply)
                    self._log_event("stats",
                        f"Latency {time.time()-turn_start:.1f}s  ·  Total ${self._cost_usd:.4f}  ·  tokens {self._tokens_in}↑/{self._tokens_out}↓",
                        duration=round(time.time()-turn_start, 2),
                        cost_usd=round(self._cost_usd, 6),
                        tokens_in=self._tokens_in,
                        tokens_out=self._tokens_out)
                    return reply

                # If the model also produced a "thinking out loud" content alongside tool calls,
                # surface it so the user sees the reasoning step.
                if msg.content:
                    self._log_event("thought", msg.content)

                for tc in msg.tool_calls:
                    name = tc.function.name
                    try:
                        args = json.loads(tc.function.arguments or "{}")
                    except json.JSONDecodeError:
                        args = {}
                    self.get_logger().info(f"tool call: {name}({args})")
                    self._log_event("tool_call", f"{name}({args})", tool=name, args=args)
                    result = self._exec_tool(name, args)
                    self._log_event("tool_result", json.dumps(result, ensure_ascii=False), result=result)
                    self.history.append({
                        "role": "tool",
                        "tool_call_id": tc.id,
                        "content": json.dumps(result),
                    })
            self._log_event("system", "(max tool rounds reached)")
            return "(max tool rounds reached)"

    def _start_web_server(self):
        app = Flask(__name__, static_folder=None)
        web_dir = Path(__file__).parent / "web"
        if not (web_dir / "index.html").exists():
            # iterate all entries in AMENT_PREFIX_PATH to find installed share dir
            candidates = []
            for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
                if not p:
                    continue
                c = Path(p) / "share" / "llm_robot_control" / "web"
                candidates.append(c)
                if (c / "index.html").exists():
                    web_dir = c
                    break
            else:
                # also try a direct known install location
                fallback = Path.home() / "desktoprobo" / "install" / "llm_robot_control" / "share" / "llm_robot_control" / "web"
                if (fallback / "index.html").exists():
                    web_dir = fallback
                else:
                    self.get_logger().error(f"web dir not found. checked: {candidates} and {fallback}")
        self.get_logger().info(f"serving web from {web_dir}")

        @app.route("/")
        def index():
            return send_from_directory(str(web_dir), "index.html")

        @app.route("/chat", methods=["POST"])
        def chat():
            data = request.get_json(force=True) or {}
            user = data.get("message", "").strip()
            source = data.get("source", "text")
            if not user:
                return jsonify({"reply": "(empty)"}), 400
            try:
                reply = self.handle_user_message(user, source=source)
            except Exception as e:
                self.get_logger().error(f"chat error: {e}")
                return jsonify({"reply": f"error: {e}"}), 500
            return jsonify({"reply": reply, "odom": self._last_odom})

        @app.route("/estop", methods=["POST"])
        def estop():
            self._stop()
            return jsonify({"ok": True})

        @app.route("/reset", methods=["POST"])
        def reset():
            with self._chat_lock:
                self.history = [{"role": "system", "content": SYSTEM_PROMPT}]
            with self._event_lock:
                self.events.clear()
                self._next_event_id = 0
            self._log_event("system", "Conversation reset")
            return jsonify({"ok": True})

        @app.route("/event", methods=["POST"])
        def event_post():
            data = request.get_json(force=True) or {}
            kind = data.get("kind", "system")
            text = data.get("text", "")
            extra = {k: v for k, v in data.items() if k not in ("kind", "text")}
            self._log_event(kind, text, **extra)
            return jsonify({"ok": True})

        @app.route("/events", methods=["GET"])
        def events_get():
            try:
                since_id = int(request.args.get("since_id", -1))
            except ValueError:
                since_id = -1
            with self._event_lock:
                evs = [e for e in self.events if e["id"] > since_id]
            return jsonify({
                "events": evs,
                "next_since_id": evs[-1]["id"] if evs else since_id,
                "listening_enabled": self._listening_enabled,
                "cost_usd": round(self._cost_usd, 6),
                "tokens_in": self._tokens_in,
                "tokens_out": self._tokens_out,
            })

        @app.route("/listening", methods=["GET", "POST"])
        def listening():
            if request.method == "POST":
                data = request.get_json(force=True) or {}
                self._listening_enabled = bool(data.get("enabled", True))
                self._log_event("system",
                    f"Voice listening: {'enabled' if self._listening_enabled else 'paused'}")
            return jsonify({"enabled": self._listening_enabled})

        @app.route("/export", methods=["GET"])
        def export_get():
            """Return everything for download: events + chat history + stats."""
            with self._event_lock:
                evs = list(self.events)
            with self._chat_lock:
                hist = list(self.history)
            return jsonify({
                "exported_at": time.time(),
                "events": evs,
                "chat_history": hist,
                "stats": {
                    "cost_usd": round(self._cost_usd, 6),
                    "tokens_in": self._tokens_in,
                    "tokens_out": self._tokens_out,
                },
            })

        def _run():
            app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)

        t = threading.Thread(target=_run, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = LLMControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
