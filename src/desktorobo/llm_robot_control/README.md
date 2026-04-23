# llm_robot_control

Natural-language control for the desktop robot using OpenAI tool-use + a simple web UI.

## What it does
- Hosts a Flask web UI on `http://<pi-ip>:5000` — open it from any device on the same LAN.
- Sends user text to OpenAI `gpt-4o-mini` with four tools: `drive`, `stop`, `wait`, `get_status`.
- Translates each tool call into `geometry_msgs/Twist` messages on `/cmd_vel`, so it coexists with the joystick stack (either can command the robot at any time).
- Server-side clamping keeps velocities within safe bounds regardless of what the model outputs.

## Setup
1. Install Python deps on the Pi:
   ```bash
   pip3 install --break-system-packages openai flask python-dotenv
   ```
2. Create `~/desktoprobo/.env` with your API key (copy from `.env.example`):
   ```
   OPENAI_API_KEY=sk-...
   ```
   ```bash
   chmod 600 ~/desktoprobo/.env
   ```
3. Build and source:
   ```bash
   cd ~/desktoprobo
   colcon build --packages-select llm_robot_control
   source install/setup.bash
   ```

## Run
Two terminals, one command each:

```bash
# terminal 1 — motors + joystick (existing stack)
./start_controller.sh
```

```bash
# terminal 2 — LLM web UI
ros2 run llm_robot_control llm_controller_node
```

Open `http://<pi-ip>:5000` in a browser. Try "go forward 2 seconds", "turn left 90 degrees", "drive in a square".

Or launch both the motor node and the LLM node together:
```bash
ros2 launch llm_robot_control llm_control_launch.py
```

## Safety
- `MAX_LINEAR = 0.3 m/s`, `MAX_ANGULAR = 1.5 rad/s`, `MAX_DURATION = 10 s` are enforced server-side.
- Every `drive()` ends with an auto-stop (`cmd_vel = 0`).
- The red STOP button in the web UI posts to `/estop` and bypasses the LLM entirely.
- The joystick can always override by publishing a later `cmd_vel` message.

## Tools exposed to the LLM
| Tool | Args | Description |
|------|------|-------------|
| `drive` | `linear` (m/s), `angular` (rad/s), `duration` (s) | Move with given velocities for N seconds, auto-stop at the end. |
| `stop` | — | Immediately publish zero velocity. |
| `wait` | `seconds` | Pause without moving. |
| `get_status` | — | Read current odometry (x, y, theta_deg). |

## Switching to a local LLM (Ollama) later
Change two lines in `llm_controller_node.py`:
```python
self.openai = OpenAI(api_key="ollama", base_url="http://localhost:11434/v1")
MODEL = "qwen2.5:3b"  # or whichever model you pulled
```
No other changes needed — prompt and tool schema are compatible.
