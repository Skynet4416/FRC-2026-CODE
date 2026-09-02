# Gemini Robotics-ER 2 agent

Drives the robot from plain-language instructions, through the `/AIControl`
NetworkTables API. Full guide: [`docs/gemini-agent.md`](../../docs/gemini-agent.md).

```bash
pip install -r requirements.txt
export GEMINI_API_KEY=...            # free key: https://aistudio.google.com/apikey

./gradlew simulateJava               # terminal 1
python3 scripts/gemini_agent/agent.py  # terminal 2
> run the left trench, pick up fuel, and score it
```

| File | Role |
| --- | --- |
| `agent.py` | CLI and model loop. `--task` for one-shot, `--host` for a real robot, `--print-prompt` to see what the model is told. |
| `robot_tools.py` | The six tools and what they write to `/AIControl`. |
| `field_view.py` | Top-down field map - the model's main sense. |
| `nt4.py` | NT4 client, websockets + msgpack only. |
| `camera.py` | One JPEG out of a PhotonVision MJPEG stream. |
| `prompt.py` | The behaviour half of the system prompt; the rest comes from the robot. |
| `selftest.py` | End-to-end test against a fake robot and a fake Gemini. No key needed. |
