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

The intake latches: `INTAKE` lowers it and leaves it down until `STOW_INTAKE` or
`STOP`, and its normal resting state is down everywhere on the field - the robot
folds it by itself only to cross the trench and the bumps, and lowers it again
once clear. Fuel is on the robot's **left flank**, about half a metre out, not
the front, and is only collected by driving over it with the intake down; the
robot clamps you to 1.5 m/s while it is down so the sweep actually works. Three
tools do the fuel work so the model rarely has to drive-and-intake by hand:
`find_fuel()` for where fuel actually is (ground truth, not a camera guess),
`grab_fuel(count)` to chase down a nearby piece the camera can see, and
`collect_fuel(count)` for an autonomous slow sweep across a whole line of it.

The robot also publishes its own playbook: `list_plays()` shows the team's real
autonomous routines (name, purpose, and a step-by-step summary), and
`run_play(name)` replays one whole play on the robot's own pathfinding - the
model's preferred move whenever a situation matches one, since these are the
lanes and speeds the team's own drivers trust rather than something improvised
call by call. `run_action`/`run_play` both take an optional `shoot_seconds` to
size a shot's window (1-15 s) to how much fuel is actually on board, the same
way the real autos vary theirs.

| File | Role |
| --- | --- |
| `agent.py` | CLI and model loop. `--task` for one-shot, `--host` for a real robot, `--print-prompt` to see what the model is told. |
| `robot_tools.py` | The tools and what they write to `/AIControl`: driving, actions, the field/camera views, `find_fuel`/`grab_fuel`/`collect_fuel`, and `list_plays`/`run_play`. |
| `field_view.py` | Top-down field map - the model's main sense. Draws every fuel piece on the field (faint) with camera-seen ones highlighted, plus the field's zones. |
| `nt4.py` | NT4 client, websockets + msgpack only. Reads `Fuel`, `Zones`, `GameBrief`, `Tactics`, `Playbook`, `IntakePolicy` and `HubState`; writes `CollectTarget`, `AutoFoldIntake`, `PlayName` and `ShootSeconds`. |
| `camera.py` | One JPEG out of a PhotonVision MJPEG stream. |
| `prompt.py` | The behaviour half of the system prompt - the intake rules, the fuel tools, the Alliance Shift hub cycle, and how to choose between a play and a hand-built cycle; the rest (game brief, notes, tactics, landmarks, zones) comes from the robot. |
| `selftest.py` | End-to-end test against a fake robot and a fake Gemini. No key needed. |
