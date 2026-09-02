# /AIControl demo recorder

Drives the simulated robot through the `/AIControl/` NetworkTables API and renders
the run to an MP4 over PathPlanner's official 2026 field image.

Nothing here is part of the robot program - it is a plain NT4 client, exactly the
shape an LLM tool-call layer (Gemini Robotics ER 2 and friends) would take.

## Running it

```bash
pip install websockets msgpack matplotlib imageio imageio-ffmpeg pillow
curl -sSLo /tmp/field26.png \
  https://raw.githubusercontent.com/mjansen4857/pathplanner/main/images/field26.png

./gradlew simulateJava          # in one terminal
python3 scripts/ai_demo/record_demo.py   # drives the robot, writes /tmp/demo_frames.pkl
python3 scripts/ai_demo/render_demo.py   # writes /tmp/ai_control_demo.mp4
```

`record_demo.py` writes only `TargetPose`, `ActionTrigger`, `MaxSpeed` and
`MaxAccel`, and reads back pose, status and the simulated fuel - so it doubles as a
worked example of the API.
