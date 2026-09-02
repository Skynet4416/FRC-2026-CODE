# AI control demo

`ai-control-demo.mp4` is a 45 second recording of the simulated robot being driven
entirely through the `/AIControl/` NetworkTables API - no joystick, no auto routine.
Each caption is the natural-language instruction an LLM would be given, with the
NetworkTables call it turns into underneath.

What it shows, in order:

1. **Line up in the trench lane** - `TargetPose` starts a PathPlanner pathfind.
2. **Run the trench and pick up fuel** - a path and `ActionTrigger INTAKE` at once;
   paths and mechanism actions run in independent slots.
3. **Get in range and shoot** - `SHOOT_FUEL` raises `RotationLocked` (red ring): the
   shooter owns the heading and the heading in `TargetPose` is ignored.
4. **Shoot on the move** - the same action layered on a running path, so the robot
   keeps driving while it feeds fuel and holds the launch heading.
5. **Charge the bump** - `MaxSpeed` raised to 4.5 m/s, because the bumps only give
   way to a running start.
6. **Stop** - `ActionTrigger STOP` cancels the path and the action.

Orange dots are simulated fuel in flight (MapleSim + the team's fuel physics sim),
the blue line is the active PathPlanner trajectory, the green X the requested pose.
Field image is PathPlanner's official 2026 field.

The recorder and renderer live in `scripts/ai_demo/` - the recorder is a plain NT4
client, so it is also a worked example of the API.
