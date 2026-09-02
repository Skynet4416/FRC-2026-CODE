# Docs

- **[ai-control.md](ai-control.md)** - the `/AIControl` NetworkTables API: topics,
  actions, behaviour, LLM tool schema, tuning and troubleshooting.
- **ai-control-demo.mp4** - see below.

## AI control demo

`ai-control-demo.mp4` is a 44 second recording of the simulated robot being driven
entirely through the `/AIControl/` NetworkTables API - no joystick, no auto routine.
Each caption is the natural-language instruction an LLM would be given, with the
NetworkTables call it turns into underneath. The counter in the corner is the fuel
physics sim's hub score, so the shots in the video are shots that actually went in.

What it shows, in order:

1. **Line up in the trench lane** - `TargetPose` starts a PathPlanner pathfind.
2. **Run the trench and pick up fuel** - a path and `ActionTrigger INTAKE` at once;
   paths and mechanism actions run in independent slots.
3. **Shoot the fuel** - triggered from out in the neutral zone, where a shot is only
   a lob pass. The bridge drives back into our alliance zone, behind the hub, and
   shoots from there; the score starts climbing.
4. **Keep shooting while you move** - `SHOOT_ON_THE_MOVE` translates across the
   alliance zone on the same launch drive the driver's right trigger uses, so the
   shot is aimed and led for the robot's own velocity while it scores.
5. **Charge the bump** - `MaxSpeed` raised to 4.5 m/s, because the bumps only give
   way to a running start.
6. **Stop** - `ActionTrigger STOP` cancels the path and the action.

Orange dots are simulated fuel in flight (MapleSim + the team's fuel physics sim),
the blue line is the active PathPlanner trajectory, the green X the requested pose,
and the red ring means a shooting action holds the drivetrain. Field image is
PathPlanner's official 2026 field.

The recorder and renderer live in `scripts/ai_demo/` - the recorder is a plain NT4
client, so it is also a worked example of the API.
