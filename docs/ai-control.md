# `/AIControl` - driving the robot from an LLM

A NetworkTables API that lets an external agent - a Gemini Robotics ER 2 tool call, a
script, anything that speaks NT4 - drive the robot and run its mechanisms, and read
back enough state to decide what to do next.

Everything lives under the `/AIControl/` table, so it shows up as one folder in
AdvantageScope, Elastic, or Glass. Every topic is published in the constructor, so the
whole API is visible the moment robot code starts, before any command is sent.

- [Quick start](#quick-start)
- [Topics](#topics)
- [Actions](#actions)
- [How it behaves](#how-it-behaves)
- [Vision](#vision)
- [Wiring an LLM to it](#wiring-an-llm-to-it)
- [Tuning](#tuning)
- [Troubleshooting](#troubleshooting)
- [Files](#files)

## Quick start

```bash
./gradlew simulateJava
```

The robot publishes `/AIControl/` on the usual NT ports (NT4 5810, NT3 1735). Point
AdvantageScope at `localhost`, open the `/AIControl` folder, and write to it - from
AdvantageScope, from Elastic, or from a client of your own:

```python
# minimal NT4 client: drive to (3.6, 7.4) facing -90, then intake
await ws.send(msgpack.packb([target_pose_pubuid, now_us(), 17, [3.6, 7.4, -90.0]]))
await ws.send(msgpack.packb([action_pubuid,      now_us(),  4, "INTAKE"]))
```

`scripts/ai_demo/record_demo.py` is a complete working client; `docs/ai-control-demo.mp4`
is what it produces.

In simulation the bridge enables the robot itself while it has work to do, because an
agent has no driver station. On a real robot it never does - only the driver station
enables the robot.

## Topics

### Written by the agent

| Topic | Type | Meaning |
| --- | --- | --- |
| `TargetPose` | `double[3]` | `[x m, y m, heading deg]`, blue-origin field coordinates. Writing it starts a PathPlanner pathfind to that pose. |
| `ActionTrigger` | `String` | Name of an action from `AvailableActions`. `STOP` cancels the path and the action. |
| `MaxSpeed` | `double` | Pathfinding speed limit, m/s. Clamped to 0.3 - 4.5. Applies to the next path. |
| `MaxAccel` | `double` | Pathfinding acceleration limit, m/s². Clamped to 0.3 - 8.0. |

### Read by the agent

| Topic | Type | Meaning |
| --- | --- | --- |
| `RobotPose` | `double[3]` | `[x m, y m, heading deg]`, same convention as `TargetPose`. |
| `Navigating` | `boolean` | A pathfinding command is running. |
| `ActionRunning` | `boolean` | A mechanism action is running. |
| `RotationLocked` | `boolean` | A shooting action holds the drivetrain; the heading you asked for does not apply. |
| `InShootingZone` | `boolean` | The robot is in our alliance zone, where a shot scores in the hub. |
| `AtTarget` | `boolean` | The robot is within 0.15 m and 5° of the last requested pose. |
| `ActiveTarget` | `double[3]` | Pose currently being driven to, empty when there is none. |
| `Status` | `String` | One line of human-readable state: `IDLE`, `AT TARGET`, `PATHFINDING to (…)`, `MOVING INTO THE ALLIANCE ZONE (…), then SHOOT_FUEL`, `RUNNING <action> - shooter owns the drivetrain`, `STOPPED`, `ERROR: …`. |
| `LastAction` | `String` | Last action name received, valid or not. |
| `LastError` | `String` | Sticky copy of the last error, so a failure is not lost when `Status` moves on. |
| `AvailableActions` | `String[]` | Every action the robot accepts right now, `STOP` first. |
| `HeadingUnits` | `String` | `degrees`. |
| `Notes` | `String` | The rules above in plain text, meant to be dropped straight into a system prompt. |

Everything is mirrored to AdvantageKit under `AIControl/…` (`Navigating`, `ActionRunning`,
`ShooterOwnsDrivetrain`, `InShootingZone`, `LastAction`, `LastError`, `ActiveTarget`), so a
run can be replayed from the log.

## Actions

| Action | What it does | Takes the drivetrain |
| --- | --- | --- |
| `INTAKE` | Lowers the intake, runs it 4 s, folds it back up. | no |
| `SHOOT_FUEL` | Aims and shoots at the hub for 4 s from a standstill. | yes |
| `SHOOT_ON_THE_MOVE` | Same, for 5 s, while translating across the shooting line. | yes |
| `ALIGN_HUB` | Aims and spins up for 3 s without feeding fuel. | yes |
| `STOP` | Cancels the running path and action. | - |

Actions are registered in `RobotContainer`, so adding one is a line:

```java
aiControlBridge.registerAction("CLIMB", () -> climbCommand());          // agent keeps the drivetrain
aiControlBridge.registerAction("SHOOT_FAR", () -> farShot(3.0), true);  // action drives the robot
```

`AvailableActions` updates itself, so a new action is immediately visible to the agent.

## How it behaves

**Navigation and actions are separate slots.** A new `TargetPose` replaces the running
path; a new `ActionTrigger` replaces the running action. An action that does not take the
drivetrain (`INTAKE`) runs happily on top of a path.

**All travel is PathPlanner.** `AutoBuilder.pathfindToPose` does the distance. Its command
ends when its trajectory timer runs out rather than when the robot arrives, so the bridge
repeats it - each repeat replans from where the robot really is - and inside 0.75 m hands
over to PathPlanner's own holonomic controller, which settles the last few centimetres onto
the pose. Arrivals land within a couple of centimetres.

**Shooting is the robot's own launch drive.** A shooting action runs
`DriveCommands.joystickDriveWhileLaunching`, the command behind the driver's right trigger:
it aims with the launch solution, leads the target for the robot's own velocity, applies the
launcher centre-of-rotation offset, and caps translation at what the shot can tolerate. So a
shooting action cancels the path and drives itself - that is what `RotationLocked` reports.

**Shots are taken from our alliance zone, behind the hub.** `LaunchCalculator` switches to a
lob pass the moment the robot is past the hub, which throws fuel back into our own zone
instead of scoring. A shooting action triggered from outside the zone therefore pathfinds to
a shooting spot 2.6 m in front of the hub first, and shoots when it gets there. `Status` says
so while it is on the way.

**Bumps need a running start.** The bumps beside each hub are not solid, but the bump sim
holds the robot's position until it has the speed to get over. Back off a couple of metres
and cross with `MaxSpeed` at 4+ m/s, or take the flat trench lanes along either side wall.
`navgrid.json` (regenerated for the 2026 field: border, both hubs, four trench walls, four
tower uprights) keeps paths off the things MapleSim actually collides with; the bumps are
deliberately left open so the robot may cross them.

**Failures are reported, not swallowed.** An unreachable pose (blocked, inside an obstacle,
or simply too slow) ends after 20 s with `ERROR: could not reach (x, y) …` in `Status` and
`LastError`. An unknown action name is an error too - check `AvailableActions`.

## Vision

Simulation runs three PhotonVision cameras so an agent can see what the robot sees:

| Camera | Purpose | Streams |
| --- | --- | --- |
| `limelight-right`, `limelight-left` | AprilTags, feeding the pose estimator | raw + processed MJPEG |
| `fuel-cam` | Object detection on the fuel | raw + processed MJPEG |

Streams are advertised under `/CameraPublisher/<name>-raw|-processed/streams` (ports 1181+).
The fuel camera is fed from MapleSim's field pieces plus the shots in flight, nearest 24
within 8 m, and logs what it sees to `Vision/GamePieces/{VisibleCount,Yaws,Pitches,Areas,TargetPoses}`.

Camera resolution, FPS, latency and noise live in `VisionConstants`; the wireframe overlay is
off by default (`simDrawWireframe`) because it is expensive enough to cause loop overruns.

## Wiring an LLM to it

The model never talks to NetworkTables directly. Give it two or three tools that write the
topics above, and feed `Notes`, `AvailableActions`, `RobotPose`, `Status` and `InShootingZone`
back as observations. A tool schema that matches this API:

```json
[
  {"name": "drive_to",
   "description": "Drive the robot to a field pose using PathPlanner pathfinding.",
   "parameters": {"type": "object", "properties": {
     "x": {"type": "number", "description": "metres, 0 at the blue wall"},
     "y": {"type": "number", "description": "metres, 0 at the scoring-table wall"},
     "heading_deg": {"type": "number"},
     "max_speed": {"type": "number", "description": "m/s, 0.3-4.5; 4+ to cross a bump"}},
     "required": ["x", "y", "heading_deg"]}},
  {"name": "run_action",
   "description": "Run a mechanism action. Shooting actions drive the robot into the alliance zone first.",
   "parameters": {"type": "object", "properties": {
     "action": {"type": "string", "enum": ["INTAKE", "SHOOT_FUEL", "SHOOT_ON_THE_MOVE", "ALIGN_HUB", "STOP"]}},
     "required": ["action"]}},
  {"name": "get_state",
   "description": "Read the robot's pose and status.",
   "parameters": {"type": "object", "properties": {}}}
]
```

Two things worth putting in the system prompt, both of which the robot also states in `Notes`:
the agent does not control heading while a shooting action runs, and shots only count from the
alliance zone.

## Tuning

| Constant | Where | Default | What it changes |
| --- | --- | --- | --- |
| `DEFAULT_MAX_SPEED_MPS` / `DEFAULT_MAX_ACCEL_MPS2` | `AIControlBridge` | 2.5 / 2.5 | Pathfinding limits before the agent sets its own. |
| `POSE_TOLERANCE_METERS` / `HEADING_TOLERANCE_DEGREES` | `AIControlBridge` | 0.15 / 5.0 | When a pose counts as reached. |
| `FINAL_APPROACH_RADIUS_METERS` | `AIControlBridge` | 0.75 | Where the pathfinder hands over to the holonomic controller. |
| `NAVIGATION_TIMEOUT_SECONDS` | `AIControlBridge` | 20.0 | How long a path may try before reporting failure. |
| `shootingDistanceMeters` | `RobotContainer` | 2.6 | How far in front of the hub the robot stands to shoot. |
| `shootingZoneMarginMeters` | `RobotContainer` | 0.5 | Margin inside the hub line before a shot counts as a hub shot. |
| `strafeHalfWidth` / `strafeInputScalar` | `RobotContainer` | 1.3 / 0.45 | How far and how fast `SHOOT_ON_THE_MOVE` translates. |
| `simDrawWireframe`, `simCamera*` | `VisionConstants` | see file | Simulated camera quality and cost. |

## Troubleshooting

**Nothing moves.** Check `Status`. `ERROR: PathPlanner AutoBuilder is not configured` means
`Drive` never ran its `AutoBuilder.configure`. On a real robot, also check the robot is
enabled - the bridge only self-enables in simulation.

**The robot creeps and then reports `could not reach`.** It is against something: a target
inside the hub or a trench wall (the pose itself is unreachable), or a bump taken too slowly.
Pick a pose in open carpet, or raise `MaxSpeed`.

**Shots do not score.** Look at `InShootingZone`. From the neutral zone the launch solution is
a pass, not a hub shot - that is exactly what the zone approach exists to prevent, so if the
robot is shooting from out there something is bypassing the bridge.

**Topics are missing in AdvantageScope.** They are created in the constructor, so a missing
`/AIControl` folder means `RobotContainer` never built the bridge, or the client is connected
to the wrong host.

## Files

| File | Role |
| --- | --- |
| `src/main/java/frc/robot/subsystems/AIControlBridge.java` | The API: topics, slots, pathfinding, shooting-zone approach. |
| `src/main/java/frc/robot/RobotContainer.java` | Registers the actions, defines the shooting commands and the zone. |
| `src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java` | Simulated AprilTag cameras with live streams. |
| `src/main/java/frc/robot/subsystems/vision/GamePieceVisionSim.java` | Simulated fuel detection camera. |
| `src/main/deploy/pathplanner/navgrid.json` | Pathfinding obstacle map, regenerated for the 2026 field. |
| `scripts/ai_demo/` | Demo recorder (a worked NT4 client) and renderer. |
| `docs/ai-control-demo.mp4` | 44 s recording of the API driving the robot. |
