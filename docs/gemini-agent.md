# Driving the robot with Gemini Robotics-ER 2

Type an instruction in English, watch the robot carry it out in AdvantageScope.

```
> run the left trench, pick up fuel, and score it
  -> look_at_field()
     field view (18 fuel in sight)
  -> drive_to(x=4.6, y=7.4, heading_deg=0, max_speed=3.5)
     pose (4.58, 7.42, 0 deg) | AT TARGET
  -> run_action(action='INTAKE')
     pose (4.60, 7.43, 0 deg) | RUNNING INTAKE
  -> run_action(action='SHOOT_FUEL')
     pose (2.05, 4.06, -2 deg) | MOVING INTO THE ALLIANCE ZONE, then SHOOT_FUEL
Ran the left trench lane, intook, and shot from the alliance zone at (2.05, 4.06).
```

The model is Google's [Gemini Robotics-ER 2](https://ai.google.dev/gemini-api/docs/robotics-overview),
the embodied-reasoning model meant to be a robot's high-level brain: it plans and calls tools,
while the robot's own controllers do the driving, aiming and shooting. It talks to
[`/AIControl`](ai-control.md), the NetworkTables API this repo already exposes, so nothing
about the robot code is Gemini-specific - the same bridge works from a script, from
AdvantageScope, or from any other model.

- [What you need](#what-you-need)
- [GitHub secrets](#github-secrets)
- [Running it](#running-it)
- [What the model sees](#what-the-model-sees)
- [The tools](#the-tools)
- [The prompt](#the-prompt)
- [Streaming - and why the cockpit does not use it](#streaming)
- [Testing it without a key](#testing-it-without-a-key)
- [Cost and rate limits](#cost-and-rate-limits)
- [Troubleshooting](#troubleshooting)
- [Files](#files)

## What you need

1. **An API key.** Go to [aistudio.google.com/apikey](https://aistudio.google.com/apikey), sign in
   with a Google account, and click *Create API key*. The free tier is enough to fly the robot
   around; no billing account is required to get started.
2. **Python 3.10+** and the agent's three dependencies.
3. **The robot**, real or simulated.

```bash
pip install -r scripts/gemini_agent/requirements.txt
```

## GitHub secrets

One secret, and only if you want the workflow to drive the simulator for you. The agent itself
reads the key from your shell, not from GitHub.

Go to **Settings → Secrets and variables → Actions → New repository secret** on
`Skynet4416/FRC-2026-CODE` and add:

| Name | Value | Where it is used |
| --- | --- | --- |
| `GEMINI_API_KEY` | the key from [aistudio.google.com/apikey](https://aistudio.google.com/apikey), pasted whole - it looks like `AIza...`, about 39 characters, no quotes and no `GEMINI_API_KEY=` prefix | `.github/workflows/ai-control-sim.yml`, the *Drive the simulator with Gemini* job |

That is the whole list. Nothing else needs a secret: NetworkTables is local to the runner, and the
model name is a workflow input rather than a secret so you can change it per run.

Once the secret exists, go to **Actions → AI control → Run workflow**, type an instruction, and the
run boots the simulator, hands the instruction to Gemini, and uploads the simulator and agent logs
as an artifact. Until the secret exists that job stops immediately with a message saying so, and the
key-free self-test job keeps running on every push.

To use the key locally instead:

```bash
export GEMINI_API_KEY=AIza...      # the same key; keep it out of git
```

## Running it

```bash
./gradlew simulateJava                    # terminal 1: the robot
python3 scripts/gemini_agent/agent.py     # terminal 2: you and the model
```

Open AdvantageScope on `localhost` to watch. Then type instructions:

```
> line up in the left trench lane
> pick up fuel and shoot it
> back off two metres, then charge the bump at full speed
> what can you see right now?
> stop
```

Useful flags:

| Flag | What it does |
| --- | --- |
| `--task "..."` | Run one instruction and exit. Repeatable, and what CI uses. |
| `--host 10.44.16.2` | Talk to the real robot instead of the simulator. |
| `--model ...` | Another model. Defaults to `gemini-robotics-er-2-preview`. |
| `--verbose` | Print the model's thinking as well as its tool calls. |
| `--print-prompt` | Print the system prompt and tool schema and exit. No API key needed. |
| `--no-field-view` | Stop attaching the field map to each instruction (text only). |
| `--max-steps N` | Tool calls per instruction before the agent gives up. Default 12. |

On the real robot the bridge never enables the robot - the driver station does. In simulation the
bridge enables it while it has work to do, which is why the agent can drive a fresh simulator with
nothing else running.

## What the model sees

Every instruction arrives with a **top-down view of the field**, in the same coordinates the tools
take, and `look_at_field` redraws it on demand:

- the official 2026 field render underneath - the same background as the demo video, so the
  hubs, trenches and alliance zones are the real ones rather than a sketch. It is the empty
  field, with no game pieces drawn on it, so every ball in the picture is simulated fuel;
- a one-metre grid with labelled axes, so a position can be read straight off the picture;
- the obstacles out of the robot's own `navgrid.json`, so the map and the pathfinder agree about
  what is solid;
- the robot as a box with a heading arrow - red instead of blue while the shooter owns the
  drivetrain;
- the fuel the game-piece camera can see, as orange dots;
- the active PathPlanner trajectory and the pose that was requested;
- the named landmarks, so "the hub" has somewhere to point.

That map, rather than the camera, is the agent's main sense: spatial questions ("which lane is
clear?", "am I past the hub?") are answered by looking at the field from above, and a camera frame
of a dark carpet is not much help with any of them. The camera is still there as
`look_through_camera` for the questions the map cannot answer.

Alongside the picture, every tool call returns the robot's live state - pose, status, whether it is
navigating, whether the shooter has taken the drivetrain, whether it is in the shooting zone, and
the last error. So the model is never guessing about what the robot just did.

## The tools

| Tool | What it does |
| --- | --- |
| `drive_to(x, y, heading_deg, max_speed?, max_accel?, wait?)` | PathPlanner pathfinding to a pose. Waits for arrival by default. |
| `run_action(action, wait?)` | Runs a mechanism action - the enum comes from the robot's own `AvailableActions`, so a newly registered action is offered without touching the agent. Actions on different mechanisms run at the same time, so `wait=false` then `drive_to` is how the model intakes while driving. |
| `look_at_field()` | Redraws the top-down view. |
| `look_through_camera(camera?)` | One frame from a PhotonVision MJPEG stream. Defaults to the game-piece camera's processed stream. |
| `get_robot_state()` | Pose and status, without doing anything. |
| `wait(seconds)` | Let time pass, then read the state again. |

Every one of them is a thin wrapper over the `/AIControl` topics: `drive_to` writes `TargetPose`,
`run_action` writes `ActionTrigger`, and nothing else in the agent knows what a shooter is. An
action the robot does not have is refused locally and reported back to the model as an error,
rather than written and ignored.

## The prompt

The system prompt is assembled at connect time from what the robot is publishing, so it cannot
drift away from the robot code:

- the rules of the API come from `/AIControl/Notes`, written by `AIControlBridge`;
- the action list from `/AIControl/AvailableActions`;
- the field size from `/AIControl/FieldSize` and the named positions from `/AIControl/Landmarks`,
  both derived from `FieldConstants` in `RobotContainer`;
- the camera list from `/CameraPublisher`.

Only the part about *how to behave* - one tool call at a time, read the result, use the map, do not
repeat a call that failed, say when something is impossible - lives in `scripts/gemini_agent/prompt.py`.

Print the whole thing, against a running robot, with:

```bash
python3 scripts/gemini_agent/agent.py --print-prompt
```

## Streaming - and why the cockpit does not use it
<a id="streaming"></a>

The operator's cockpit ([cockpit.md](cockpit.md)) shows the model's thinking as it plays, which
raises the obvious question: does the Interactions API actually stream a thought as it is
generated, word by word, or only hand it over once the step is done?

**It can stream real thought text, not just a token.** `client.interactions.create(...,
stream=True)` returns an iterator of server-sent events - documented at
[ai.google.dev/gemini-api/docs/streaming](https://ai.google.dev/gemini-api/docs/streaming) -
and one of the deltas it emits mid-step is a **`step.delta` event whose `delta.type` is
`"thought_summary"`**, carrying a real, readable chunk of the thought summary at
`delta.content.text`:

```json
{"event_type": "step.delta", "index": 0,
 "delta": {"type": "thought_summary", "content": {"type": "text", "text": "Thinking..."}}}
```

That is a genuinely different thing from a **`thought_signature`** delta, which the same API
also emits - an opaque continuation token the SDK needs for its own bookkeeping, with no
readable text in it at all. A design that only had access to `thought_signature` deltas would
have nothing honest to show the operator; `thought_summary` is the real thing, confirmed
against the installed `google-genai` SDK (2.22.0, the version this repo pins) by standing up a
throwaway local SSE server and running `client.interactions.create(stream=True, ...)` against
it - `ThoughtSummaryDelta.content.text` came back exactly as the docs describe, chunk by chunk.

**The cockpit still does not use it**, for a reason specific to this preview model rather than
to the API in general: `stream=True` has to be tried the same way every other request is, which
means the first time it is *not* actually honoured - an older SDK, or this specific preview
endpoint accepting the flag but replying with one plain JSON body instead of real SSE framing -
the client has no way to tell before it has already spent a full request finding out. Falling
back at that point means a *second*, separate call for the same step: double latency and quota
on every single turn if that is what this endpoint does, and - the concrete failure this repo's
self-test caught while trying it - a real risk of the fallback's response landing under a
different interaction than the one already shown to the operator, breaking the `call_id` a tool
result is matched back to. Nothing here has been able to confirm which way
`gemini-robotics-er-2-preview` actually behaves without spending real quota against a live key
to find out, so `Session.ask()` sends one plain (non-streaming) request per step, exactly as it
always has.

What the cockpit does instead - and it is worth being precise about what this is *not* - is a
"thinking... (Ns)" placeholder that goes up the moment the request is sent, ticks off elapsed
seconds while the blocking call is in flight, and is replaced by the real thought in one shot
the instant the response comes back (`Session._run_step` in `agent.py`). That is an honest
progress indicator, not token-level streaming: nothing about the thought's *content* is shown
until the whole thing has arrived. If a later SDK release or a non-preview model confirms
`stream=True` behaves as documented against this endpoint, wiring up `thought_summary` deltas
the way described above is the way to turn that placeholder into the real thing.

## Testing it without a key

```bash
python3 scripts/gemini_agent/selftest.py
```

This stands up a fake NT4 server that behaves like the bridge, a fake camera, and a fake
Interactions API that replies with a scripted sequence of tool calls, then runs the real agent
against them and checks the whole path: that the model's `drive_to` arrives at the robot as a
`TargetPose`, that results go back as `function_result` blocks matched by `call_id`, that the field
view is attached as an image, that the conversation is continued with `previous_interaction_id`, and
that an invented action is refused instead of written. It needs no API key, no robot, and no
network, and it runs on every push through the *Agent self-test* job.

## Cost and rate limits

Gemini Robotics-ER 2 is available on the Gemini API free tier through AI Studio, with paid pricing
of $2.00 per million input tokens and $10.00 per million output tokens beyond it. One instruction is
a handful of interactions, each carrying the system prompt, the field view PNG and the tool results,
so a session of playing with the robot is cheap but not free of quota. If you hit a rate limit the
API returns 429 and the agent prints it - wait, or use `--no-field-view` to cut the images out.

## Troubleshooting

**`GEMINI_API_KEY is not set`.** Export it in the shell you run the agent from. In CI it comes from
the repository secret of the same name.

**`no NetworkTables server at ws://...`.** Robot code is not running, or `--host` is wrong. The
simulator is `127.0.0.1`; the robot is `10.44.16.2`.

**`connected, but /AIControl/ never appeared`.** The robot code that is running does not build the
`AIControlBridge` - an older branch, or a deploy that predates it.

**The model plans nicely and the robot does not move.** Look at `Status` in AdvantageScope. The
usual causes are in [ai-control.md](ai-control.md#troubleshooting): an unreachable pose, or a bump
taken too slowly.

**`no camera <name>`.** The tool result lists the cameras that are actually publishing. In
simulation the streams are advertised under the simulator's own hostname, which the agent rewrites
to whatever host it connected to.

## Files

| File | Role |
| --- | --- |
| `scripts/gemini_agent/agent.py` | The CLI and the model loop: instruction in, tool calls out, answer back. |
| `scripts/gemini_agent/robot_tools.py` | Tool schema and the code behind each tool. |
| `scripts/gemini_agent/field_view.py` | The top-down field view the model looks at. |
| `scripts/assets/field26.png` | The empty 2026 field render, shared by the field view and the demo renderer. |
| `scripts/gemini_agent/nt4.py` | NT4 client for `/AIControl`, with no robotpy dependency. |
| `scripts/gemini_agent/camera.py` | Pulls one JPEG out of an MJPEG stream. |
| `scripts/gemini_agent/prompt.py` | The behaviour half of the system prompt. |
| `scripts/gemini_agent/selftest.py` | End-to-end test with a fake robot and a fake Gemini. |
| `.github/workflows/ai-control-sim.yml` | Self-test on every push; drive the simulator on demand. |
