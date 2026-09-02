# The cockpit - driving the robot from a browser

A page that shows the robot on the field and lets you type instructions to it. The model's
thoughts, tool calls and answers appear beside the field as they happen.

It is one Python process serving one port, standard library only, so it works the same on
your laptop and in a Codespace.

- [Run it locally](#run-it-locally)
- [Run it in a Codespace](#run-it-in-a-codespace)
- [Who can drive](#who-can-drive)
- [What the page shows](#what-the-page-shows)
- [What it is not](#what-it-is-not)

## Run it locally

```bash
pip install -r scripts/gemini_agent/requirements.txt
export GEMINI_API_KEY=...              # free key: https://aistudio.google.com/apikey

./gradlew simulateJava &               # the robot
python3 scripts/gemini_agent/serve.py  # the cockpit, on http://localhost:8000
```

Open <http://localhost:8000>, type "run the left trench, pick up fuel, and score it", and
watch. The same flags as the CLI apply: `--host` for a real robot, `--model`, `--max-steps`,
`--web-port`.

## Run it in a Codespace

The repo carries a devcontainer, so a Codespace comes up with Java 21, Python 3.11 and the
agent's dependencies already installed.

1. **Code → Codespaces → Create codespace** on this branch.
2. Add your key once, in the Codespace terminal:
   ```bash
   echo 'export GEMINI_API_KEY=...' >> ~/.bashrc && source ~/.bashrc
   ```
   Or set it as a [Codespaces secret](https://github.com/settings/codespaces) named
   `GEMINI_API_KEY` so every future Codespace has it.
3. Run the two commands:
   ```bash
   ./gradlew simulateJava &
   python3 scripts/gemini_agent/serve.py
   ```
4. The **Ports** tab shows *AI cockpit* on 8000 - open it.

**Forwarded ports are private by default**, so the URL only works for someone signed in to
GitHub as you. To let the team watch, set port 8000's visibility to **Organization** in the
Ports tab: anyone in Skynet4416 can then open the link, and nobody else can. Making it
*Public* removes that check - don't, unless you have also set `AGENT_WEB_TOKEN`.

Codespaces bills by the minute against your monthly free allowance (120 core-hours on a free
account, so about 30 hours on the 4-core machine this devcontainer asks for), and idles out
after 30 minutes. Stop it from the Codespaces page when the demo is over.

## Who can drive

Watching is open to anyone who can reach the page. Typing an instruction is what spends
Gemini quota, so that is the thing to gate:

```bash
AGENT_WEB_TOKEN=some-shared-secret python3 scripts/gemini_agent/serve.py
```

The page then asks for the token before it will send an instruction or a stop. Combined with
organization-visible port forwarding, that is two independent locks; on a public port it is
the only one.

Nothing about the API key ever reaches the browser: prompts go to the cockpit, and the
cockpit talks to Google.

## What the page shows

- **The field**, drawn on PathPlanner's 2026 image, with the navgrid obstacles the
  pathfinder refuses to cross, the named landmarks, and a one-metre grid.
- **The robot**, as a bumper square with a red nose for its heading, ringed in red while a
  shooting action owns the drivetrain.
- **Fuel**, in two layers: every ball the physics sim has (faint), and the ones the
  game-piece camera can actually see (bright) - which is all the model is told about.
- **The active PathPlanner trajectory** and the pose that was requested.
- **The transcript**: the instruction, the model's thinking, each tool call and its result,
  and the final answer.
- **The status pill**: what the robot is doing, or why it cannot - a missing robot, a missing
  key, or an error from the model.

The page polls `/state` ten times a second. There is no WebSocket, which is what makes it
survive Codespaces' port forwarding, corporate proxies and everything else without fuss.

## What it is not

It is not a public website. The robot has to be running somewhere - your machine or a
Codespace - and the page is a window onto that. Anyone without a simulator running sees a
page that says the robot is not connected. For a link that shows a real run to anyone,
trigger the *AI control* workflow: it drives the robot with Gemini and uploads the logs.
