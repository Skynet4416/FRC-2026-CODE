#!/usr/bin/env python3
"""Drive the robot with natural language, through Gemini Robotics-ER 2.

    export GEMINI_API_KEY=...            # free key from https://aistudio.google.com/apikey
    ./gradlew simulateJava &             # or point --host at a real robot
    python3 scripts/gemini_agent/agent.py

Then type instructions: "run the left trench, pick up fuel, and score it".

The model is given five tools (drive_to, run_action, get_robot_state, look,
wait); each one reads or writes the robot's /AIControl NetworkTables topics and
hands the robot's live state straight back to the model.

Session._run_step wraps that call with a ticking "thinking... (Ns)" placeholder
so the operator's cockpit shows something the moment a request goes out, rather
than nothing until the whole step returns. This is NOT token-level streaming:
the Interactions API's own docs (https://ai.google.dev/gemini-api/docs/streaming)
do document a stream=True mode whose step.delta events carry a ThoughtSummaryDelta
with real incremental thought text - not just a ThoughtSignatureDelta's opaque
continuation token, which would not have been enough to justify this. But wiring
it up here made every step a *second*, discarded model call against this preview
endpoint whenever the server accepted stream=True but replied with a single
non-streaming body instead of real SSE - silently doubling latency and cost, and
corrupting the call_id a tool result is matched back to on that discarded call.
Given a preview model with unverified real-world stream support, that risk is
not worth it for a decoration. See docs/gemini-agent.md for the full writeup.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nt4 import RobotConnection  # noqa: E402
from prompt import build_system_prompt  # noqa: E402
import field_view  # noqa: E402
from robot_tools import RobotTools, encode_image, tool_declarations  # noqa: E402

DEFAULT_MODEL = "gemini-robotics-er-2-preview"
DEFAULT_MAX_STEPS = 12

DIM, BOLD, GREEN, RED, RESET = "\033[2m", "\033[1m", "\033[32m", "\033[31m", "\033[0m"


def make_client(api_key: str):
    from google import genai

    base_url = os.environ.get("GEMINI_BASE_URL")
    if base_url:
        return genai.Client(api_key=api_key, http_options={"base_url": base_url})
    return genai.Client(api_key=api_key)


class Session:
    """One conversation with the model, driving one robot."""

    def __init__(
        self,
        client,
        robot: RobotConnection,
        model: str,
        max_steps: int,
        verbose: bool,
        show_field: bool = True,
        on_event=None,
    ):
        self.client = client
        self.robot = robot
        self.model = model
        self.max_steps = max_steps
        self.verbose = verbose
        self.show_field = show_field
        # Called with (kind, text, event_id=None, live=False) for every thought,
        # tool call, result and answer. event_id/live let the same transcript
        # line update in place - a ticking "thinking..." placeholder, then the
        # real thought that replaces it - instead of piling up a new line every
        # second, see _run_step. The dashboard listens on this; nothing else needs it.
        self.on_event = on_event or (lambda kind, text, **_kwargs: None)
        self.tools = tool_declarations(robot.available_actions())
        self.system_instruction = build_system_prompt(robot)
        self.executor = RobotTools(robot, on_call=self._log_call)
        self.previous_id: str | None = None
        # Ticks up once per model turn, so every "thinking..." placeholder and
        # every thought bubble gets a transcript id nothing else could collide with.
        self._step_seq = 0

    def _log_call(self, name: str, args: dict) -> None:
        pretty = ", ".join(f"{k}={v!r}" for k, v in args.items())
        print(f"  {GREEN}-> {name}({pretty}){RESET}", flush=True)
        self.on_event("tool_call", f"{name}({pretty})")

    def ask(self, instruction: str) -> str:
        """Runs one instruction to completion, tool call by tool call."""
        self.on_event("instruction", instruction)
        pending_input: object = self._opening_input(instruction)
        for step_index in range(self.max_steps):
            interaction = self._run_step(pending_input)
            self.previous_id = getattr(interaction, "id", None)

            calls = [s for s in (interaction.steps or []) if getattr(s, "type", None) == "function_call"]
            if not calls:
                answer = interaction.output_text or "(no reply)"
                self.on_event("answer", answer)
                return answer

            pending_input = []
            for call in calls:
                result, image = self.executor.call(call.name, dict(call.arguments or {}))
                summary = _summarise(result)
                print(f"  {DIM}   {summary}{RESET}", flush=True)
                self.on_event("tool_result", _strip_colour(summary))
                content = [{"type": "text", "text": json.dumps(result)}]
                if image:
                    content.append(encode_image(image))
                pending_input.append(
                    {
                        "type": "function_result",
                        "name": call.name,
                        "call_id": call.id,
                        "result": content,
                        "is_error": "error" in result,
                    }
                )
        return (
            f"(stopped after {self.max_steps} tool calls - the robot is at "
            f"{self.robot.state()['status']!r}; raise --max-steps or give a smaller instruction)"
        )

    def _opening_input(self, instruction: str):
        """The instruction, with a look at the field attached so the model starts oriented."""
        if not self.show_field:
            return instruction
        try:
            image = field_view.render(self.robot, self.robot.landmarks())
        except Exception as exc:  # a missing renderer must not cost you the robot
            print(f"  {DIM}(no field view: {exc}){RESET}", flush=True)
            return instruction
        return [
            {"type": "text", "text": instruction},
            {"type": "text", "text": "The field as it looks right now:"},
            encode_image(image),
        ]

    def _request(self, model_input) -> dict:
        request = {
            "model": self.model,
            "input": model_input,
            "tools": self.tools,
            "system_instruction": self.system_instruction,
            "generation_config": {"thinking_level": "high"},
        }
        if self.previous_id:
            request["previous_interaction_id"] = self.previous_id
        return request

    def _create(self, model_input):
        return self.client.interactions.create(**self._request(model_input))

    def _run_step(self, model_input):
        """Runs one model turn and returns the finished interaction.

        No true token streaming here - see the module docstring for why - so
        the whole thought arrives at once, whenever the one blocking _create()
        call returns. What we *can* give the operator honestly in the meantime
        is a "thinking... (Ns)" placeholder that starts ticking the moment the
        request goes out, then gets replaced by the real thought(s) in one shot
        the instant the response is parsed - never a guess at content, just an
        honest clock while there is nothing else to show.
        """
        step_id = f"think-{self._next_id()}"
        started = time.time()
        stop_ticking = threading.Event()
        self.on_event("thought", "thinking…", event_id=step_id, live=True)
        ticker = threading.Thread(
            target=self._tick_thinking, args=(step_id, started, stop_ticking), daemon=True
        )
        ticker.start()
        try:
            interaction = self._create(model_input)
        finally:
            stop_ticking.set()
        self._emit_thoughts(interaction, step_id)
        return interaction

    def _tick_thinking(self, step_id: str, started: float, stop: threading.Event) -> None:
        """Ticks a "thinking... (Ns)" placeholder once a second. The elapsed clock
        is the only honest thing to show during the gap before the blocking call
        this brackets returns with anything real."""
        while not stop.wait(1.0):
            self.on_event(
                "thought", f"thinking… ({time.time() - started:.0f}s)",
                event_id=step_id, live=True,
            )

    def _emit_thoughts(self, interaction, placeholder_id: str) -> None:
        """Replaces the ticking placeholder with whatever the model actually
        thought, in one shot - the whole thought lands at once from a plain
        (non-streaming) call, so there is nothing to trickle in."""
        used = False
        for step in interaction.steps or []:
            if getattr(step, "type", None) != "thought":
                continue
            thought = _step_text(step, full=True)
            if not thought:
                continue
            event_id = placeholder_id if not used else f"think-{self._next_id()}"
            used = True
            self.on_event("thought", thought, event_id=event_id, live=False)
            if self.verbose:
                print(f"  {DIM}(thinking) {_step_text(step)}{RESET}", flush=True)
        if not used:
            self.on_event("thought", "", event_id=placeholder_id, live=False)

    def _next_id(self) -> int:
        self._step_seq += 1
        return self._step_seq


def _strip_colour(text: str) -> str:
    return re.sub(r"\033\[[0-9;]*m", "", text)


def _step_text(step, full: bool = False) -> str:
    """First line of a step's text, whether it carries it directly or in content
    blocks - or the whole thing when full=True, which is what the cockpit's
    transcript shows (the terminal's one-line (thinking) print stays truncated)."""
    for attr in ("text", "summary", "output_text"):
        value = getattr(step, attr, None)
        if isinstance(value, str) and value.strip():
            return value.strip() if full else value.strip().splitlines()[0][:160]
    for block in getattr(step, "content", None) or []:
        text = getattr(block, "text", None) or (
            block.get("text") if isinstance(block, dict) else None
        )
        if isinstance(text, str) and text.strip():
            return text.strip() if full else text.strip().splitlines()[0][:160]
    return ""


def _summarise(result: dict) -> str:
    if "error" in result:
        return f"{RED}{result['error']}{RESET}"
    pose = result.get("pose") or {}
    bits = []
    if pose:
        bits.append(f"pose ({pose['x']:.2f}, {pose['y']:.2f}, {pose['heading_deg']:.0f} deg)")
    if result.get("status"):
        bits.append(str(result["status"]))
    if result.get("camera"):
        bits.append(f"image from {result['camera']}")
    if result.get("view"):
        seen = len(result.get("fuel_seen_by_camera") or [])
        bits.append(f"field view ({seen} fuel in sight)")
    return " | ".join(bits) or json.dumps(result)[:120]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", "127.0.0.1"),
                        help="robot / simulator address (default 127.0.0.1; a real robot is 10.44.16.2)")
    parser.add_argument("--port", type=int, default=5810, help="NT4 port (default 5810)")
    parser.add_argument("--model", default=os.environ.get("GEMINI_MODEL", DEFAULT_MODEL))
    parser.add_argument("--task", action="append", default=[],
                        help="run this instruction and exit; repeatable")
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS,
                        help="tool calls per instruction before giving up")
    parser.add_argument("--verbose", action="store_true", help="print the model's thinking")
    parser.add_argument("--no-field-view", action="store_true",
                        help="do not attach the top-down field view to each instruction")
    parser.add_argument("--print-prompt", action="store_true",
                        help="print the system prompt and tool schema, then exit (no API key needed)")
    args = parser.parse_args()

    print(f"{DIM}connecting to the robot at {args.host}:{args.port} ...{RESET}", flush=True)
    try:
        robot = RobotConnection(args.host, args.port).connect()
    except Exception as exc:
        print(f"{RED}{exc}{RESET}\nIs the robot code running? Try ./gradlew simulateJava", file=sys.stderr)
        return 2

    print(f"{DIM}connected. actions: {', '.join(robot.available_actions())}{RESET}")

    if args.print_prompt:
        print("\n" + build_system_prompt(robot) + "\n")
        print(json.dumps(tool_declarations(robot.available_actions()), indent=2))
        robot.close()
        return 0

    api_key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    if not api_key:
        print(f"{RED}GEMINI_API_KEY is not set.{RESET} Get a free key at "
              "https://aistudio.google.com/apikey, then: export GEMINI_API_KEY=...", file=sys.stderr)
        robot.close()
        return 2

    session = Session(make_client(api_key), robot, args.model, args.max_steps, args.verbose,
                      show_field=not args.no_field_view)
    print(f"{DIM}model: {args.model}{RESET}")

    try:
        if args.task:
            for task in args.task:
                print(f"\n{BOLD}> {task}{RESET}", flush=True)
                print(session.ask(task), flush=True)
            return 0
        print(f"{DIM}Type an instruction, or Ctrl-D to quit.{RESET}")
        while True:
            try:
                instruction = input(f"\n{BOLD}> {RESET}").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not instruction:
                continue
            if instruction in {"quit", "exit"}:
                break
            started = time.time()
            print(session.ask(instruction), flush=True)
            print(f"{DIM}({time.time() - started:.1f}s){RESET}")
        return 0
    finally:
        robot.set_action("STOP")
        time.sleep(0.2)
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
