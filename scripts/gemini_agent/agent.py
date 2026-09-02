#!/usr/bin/env python3
"""Drive the robot with natural language, through Gemini Robotics-ER 2.

    export GEMINI_API_KEY=...            # free key from https://aistudio.google.com/apikey
    ./gradlew simulateJava &             # or point --host at a real robot
    python3 scripts/gemini_agent/agent.py

Then type instructions: "run the left trench, pick up fuel, and score it".

The model is given five tools (drive_to, run_action, get_robot_state, look,
wait); each one reads or writes the robot's /AIControl NetworkTables topics and
hands the robot's live state straight back to the model.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
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
    ):
        self.client = client
        self.robot = robot
        self.model = model
        self.max_steps = max_steps
        self.verbose = verbose
        self.show_field = show_field
        self.tools = tool_declarations(robot.available_actions())
        self.system_instruction = build_system_prompt(robot)
        self.executor = RobotTools(robot, on_call=self._log_call)
        self.previous_id: str | None = None
        self.transcript: list[dict] = []
        self._publish("Model", model)
        self._publish("Instruction", "")
        self._publish("Thought", "")
        self._publish("ToolCall", "")
        self._publish("Answer", "")
        self._publish("Busy", "false")
        self._publish("Transcript", "[]")

    def _log_call(self, name: str, args: dict) -> None:
        pretty = ", ".join(f"{k}={v!r}" for k, v in args.items())
        print(f"  {GREEN}-> {name}({pretty}){RESET}", flush=True)
        self._publish("ToolCall", f"{name}({pretty})")
        self._record("tool_call", f"{name}({pretty})")

    def _publish(self, key: str, value: str) -> None:
        """Mirrors the agent's own state onto NetworkTables, for the dashboard."""
        try:
            self.robot.publish_agent_string(key, value)
        except Exception:
            pass  # a dashboard going dark must never cost you the robot

    def _record(self, kind: str, text: str) -> None:
        self.transcript.append({"t": time.time(), "kind": kind, "text": text})
        self._publish("Transcript", json.dumps(self.transcript[-40:]))

    def ask(self, instruction: str) -> str:
        """Runs one instruction to completion, tool call by tool call."""
        self._publish("Instruction", instruction)
        self._publish("Answer", "")
        self._publish("Busy", "true")
        self._record("instruction", instruction)
        try:
            return self._ask(instruction)
        finally:
            self._publish("Busy", "false")
            self._publish("ToolCall", "")

    def _ask(self, instruction: str) -> str:
        pending_input: object = self._opening_input(instruction)
        for step_index in range(self.max_steps):
            interaction = self._create(pending_input)
            self.previous_id = getattr(interaction, "id", None)

            calls = [s for s in (interaction.steps or []) if getattr(s, "type", None) == "function_call"]
            for step in interaction.steps or []:
                if getattr(step, "type", None) != "thought":
                    continue
                thought = _step_text(step)
                if not thought:
                    continue
                self._publish("Thought", thought)
                self._record("thought", thought)
                if self.verbose:
                    print(f"  {DIM}(thinking) {thought}{RESET}", flush=True)
            if not calls:
                answer = interaction.output_text or "(no reply)"
                self._publish("Answer", answer)
                self._record("answer", answer)
                return answer

            pending_input = []
            for call in calls:
                result, image = self.executor.call(call.name, dict(call.arguments or {}))
                summary = _summarise(result)
                print(f"  {DIM}   {summary}{RESET}", flush=True)
                self._record("tool_result", _strip_colour(summary))
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

    def _create(self, model_input):
        request = {
            "model": self.model,
            "input": model_input,
            "tools": self.tools,
            "system_instruction": self.system_instruction,
            "generation_config": {"thinking_level": "high"},
        }
        if self.previous_id:
            request["previous_interaction_id"] = self.previous_id
        return self.client.interactions.create(**request)


def _strip_colour(text: str) -> str:
    return re.sub(r"\033\[[0-9;]*m", "", text)


def _step_text(step) -> str:
    """First line of a step's text, whether it carries it directly or in content blocks."""
    for attr in ("text", "summary", "output_text"):
        value = getattr(step, attr, None)
        if isinstance(value, str) and value.strip():
            return value.strip().splitlines()[0][:160]
    for block in getattr(step, "content", None) or []:
        text = getattr(block, "text", None) or (
            block.get("text") if isinstance(block, dict) else None
        )
        if isinstance(text, str) and text.strip():
            return text.strip().splitlines()[0][:160]
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
