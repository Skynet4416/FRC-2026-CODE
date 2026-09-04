#!/usr/bin/env python3
"""End-to-end test of the agent loop with no API key and no robot.

Stands up a fake NT4 server that behaves like the /AIControl bridge, a fake
MJPEG camera, and a fake Interactions API that replies with a scripted sequence
of function calls, then runs the real Session against them and checks that the
robot received the writes the model asked for.

    python3 scripts/gemini_agent/selftest.py

Everything between the model and the robot is the production code path: the
same NT4 client, the same tool schema, the same function_result round trip.
"""

from __future__ import annotations

import asyncio
import base64
import http.server
import json
import os
import socket
import sys
import threading
import time

import msgpack
import websockets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nt4 import RobotConnection  # noqa: E402
from robot_tools import RobotTools, tool_declarations  # noqa: E402

# A 1x1 black JPEG, so `look` has something real to decode and forward.
# How long an action "runs" on the fake robot before it retires itself.
ACTION_SECONDS = 2.0

# Empty arrays carry no element type, so the string ones are named rather than guessed.
STRING_ARRAY_TOPICS = {"/AIControl/AvailableActions", "/AIControl/RunningActions"}

JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0a"
    "HBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAALCAABAAEBAREA/8QAFAABAAAAAAAA"
    "AAAAAAAAAAAACf/EABQQAQAAAAAAAAAAAAAAAAAAAAD/2gAIAQEAAD8AKp//2Q=="
)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# --------------------------------------------------------------------------
# Fake robot: an NT4 server that answers like the AIControlBridge
# --------------------------------------------------------------------------


class FakeRobot:
    """Speaks enough NT4 to stand in for robot code running the bridge."""

    def __init__(self, port: int, camera_url: str):
        self.port = port
        self.writes: list[tuple[str, object]] = []
        self.values = {
            "/AIControl/RobotPose": [1.5, 4.0, 0.0],
            "/AIControl/ActiveTarget": [],
            "/AIControl/Navigating": False,
            "/AIControl/ActionRunning": False,
            "/AIControl/RotationLocked": False,
            "/AIControl/InShootingZone": True,
            "/AIControl/AtTarget": False,
            "/AIControl/Status": "IDLE",
            "/AIControl/LastAction": "",
            "/AIControl/LastError": "",
            "/AIControl/MaxSpeed": 2.5,
            "/AIControl/MaxAccel": 2.5,
            "/AIControl/AvailableActions": [
                "STOP", "INTAKE", "STOW_INTAKE", "COLLECT_FUEL", "GRAB_FUEL",
                "AUTO_FOLD_ON", "AUTO_FOLD_OFF", "SHOOT_FUEL", "ALIGN_HUB",
            ],
            "/AIControl/RunningActions": [],
            "/AIControl/ResetSimulation": False,
            "/AIControl/HeadingUnits": "degrees",
            "/AIControl/Notes": "Test notes from the robot.",
            "/AIControl/Landmarks": '{"our_hub":[9.000,4.000,0.0]}',
            "/AIControl/FieldSize": [17.55, 8.05],
            "/CameraPublisher/fuel-cam-processed/streams": [f"mjpg:{camera_url}"],
            # v2 contract: ground-truth fuel, zones, the game brief, and the
            # collect target the model can write.
            "/AIControl/Fuel": json.dumps({
                "available": True,
                "source": "simulation ground truth",
                "vision_source": "sim-ground-truth",
                "on_board": 3,
                "capacity": 150,
                "intake_down": True,
                "intake_collecting": True,
                "collect_target": 10,
                "total_on_field": 5,
                "zones": {
                    "neutral_zone": {"count": 5, "centre": [8.0, 4.0],
                                      "nearest": [7.5, 4.2], "nearest_distance_m": 2.1},
                },
                "nearest": [{"x": 7.5, "y": 4.2, "distance_m": 2.1}],
                "clusters": [{"x": 8.0, "y": 4.0, "count": 5, "radius_m": 1.0}],
                "seen_by_camera": [
                    {"x": 7.5, "y": 4.2, "distance_m": 2.1, "bearing_deg": 3.0}
                ],
                "pickup": {
                    "x": 7.5, "y": 4.2, "heading_deg": 90.0, "max_speed": 1.2,
                    "sweep_to": [7.5, 5.2], "fuel": [7.5, 4.2],
                    "note": "drive here with INTAKE down, fuel passes on your LEFT",
                },
            }),
            "/AIControl/Zones": json.dumps({
                "our_alliance_zone": {"x_min": 0.0, "y_min": 0.0, "x_max": 3.0,
                                       "y_max": 8.05, "what": "score here"},
                "left_trench": {"x_min": 6.0, "y_min": 0.0, "x_max": 7.0,
                                 "y_max": 2.0, "what": "low crossing"},
            }),
            "/AIControl/GameBrief": "REBUILT: collect fuel, score in the hub.",
            "/AIControl/FuelPositions": [7.5, 4.2, 8.0, 4.0],
            "/AIControl/FuelOnBoard": 3.0,
            "/AIControl/CollectTarget": 10.0,
            # v2.1 addendum: the trench/bump auto-fold.
            "/AIControl/IntakePolicy": json.dumps({
                "intake_down": True, "auto_fold": True, "folded_for": None,
                "in_hazard": False, "hazard": None, "wants_down": True,
                "reason": "clear of every hazard",
            }),
            "/AIControl/AutoFoldIntake": True,
            # HubState addendum: which alliance's hub is active this shift.
            "/AIControl/HubState": json.dumps({
                "active": True, "shift": "SHIFT2", "shift_remaining_s": 12.4,
                "match_time_s": 78.2, "hub_state_ignored": True,
            }),
        }
        self.types = {
            bool: 0, float: 1, str: 4, int: 1,
        }
        # name -> the moment it stops running. Actions on different mechanisms overlap
        # here exactly as they do on the robot, so the tools' waits are really exercised.
        self.action_ends: dict[str, float] = {}
        self.resets = 0
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ready = threading.Event()

    def start(self) -> "FakeRobot":
        threading.Thread(target=self._run, daemon=True).start()
        self._ready.wait(10)
        return self

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self) -> None:
        async with websockets.serve(
            self._handle, "127.0.0.1", self.port,
            subprotocols=["v4.1.networktables.first.wpi.edu"],
        ):
            self._ready.set()
            await asyncio.Future()

    async def _handle(self, ws) -> None:
        pubs: dict[int, str] = {}
        ids = {name: i for i, name in enumerate(self.values, start=100)}

        async def send_all():
            self._expire_actions()
            await ws.send(json.dumps([
                {"method": "announce",
                 "params": {"name": name, "id": ids[name], "type": _nt_type(name, value),
                            "pubuid": None, "properties": {}}}
                for name, value in self.values.items()
            ]))
            for name, value in self.values.items():
                await ws.send(msgpack.packb([ids[name], int(time.time() * 1e6),
                                             _nt_type_id(name, value), value]))

        async def heartbeat():
            # The real bridge publishes every loop, which is how a client sees an action
            # finish. Without it the tools' waits would only ever see the last write.
            try:
                while True:
                    await asyncio.sleep(0.05)
                    await send_all()
            except Exception:
                pass  # the connection went away

        async for message in ws:
            if isinstance(message, str):
                for item in json.loads(message):
                    params = item.get("params", {})
                    if item.get("method") == "publish":
                        pubs[params["pubuid"]] = params["name"]
                    elif item.get("method") == "subscribe":
                        asyncio.create_task(heartbeat())
                        await send_all()
                continue
            unpacker = msgpack.Unpacker(raw=False, use_list=True)
            unpacker.feed(message)
            for pubuid, _ts, _type, value in unpacker:
                name = pubs.get(pubuid, str(pubuid))
                self.writes.append((name, value))
                self._apply(name, value)
            try:
                await send_all()
            except Exception:
                return  # the client hung up between its write and our reply

    def _expire_actions(self) -> None:
        """Retires the actions whose time is up, the way the bridge's periodic does."""
        now = time.time()
        self.action_ends = {n: end for n, end in self.action_ends.items() if end > now}
        self.values["/AIControl/RunningActions"] = list(self.action_ends)
        self.values["/AIControl/ActionRunning"] = bool(self.action_ends)

    def _apply(self, name: str, value) -> None:
        """The bit of bridge behaviour the tools actually depend on."""
        if name == "/AIControl/TargetPose" and value:
            self.values["/AIControl/RobotPose"] = list(value)
            self.values["/AIControl/ActiveTarget"] = list(value)
            self.values["/AIControl/AtTarget"] = True
            self.values["/AIControl/Status"] = "AT TARGET"
        elif name == "/AIControl/ActionTrigger" and value:
            self.values["/AIControl/LastAction"] = value
            if value == "STOP":
                self.action_ends.clear()
                self.values["/AIControl/Status"] = "STOPPED"
            else:
                self.action_ends[value] = time.time() + ACTION_SECONDS
                self.values["/AIControl/Status"] = f"RUNNING {value}"
        elif name == "/AIControl/ResetSimulation" and value:
            self.resets += 1
            self.action_ends.clear()
            self.values["/AIControl/RobotPose"] = [1.5, 4.0, 0.0]
            self.values["/AIControl/Status"] = "RESET"
        elif name in ("/AIControl/MaxSpeed", "/AIControl/MaxAccel"):
            self.values[name] = value


def _nt_type(name: str, value) -> str:
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, str):
        return "string"
    if isinstance(value, (int, float)):
        return "double"
    if name in STRING_ARRAY_TOPICS or (value and isinstance(value[0], str)):
        return "string[]"
    return "double[]"


def _nt_type_id(name: str, value) -> int:
    return {"boolean": 0, "double": 1, "string": 4, "double[]": 17, "string[]": 20}[
        _nt_type(name, value)
    ]


# --------------------------------------------------------------------------
# Fake camera and fake Interactions API
# --------------------------------------------------------------------------


def start_camera(port: int) -> None:
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("content-type", "image/jpeg")
            self.send_header("content-length", str(len(JPEG)))
            self.end_headers()
            self.wfile.write(JPEG)

        def log_message(self, *_args):
            pass

    server = http.server.HTTPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()


class FakeGemini:
    """Replies with a scripted sequence, and records what it was sent."""

    def __init__(self, port: int, script: list[dict]):
        self.port = port
        self.script = script
        self.requests: list[dict] = []
        outer = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["content-length"])))
                outer.requests.append(body)
                step = outer.script[min(len(outer.requests) - 1, len(outer.script) - 1)]
                payload = {
                    "id": f"int_{len(outer.requests)}",
                    "status": "completed",
                    "steps": step["steps"],
                }
                data = json.dumps(payload).encode()
                self.send_response(200)
                self.send_header("content-type", "application/json")
                self.send_header("content-length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *_args):
                pass

        self.server = http.server.HTTPServer(("127.0.0.1", port), Handler)

    def start(self) -> "FakeGemini":
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        return self


def call(name: str, arguments: dict, call_id: str) -> dict:
    return {"type": "function_call", "name": name, "arguments": arguments, "id": call_id}


SCRIPT = [
    {"steps": [{"type": "thought", "content": [{"type": "text", "text": "Heading for the hub."}]},
               call("drive_to", {"x": 6.4, "y": 4.0, "heading_deg": 0, "max_speed": 4.0}, "c1")]},
    {"steps": [call("look_at_field", {}, "c2")]},
    {"steps": [call("look_through_camera", {}, "c2b")]},
    {"steps": [call("run_action", {"action": "INTAKE"}, "c3")]},
    {"steps": [call("run_action", {"action": "FLY"}, "c4")]},
    # Intake down, then drive - the two have to overlap, not queue up behind each other.
    {"steps": [call("run_action", {"action": "INTAKE", "wait": False}, "c5")]},
    {"steps": [call("drive_to", {"x": 3.0, "y": 2.0, "heading_deg": 90}, "c6")]},
    {"steps": [{"type": "model_output",
                "content": [{"type": "text",
                             "text": "Drove to (6.4, 4.0), looked, and intook. FLY is not a real "
                                     "action, so I skipped it."}]}]},
]


# --------------------------------------------------------------------------


def check_defensive_json_parsing(check) -> None:
    """fuel()/zones()/intake_policy()/hub_state() must degrade to {} on malformed
    JSON rather than raising - a corrupt topic must not cost the robot connection.
    Built with __new__ so this needs no live server: get() only touches _values
    and _lock, which is all a bare instance needs.
    """
    robot = RobotConnection.__new__(RobotConnection)
    robot._lock = threading.Lock()
    robot._values = {
        "/AIControl/Fuel": "{not json",
        "/AIControl/Zones": "[1, 2,",
        "/AIControl/IntakePolicy": "nope",
        "/AIControl/HubState": "{",
    }
    check("malformed Fuel JSON returns {} instead of raising",
          robot.fuel() == {}, str(robot.fuel()))
    check("malformed Zones JSON returns {} instead of raising",
          robot.zones() == {}, str(robot.zones()))
    check("malformed IntakePolicy JSON returns {} instead of raising",
          robot.intake_policy() == {}, str(robot.intake_policy()))
    check("malformed HubState JSON returns {} instead of raising",
          robot.hub_state() == {}, str(robot.hub_state()))


def _await_write(robot_sim: "FakeRobot", name: str, value, timeout: float = 5.0) -> None:
    """Polls robot_sim.writes for one (name, value) pair, the way the reset check
    below already does - a write lands on the fake robot a moment after the
    client sends it, not necessarily before the call that triggered it returns."""
    deadline = time.time() + timeout
    while time.time() < deadline and (name, value) not in robot_sim.writes:
        time.sleep(0.02)


def check_v2_contract(check, robot: RobotConnection, robot_sim: "FakeRobot") -> None:
    """The v2 contract: ground-truth fuel, zones, the game brief, the trench/bump
    auto-fold, and the Alliance Shift hub state - parsed defensively, surfaced in
    state(), and acted on by find_fuel/collect_fuel/grab_fuel/set_auto_fold."""
    check("fuel() parses the Fuel JSON",
          robot.fuel().get("on_board") == 3 and robot.fuel().get("total_on_field") == 5,
          str(robot.fuel()))
    check("zones() parses the Zones JSON",
          robot.zones().get("left_trench", {}).get("x_max") == 7.0, str(robot.zones()))
    check("game_brief() reads the brief string",
          robot.game_brief() == "REBUILT: collect fuel, score in the hub.",
          robot.game_brief())
    check("intake_policy() parses the IntakePolicy JSON",
          robot.intake_policy().get("auto_fold") is True, str(robot.intake_policy()))
    check("hub_state() parses the HubState JSON",
          robot.hub_state().get("shift") == "SHIFT2", str(robot.hub_state()))
    check("field_fuel() decodes the flat FuelPositions array",
          robot.field_fuel() == [(7.5, 4.2), (8.0, 4.0)], str(robot.field_fuel()))

    state = robot.state()
    check("state() carries the new fuel keys",
          state["fuel_on_board"] == 3 and state["intake_down"] is True
          and state["intake_collecting"] is True
          and state["nearest_fuel"] == {"x": 7.5, "y": 4.2, "distance_m": 2.1}
          and state["fuel_by_zone"] == {"neutral_zone": 5},
          str({k: state[k] for k in
               ("fuel_on_board", "intake_down", "intake_collecting",
                "nearest_fuel", "fuel_by_zone")}))
    check("state() carries the auto-fold keys",
          state["auto_fold"] is True and state["folded_for"] is None
          and state["hazard"] is None,
          str({k: state[k] for k in ("auto_fold", "folded_for", "hazard")}))
    check("state() carries the hub-shift keys",
          state["hub_active"] is True and state["shift"] == "SHIFT2"
          and state["shift_remaining_s"] == 12.4 and state["hub_state_ignored"] is True,
          str({k: state[k] for k in
               ("hub_active", "shift", "shift_remaining_s", "hub_state_ignored")}))

    tools = RobotTools(robot)

    fuel_result = tools.find_fuel()
    check("find_fuel reports availability and the ground-truth picture",
          fuel_result["available"] is True and fuel_result["total_on_field"] == 5
          and fuel_result["vision_source"] == "sim-ground-truth"
          and fuel_result["seen_by_camera"], str(fuel_result))

    view = tools.look_at_field()
    check("look_at_field carries the per-zone fuel counts and the vision label",
          view["fuel_by_zone"] == {"neutral_zone": 5}
          and view["vision_source"] == "sim-ground-truth" and view["seen_by_camera"],
          str({k: view[k] for k in ("fuel_by_zone", "vision_source", "seen_by_camera")}))

    before = len(robot_sim.writes)
    tools.collect_fuel(count=7, wait=False)
    _await_write(robot_sim, "/AIControl/ActionTrigger", "COLLECT_FUEL")
    since_collect = robot_sim.writes[before:]
    names = [n for n, _v in since_collect]
    check("collect_fuel wrote CollectTarget before triggering COLLECT_FUEL",
          "/AIControl/CollectTarget" in names and "/AIControl/ActionTrigger" in names
          and names.index("/AIControl/CollectTarget") < names.index("/AIControl/ActionTrigger"),
          str(since_collect))
    check("collect_fuel's count and trigger both reached the robot",
          ("/AIControl/CollectTarget", 7.0) in since_collect
          and ("/AIControl/ActionTrigger", "COLLECT_FUEL") in since_collect,
          str(since_collect))

    before = len(robot_sim.writes)
    tools.grab_fuel(count=2, wait=False)
    _await_write(robot_sim, "/AIControl/ActionTrigger", "GRAB_FUEL")
    since_grab = robot_sim.writes[before:]
    names = [n for n, _v in since_grab]
    check("grab_fuel wrote CollectTarget before triggering GRAB_FUEL",
          "/AIControl/CollectTarget" in names and "/AIControl/ActionTrigger" in names
          and names.index("/AIControl/CollectTarget") < names.index("/AIControl/ActionTrigger"),
          str(since_grab))
    check("grab_fuel's count and trigger both reached the robot",
          ("/AIControl/CollectTarget", 2.0) in since_grab
          and ("/AIControl/ActionTrigger", "GRAB_FUEL") in since_grab,
          str(since_grab))

    robot.set_auto_fold(False)
    _await_write(robot_sim, "/AIControl/AutoFoldIntake", False)
    check("set_auto_fold(False) reached the robot",
          ("/AIControl/AutoFoldIntake", False) in robot_sim.writes)
    robot.set_auto_fold(True)
    _await_write(robot_sim, "/AIControl/AutoFoldIntake", True)
    check("set_auto_fold(True) reached the robot",
          ("/AIControl/AutoFoldIntake", True) in robot_sim.writes)


def check_cockpit_restart(check, robot: RobotConnection) -> None:
    """The cockpit's restart button: the match goes back, the conversation is dropped."""
    from serve import Cockpit

    class FakeSession:
        model = "fake"

        def __init__(self, on_event):
            self.on_event = on_event

        def ask(self, instruction):
            self.on_event("instruction", instruction)
            self.on_event("answer", f"did {instruction}")

    cockpit = Cockpit(robot, FakeSession, None)
    cockpit.submit("drive somewhere")
    deadline = time.time() + 5
    while cockpit.busy and time.time() < deadline:
        time.sleep(0.02)
    first_session = cockpit.session
    check("the cockpit kept a transcript and a session", bool(cockpit.transcript) and first_session)

    cockpit.restart()
    check("restart cleared the chat", cockpit.transcript == [], str(cockpit.transcript))
    check("restart dropped the model's memory of the run", cockpit.session is None)
    check("restart left the cockpit ready for the next instruction", not cockpit.busy)

    cockpit.submit("drive somewhere else")
    deadline = time.time() + 5
    while cockpit.busy and time.time() < deadline:
        time.sleep(0.02)
    check("the next instruction started a fresh conversation",
          cockpit.session is not None and cockpit.session is not first_session)
    check("only the new instruction is in the transcript",
          [e["text"] for e in cockpit.transcript] == ["drive somewhere else",
                                                      "did drive somewhere else"],
          str([e["text"] for e in cockpit.transcript]))


def main() -> int:
    from agent import Session, make_client

    nt_port, cam_port, api_port = free_port(), free_port(), free_port()
    start_camera(cam_port)
    robot_sim = FakeRobot(nt_port, f"http://127.0.0.1:{cam_port}/stream.mjpg").start()
    gemini = FakeGemini(api_port, SCRIPT).start()
    os.environ["GEMINI_BASE_URL"] = f"http://127.0.0.1:{api_port}"

    robot = RobotConnection("127.0.0.1", nt_port).connect()
    checks: list[tuple[str, bool, str]] = []

    def check(label: str, ok: bool, detail: str = "") -> None:
        checks.append((label, bool(ok), detail))

    check("connected and read AvailableActions",
          robot.available_actions() == [
              "STOP", "INTAKE", "STOW_INTAKE", "COLLECT_FUEL", "GRAB_FUEL",
              "AUTO_FOLD_ON", "AUTO_FOLD_OFF", "SHOOT_FUEL", "ALIGN_HUB",
          ],
          str(robot.available_actions()))
    check("read Landmarks as JSON", robot.landmarks() == {"our_hub": [9.0, 4.0, 0.0]},
          str(robot.landmarks()))
    check("found the camera stream", "fuel-cam-processed" in robot.camera_streams(),
          str(robot.camera_streams()))

    check_defensive_json_parsing(check)
    check_v2_contract(check, robot, robot_sim)

    schema = tool_declarations(robot.available_actions())
    tools = RobotTools(robot)
    check("tool declarations are well-formed function specs",
          all(t.get("type") == "function" and t.get("name") and "parameters" in t
              for t in schema),
          str(schema))
    check("no duplicate tool names",
          len({t["name"] for t in schema}) == len(schema), str([t["name"] for t in schema]))
    check("tool declarations match the handler map exactly",
          {t["name"] for t in schema} == set(tools.handlers),
          str(sorted({t["name"] for t in schema} ^ set(tools.handlers))))
    check("tool schema offers the robot's own actions",
          schema[1]["parameters"]["properties"]["action"]["enum"] == robot.available_actions())

    session = Session(make_client("test-key"), robot, "gemini-robotics-er-2-preview", 12, False)
    check("system prompt carries the robot's notes and landmarks",
          "Test notes from the robot." in session.system_instruction
          and "our_hub" in session.system_instruction)
    check("system prompt folds in the game brief and the zones",
          "REBUILT: collect fuel" in session.system_instruction
          and "left_trench" in session.system_instruction
          and "our_alliance_zone" in session.system_instruction)

    answer = session.ask("drive to the hub, look, and intake")
    print("\nmodel's final answer:", answer, "\n")

    check("the model's closing words came back as output_text",
          answer.startswith("Drove to (6.4, 4.0)"), answer)

    writes = dict(robot_sim.writes)
    check("model's drive_to reached the robot as a TargetPose",
          ("/AIControl/TargetPose", [6.4, 4.0, 0.0]) in robot_sim.writes,
          str([w for w in robot_sim.writes if w[0] == "/AIControl/TargetPose"]))
    check("max_speed reached the robot", writes.get("/AIControl/MaxSpeed") == 4.0,
          str(writes.get("/AIControl/MaxSpeed")))
    check("INTAKE reached the robot as an ActionTrigger",
          ("/AIControl/ActionTrigger", "INTAKE") in robot_sim.writes, str(robot_sim.writes))
    check("an unknown action was refused, not written",
          ("/AIControl/ActionTrigger", "FLY") not in robot_sim.writes)

    sent = gemini.requests
    check("eight interactions: seven tool rounds and the answer", len(sent) == 8, str(len(sent)))
    check("system_instruction sent every time",
          all("system_instruction" in r for r in sent))
    check("tools sent every time", all(r.get("tools") for r in sent))
    check("conversation continued with previous_interaction_id",
          [r.get("previous_interaction_id") for r in sent[1:]]
          == ["int_1", "int_2", "int_3", "int_4", "int_5", "int_6", "int_7"],
          str([r.get("previous_interaction_id") for r in sent[1:]]))

    results = [r["input"][0] for r in sent[1:]]
    check("results came back as function_result blocks matched by call_id",
          [(x["type"], x["call_id"], x["name"]) for x in results]
          == [("function_result", "c1", "drive_to"),
              ("function_result", "c2", "look_at_field"),
              ("function_result", "c2b", "look_through_camera"),
              ("function_result", "c3", "run_action"),
              ("function_result", "c4", "run_action"),
              ("function_result", "c5", "run_action"),
              ("function_result", "c6", "drive_to")],
          str([(x["type"], x["call_id"]) for x in results]))
    check("the field view went to the model as an image block",
          any(c.get("type") == "image" for c in results[1]["result"]),
          str([c.get("type") for c in results[1]["result"]]))
    opening = sent[0]["input"][0]
    check("every instruction opens with a field view image",
          opening["type"] == "user_input"
          and [c["type"] for c in opening["content"]] == ["text", "text", "image"],
          json.dumps(opening)[:160])
    check("the camera frame went to the model as an image block",
          any(c.get("type") == "image" for c in results[2]["result"]),
          str([c.get("type") for c in results[2]["result"]]))
    check("the refused action was flagged as an error result", results[4].get("is_error") is True)
    check("tool results carry the robot's live pose",
          json.loads(results[0]["result"][0]["text"])["pose"] == {"x": 6.4, "y": 4.0, "heading_deg": 0.0},
          results[0]["result"][0]["text"][:120])

    driving_state = json.loads(results[6]["result"][0]["text"])
    check("a drive_to returned while the intake was still running",
          driving_state["running_actions"] == ["INTAKE"]
          and driving_state["pose"] == {"x": 3.0, "y": 2.0, "heading_deg": 90.0},
          str(driving_state.get("running_actions")))

    robot.reset_simulation()
    deadline = time.time() + 5
    while time.time() < deadline and (
        "/AIControl/ResetSimulation", False
    ) not in robot_sim.writes:
        time.sleep(0.02)
    check("reset_simulation reached the robot on its own topic, not as an action",
          robot_sim.resets == 1
          and not any(name == "/AIControl/ActionTrigger" and value == "RESET_SIMULATION"
                      for name, value in robot_sim.writes),
          str(robot_sim.resets))
    check("restarting is not offered to the model as an action",
          not any("RESET" in t.upper() for t in robot.available_actions()),
          str(robot.available_actions()))
    reset_writes = [v for name, v in robot_sim.writes if name == "/AIControl/ResetSimulation"]
    check("the reset topic was pulsed and left back at false",
          reset_writes == [True, False], str(reset_writes))

    check_cockpit_restart(check, robot)

    robot.close()

    print()
    for label, ok, detail in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f"   [{detail}]" if not ok and detail else ""))
    failures = sum(1 for _l, ok, _d in checks if not ok)
    print(f"\n{len(checks) - failures}/{len(checks)} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
