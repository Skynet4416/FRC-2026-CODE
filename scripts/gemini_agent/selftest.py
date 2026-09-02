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
from robot_tools import tool_declarations  # noqa: E402

# A 1x1 black JPEG, so `look` has something real to decode and forward.
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
            "/AIControl/AvailableActions": ["STOP", "INTAKE", "SHOOT_FUEL", "ALIGN_HUB"],
            "/AIControl/HeadingUnits": "degrees",
            "/AIControl/Notes": "Test notes from the robot.",
            "/AIControl/Landmarks": '{"our_hub":[9.000,4.000,0.0]}',
            "/AIControl/FieldSize": [17.55, 8.05],
            "/CameraPublisher/fuel-cam-processed/streams": [f"mjpg:{camera_url}"],
        }
        self.types = {
            bool: 0, float: 1, str: 4, int: 1,
        }
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
            await ws.send(json.dumps([
                {"method": "announce",
                 "params": {"name": name, "id": ids[name], "type": _nt_type(value),
                            "pubuid": None, "properties": {}}}
                for name, value in self.values.items()
            ]))
            for name, value in self.values.items():
                await ws.send(msgpack.packb([ids[name], int(time.time() * 1e6),
                                             _nt_type_id(value), value]))

        async for message in ws:
            if isinstance(message, str):
                for item in json.loads(message):
                    params = item.get("params", {})
                    if item.get("method") == "publish":
                        pubs[params["pubuid"]] = params["name"]
                    elif item.get("method") == "subscribe":
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
            except websockets.exceptions.ConnectionClosed:
                return  # the client hung up mid-update; nothing left to tell it

    def _apply(self, name: str, value) -> None:
        """The bit of bridge behaviour the tools actually depend on."""
        if name == "/AIControl/TargetPose" and value:
            self.values["/AIControl/RobotPose"] = list(value)
            self.values["/AIControl/ActiveTarget"] = list(value)
            self.values["/AIControl/AtTarget"] = True
            self.values["/AIControl/Status"] = "AT TARGET"
        elif name == "/AIControl/ActionTrigger" and value:
            self.values["/AIControl/LastAction"] = value
            self.values["/AIControl/Status"] = f"RUNNING {value}"
        elif name in ("/AIControl/MaxSpeed", "/AIControl/MaxAccel"):
            self.values[name] = value


def _nt_type(value) -> str:
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, str):
        return "string"
    if isinstance(value, (int, float)):
        return "double"
    if value and isinstance(value[0], str):
        return "string[]"
    return "double[]"


def _nt_type_id(value) -> int:
    return {"boolean": 0, "double": 1, "string": 4, "double[]": 17, "string[]": 20}[_nt_type(value)]


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
    {"steps": [{"type": "model_output",
                "content": [{"type": "text",
                             "text": "Drove to (6.4, 4.0), looked, and intook. FLY is not a real "
                                     "action, so I skipped it."}]}]},
]


# --------------------------------------------------------------------------


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
          robot.available_actions() == ["STOP", "INTAKE", "SHOOT_FUEL", "ALIGN_HUB"],
          str(robot.available_actions()))
    check("read Landmarks as JSON", robot.landmarks() == {"our_hub": [9.0, 4.0, 0.0]},
          str(robot.landmarks()))
    check("found the camera stream", "fuel-cam-processed" in robot.camera_streams(),
          str(robot.camera_streams()))

    schema = tool_declarations(robot.available_actions())
    check("the field view is offered as a tool",
          {t["name"] for t in schema} == {"drive_to", "run_action", "get_robot_state",
                                          "look_at_field", "look_through_camera", "wait"},
          str([t["name"] for t in schema]))
    check("tool schema offers the robot's own actions",
          schema[1]["parameters"]["properties"]["action"]["enum"] == robot.available_actions())

    session = Session(make_client("test-key"), robot, "gemini-robotics-er-2-preview", 12, False)
    check("system prompt carries the robot's notes and landmarks",
          "Test notes from the robot." in session.system_instruction
          and "our_hub" in session.system_instruction)

    answer = session.ask("drive to the hub, look, and intake")
    print("\nmodel's final answer:", answer, "\n")

    check("the model's closing words came back as output_text",
          answer.startswith("Drove to (6.4, 4.0)"), answer)

    writes = dict(robot_sim.writes)
    check("model's drive_to reached the robot as a TargetPose",
          writes.get("/AIControl/TargetPose") == [6.4, 4.0, 0.0], str(writes.get("/AIControl/TargetPose")))
    check("max_speed reached the robot", writes.get("/AIControl/MaxSpeed") == 4.0,
          str(writes.get("/AIControl/MaxSpeed")))
    check("INTAKE reached the robot as an ActionTrigger",
          ("/AIControl/ActionTrigger", "INTAKE") in robot_sim.writes, str(robot_sim.writes))
    check("an unknown action was refused, not written",
          ("/AIControl/ActionTrigger", "FLY") not in robot_sim.writes)

    sent = gemini.requests
    check("six interactions: five tool rounds and the answer", len(sent) == 6, str(len(sent)))
    check("system_instruction sent every time",
          all("system_instruction" in r for r in sent))
    check("tools sent every time", all(r.get("tools") for r in sent))
    check("conversation continued with previous_interaction_id",
          [r.get("previous_interaction_id") for r in sent[1:]]
          == ["int_1", "int_2", "int_3", "int_4", "int_5"],
          str([r.get("previous_interaction_id") for r in sent[1:]]))

    results = [r["input"][0] for r in sent[1:]]
    check("results came back as function_result blocks matched by call_id",
          [(x["type"], x["call_id"], x["name"]) for x in results]
          == [("function_result", "c1", "drive_to"),
              ("function_result", "c2", "look_at_field"),
              ("function_result", "c2b", "look_through_camera"),
              ("function_result", "c3", "run_action"),
              ("function_result", "c4", "run_action")],
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

    robot.close()

    print()
    for label, ok, detail in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f"   [{detail}]" if not ok and detail else ""))
    failures = sum(1 for _l, ok, _d in checks if not ok)
    print(f"\n{len(checks) - failures}/{len(checks)} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
