"""Minimal NT4 client for the robot's /AIControl API.

No robotpy dependency - just websockets and msgpack, the same shape as
scripts/ai_demo/record_demo.py. The client runs its own asyncio loop on a
background thread and exposes a plain blocking API, because the Gemini tool
handlers are ordinary synchronous functions.
"""

from __future__ import annotations

import asyncio
import json
import socket
import struct
import threading
import time
import urllib.parse
from typing import Any

import msgpack
import websockets

# NT4 type ids for the types this client writes.
TYPE_DOUBLE = 1
TYPE_STRING = 4
TYPE_DOUBLE_ARRAY = 17

SUBPROTOCOLS = ["v4.1.networktables.first.wpi.edu", "networktables.first.wpi.edu"]

# How long to wait before dialling the robot again after the connection drops.
RECONNECT_DELAY_S = 1.0

TABLE = "/AIControl/"

# Logged robot state the field view draws, on top of the /AIControl API itself.
GAME_PIECES = "/AdvantageKit/RealOutputs/Vision/GamePieces/TargetPoses"
FIELD_PIECES = "/AdvantageKit/RealOutputs/Sim/Fuel/Positions"
TRAJECTORY = "/AdvantageKit/RealOutputs/Odometry/Trajectory"

EXTRA_TOPICS = ["/CameraPublisher/", GAME_PIECES, FIELD_PIECES, TRAJECTORY]

# pubuids are ours to choose; they only have to be unique within this client.
_PUB = {
    "TargetPose": (10, TYPE_DOUBLE_ARRAY, "double[]"),
    "ActionTrigger": (11, TYPE_STRING, "string"),
    "MaxSpeed": (12, TYPE_DOUBLE, "double"),
    "MaxAccel": (13, TYPE_DOUBLE, "double"),
}


class RobotConnection:
    """Live view of /AIControl/, plus the four topics an agent is allowed to write."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5810, name: str = "gemini-agent"):
        self._host = host
        self._uri = f"ws://{host}:{port}/nt/{name}"
        self._values: dict[str, Any] = {}
        self._topics: dict[int, str] = {}
        self._lock = threading.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws = None
        self._ready = threading.Event()
        self._error: BaseException | None = None
        self._connected = False
        self._closing = False
        self._thread = threading.Thread(target=self._run, daemon=True, name="nt4")

    # -- lifecycle ---------------------------------------------------------

    def connect(self, timeout: float = 10.0) -> "RobotConnection":
        self._thread.start()
        if not self._ready.wait(timeout):
            raise TimeoutError(f"no NetworkTables server at {self._uri}")
        if self._error is not None:
            raise self._error
        # Give the server a moment to announce and send the retained /AIControl values.
        deadline = time.time() + timeout
        while time.time() < deadline and self.get("AvailableActions") is None:
            time.sleep(0.05)
        if self.get("AvailableActions") is None:
            raise TimeoutError(
                "connected, but /AIControl/ never appeared - is this robot code running the "
                "AIControlBridge?"
            )
        return self

    @property
    def connected(self) -> bool:
        """False while the robot is away and the client is retrying."""
        return self._connected

    def close(self) -> None:
        """Closes the socket and stops the loop, so nothing is left dangling."""
        self._closing = True
        if self._loop is None:
            return
        if self._ws is not None:
            try:
                asyncio.run_coroutine_threadsafe(self._ws.close(), self._loop).result(timeout=2)
            except Exception:
                pass
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except RuntimeError:
            pass  # the loop already stopped when the socket closed
        self._thread.join(timeout=2)

    def __enter__(self) -> "RobotConnection":
        return self.connect()

    def __exit__(self, *_exc) -> None:
        self.close()

    # -- reads -------------------------------------------------------------

    def get(self, key: str, default: Any = None) -> Any:
        """Reads a topic. Bare names are /AIControl/ topics; absolute paths are absolute."""
        name = key if key.startswith("/") else TABLE + key
        with self._lock:
            return self._values.get(name, default)

    def state(self) -> dict[str, Any]:
        """Everything the agent is allowed to see, in one snapshot."""
        pose = self.get("RobotPose") or []
        target = self.get("ActiveTarget") or []
        return {
            "pose": _pose_dict(pose),
            "navigating": bool(self.get("Navigating", False)),
            "action_running": bool(self.get("ActionRunning", False)),
            "rotation_locked": bool(self.get("RotationLocked", False)),
            "in_shooting_zone": bool(self.get("InShootingZone", False)),
            "at_target": bool(self.get("AtTarget", False)),
            "active_target": _pose_dict(target) if target else None,
            "status": self.get("Status", ""),
            "last_action": self.get("LastAction", ""),
            "last_error": self.get("LastError", ""),
            "max_speed_mps": self.get("MaxSpeed"),
            "max_accel_mps2": self.get("MaxAccel"),
        }

    def available_actions(self) -> list[str]:
        return list(self.get("AvailableActions") or [])

    def notes(self) -> str:
        return self.get("Notes", "")

    def landmarks(self) -> dict[str, list[float]]:
        raw = self.get("Landmarks", "")
        try:
            return json.loads(raw) if raw else {}
        except json.JSONDecodeError:
            return {}

    def field_size(self) -> list[float]:
        return list(self.get("FieldSize") or [])

    def camera_streams(self) -> dict[str, str]:
        """Camera name -> MJPEG stream URL, from /CameraPublisher."""
        streams: dict[str, str] = {}
        with self._lock:
            items = list(self._values.items())
        for topic, value in items:
            if not topic.startswith("/CameraPublisher/") or not topic.endswith("/streams"):
                continue
            name = topic[len("/CameraPublisher/") : -len("/streams")]
            urls = [u for u in (value or []) if isinstance(u, str)]
            mjpg = [u[len("mjpg:") :] for u in urls if u.startswith("mjpg:")]
            if mjpg:
                streams[name] = self._reachable(mjpg[0])
        return streams

    def _reachable(self, url: str) -> str:
        """The simulator advertises its streams under its own hostname, which is not
        necessarily resolvable from wherever the agent runs. When it is not, the
        streams are on the same machine as the NetworkTables server."""
        parts = urllib.parse.urlsplit(url)
        try:
            socket.getaddrinfo(parts.hostname, None)
            return url
        except socket.gaierror:
            netloc = self._host + (f":{parts.port}" if parts.port else "")
            return urllib.parse.urlunsplit(parts._replace(netloc=netloc))

    def detected_game_pieces(self) -> list[tuple[float, float]]:
        """Fuel the game-piece camera can see, on the field. WPILib Pose3d structs."""
        return _decode_poses(self.get(GAME_PIECES, b""), stride=56)

    def field_game_pieces(self) -> list[tuple[float, float]]:
        """Every fuel on the field, from the physics sim. Ground truth, so it is for
        the human watching - the model is only told what the camera can see."""
        return _decode_poses(self.get(FIELD_PIECES, b""), stride=24)

    def trajectory(self) -> list[tuple[float, float]]:
        """The path the robot is currently following. WPILib Pose2d structs."""
        return _decode_poses(self.get(TRAJECTORY, b""), stride=24)

    def busy(self) -> bool:
        return bool(self.get("Navigating", False)) or bool(self.get("ActionRunning", False))

    # -- writes ------------------------------------------------------------

    def set_target_pose(self, x: float, y: float, heading_deg: float) -> None:
        self._publish("TargetPose", [float(x), float(y), float(heading_deg)])

    def set_action(self, action: str) -> None:
        self._publish("ActionTrigger", str(action))

    def set_limits(self, max_speed: float | None, max_accel: float | None) -> None:
        if max_speed is not None:
            self._publish("MaxSpeed", float(max_speed))
        if max_accel is not None:
            self._publish("MaxAccel", float(max_accel))

    def wait_until_idle(self, timeout: float = 25.0, settle: float = 0.4) -> bool:
        """Blocks until the robot stops navigating and running an action.

        Returns True if it went idle, False on timeout. `settle` covers the gap
        between writing a topic and the robot scheduling the command, so a call
        that has not started yet is not mistaken for one that has finished.
        """
        deadline = time.time() + timeout
        start = time.time()
        while time.time() < deadline:
            if not self.busy() and time.time() - start > settle:
                return True
            time.sleep(0.05)
        return False

    # -- plumbing ----------------------------------------------------------

    def _publish(self, key: str, value: Any) -> None:
        pubuid, type_id, _ = _PUB[key]
        payload = msgpack.packb([pubuid, int(time.time() * 1e6), type_id, value])
        if self._loop is None or self._ws is None:
            raise RuntimeError("not connected")
        asyncio.run_coroutine_threadsafe(self._ws.send(payload), self._loop).result(timeout=5)

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._client())
        except BaseException as exc:  # surfaced to connect()
            self._error = exc
            self._ready.set()
        finally:
            self._ws = None
            try:
                self._loop.run_until_complete(self._loop.shutdown_asyncgens())
            except RuntimeError:
                pass
            self._loop.close()

    async def _client(self) -> None:
        """Stays connected. A dashboard is left open for hours; robot code is not,
        so losing the robot is a state to report and recover from, not a crash."""
        first = True
        while not self._closing:
            try:
                await self._session()
            except Exception as exc:
                if first:
                    self._error = exc
                    self._ready.set()
                    return
            self._ws = None
            self._connected = False
            first = False
            await asyncio.sleep(RECONNECT_DELAY_S)

    async def _session(self) -> None:
        async with websockets.connect(
            self._uri, subprotocols=SUBPROTOCOLS, max_size=None
        ) as ws:
            self._ws = ws
            # Topic ids are handed out per connection, so a reconnect starts over.
            with self._lock:
                self._topics.clear()
            await ws.send(
                json.dumps(
                    [
                        {
                            "method": "subscribe",
                            "params": {
                                "topics": [TABLE] + EXTRA_TOPICS,
                                "subuid": 1,
                                "options": {"prefix": True, "all": True, "periodic": 0.05},
                            },
                        }
                    ]
                    + [
                        {
                            "method": "publish",
                            "params": {
                                "name": TABLE + key,
                                "pubuid": pubuid,
                                "type": type_name,
                                "properties": {},
                            },
                        }
                        for key, (pubuid, _, type_name) in _PUB.items()
                    ]
                )
            )
            self._connected = True
            self._ready.set()
            async for message in ws:
                self._handle(message)

    def _handle(self, message) -> None:
        if isinstance(message, str):
            for item in json.loads(message):
                if item.get("method") == "announce":
                    self._topics[item["params"]["id"]] = item["params"]["name"]
            return
        unpacker = msgpack.Unpacker(raw=False, use_list=True)
        unpacker.feed(message)
        with self._lock:
            for topic_id, _ts, _type, value in unpacker:
                name = self._topics.get(topic_id)
                if name:
                    self._values[name] = value


def _pose_dict(pose) -> dict[str, float] | None:
    if not pose or len(pose) < 3:
        return None
    return {"x": round(pose[0], 3), "y": round(pose[1], 3), "heading_deg": round(pose[2], 1)}


def _decode_poses(raw, stride: int) -> list[tuple[float, float]]:
    """X and Y out of an array of WPILib geometry structs.

    Every one of them - Pose2d, Pose3d, Translation3d - starts with the
    translation as little-endian doubles, so only the stride differs.
    """
    if not isinstance(raw, (bytes, bytearray)) or stride < 16:
        return []
    return [
        struct.unpack_from("<dd", raw, offset)
        for offset in range(0, len(raw) - stride + 1, stride)
    ]
