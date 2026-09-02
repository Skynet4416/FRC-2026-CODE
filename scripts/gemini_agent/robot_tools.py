"""The tools Gemini Robotics-ER is given, and what they do to the robot.

Every tool is a thin wrapper over the /AIControl NetworkTables topics: nothing
here knows about the robot's mechanisms, and the model never touches
NetworkTables itself. Tool results are JSON, plus a camera image for `look`.
"""

from __future__ import annotations

import base64
import time
from typing import Any, Callable

import field_view
from camera import grab_frame
from nt4 import RobotConnection

# How long a tool waits for the robot before it reports back anyway. A path that
# has not arrived by then is not an error - the model gets the live state and can
# wait again - but the bridge's own 20 s timeout will have reported a failure.
DRIVE_TIMEOUT_S = 30.0
ACTION_TIMEOUT_S = 30.0


def tool_declarations(actions: list[str]) -> list[dict[str, Any]]:
    """Function declarations for interactions.create(tools=...).

    `actions` comes from the robot's own /AIControl/AvailableActions, so a newly
    registered action shows up in the schema without touching this file.
    """
    return [
        {
            "type": "function",
            "name": "drive_to",
            "description": (
                "Drive the robot to a field pose with PathPlanner pathfinding, avoiding the "
                "field's obstacles. Returns the robot state once it arrives or gives up."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "Field X in metres, 0 at the blue wall."},
                    "y": {
                        "type": "number",
                        "description": "Field Y in metres, 0 at the scoring-table wall.",
                    },
                    "heading_deg": {
                        "type": "number",
                        "description": "Heading in degrees, 0 = facing the red wall (+X).",
                    },
                    "max_speed": {
                        "type": "number",
                        "description": "Speed limit in m/s, 0.3-4.5. Use 4+ to charge a bump.",
                    },
                    "max_accel": {
                        "type": "number",
                        "description": "Acceleration limit in m/s^2, 0.3-8.0.",
                    },
                    "wait": {
                        "type": "boolean",
                        "description": "Block until the robot arrives (default true).",
                    },
                },
                "required": ["x", "y", "heading_deg"],
            },
        },
        {
            "type": "function",
            "name": "run_action",
            "description": (
                "Run one of the robot's mechanism actions. Shooting actions take over the "
                "drivetrain and drive into the alliance zone first if the robot is not already "
                "there. STOP cancels the running path and action."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions},
                    "wait": {
                        "type": "boolean",
                        "description": "Block until the action finishes (default true).",
                    },
                },
                "required": ["action"],
            },
        },
        {
            "type": "function",
            "name": "get_robot_state",
            "description": "Read the robot's pose, status, and what it is currently doing.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "type": "function",
            "name": "look_at_field",
            "description": (
                "Look at the field from above: a top-down map in the same coordinates these "
                "tools take, with a one-metre grid, the obstacles the robot cannot drive "
                "through, the robot and its heading, the fuel its camera can see, the path it "
                "is following and the pose it was asked for. This is the view to use for "
                "anything about position, route or layout."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "type": "function",
            "name": "look_through_camera",
            "description": (
                "Take a picture through one of the robot's cameras. Only useful for detail the "
                "map cannot show - what a piece of fuel looks like up close, whether a tag is "
                "in view. For position and routing use look_at_field instead."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "camera": {
                        "type": "string",
                        "description": (
                            "Camera name. Omit for the game-piece camera. Call with an unknown "
                            "name to get the list back."
                        ),
                    }
                },
            },
        },
        {
            "type": "function",
            "name": "wait",
            "description": "Let time pass, then read the robot state again.",
            "parameters": {
                "type": "object",
                "properties": {
                    "seconds": {"type": "number", "description": "0.1 to 20 seconds."}
                },
                "required": ["seconds"],
            },
        },
    ]


class RobotTools:
    """Executes the tool calls against a live robot connection."""

    def __init__(self, robot: RobotConnection, on_call: Callable[[str, dict], None] | None = None):
        self.robot = robot
        self.on_call = on_call or (lambda name, args: None)
        self.handlers: dict[str, Callable[..., dict]] = {
            "drive_to": self.drive_to,
            "run_action": self.run_action,
            "get_robot_state": self.get_robot_state,
            "look_at_field": self.look_at_field,
            "look_through_camera": self.look_through_camera,
            "wait": self.wait,
        }

    def call(self, name: str, arguments: dict) -> tuple[dict, bytes | None]:
        """Runs one tool call. Returns (json result, optional JPEG to show the model)."""
        self.on_call(name, arguments)
        handler = self.handlers.get(name)
        if handler is None:
            return {"error": f"unknown tool {name!r}"}, None
        try:
            result = handler(**arguments)
        except TypeError as exc:
            return {"error": f"bad arguments for {name}: {exc}"}, None
        except Exception as exc:  # a tool failure is information, not a crash
            return {"error": f"{type(exc).__name__}: {exc}"}, None
        image = result.pop("_image", None)
        return result, image

    # -- tools -------------------------------------------------------------

    def drive_to(
        self,
        x: float,
        y: float,
        heading_deg: float,
        max_speed: float | None = None,
        max_accel: float | None = None,
        wait: bool = True,
    ) -> dict:
        self.robot.set_limits(max_speed, max_accel)
        self.robot.set_target_pose(x, y, heading_deg)
        if wait:
            self.robot.wait_until_idle(timeout=DRIVE_TIMEOUT_S)
        else:
            time.sleep(0.3)
        return self._state_with("driving to (%.2f, %.2f, %.1f deg)" % (x, y, heading_deg))

    def run_action(self, action: str, wait: bool = True) -> dict:
        available = self.robot.available_actions()
        if action.upper() not in {a.upper() for a in available}:
            return {"error": f"no such action {action!r}", "available_actions": available}
        self.robot.set_action(action.upper())
        if wait and action.upper() != "STOP":
            self.robot.wait_until_idle(timeout=ACTION_TIMEOUT_S)
        else:
            time.sleep(0.3)
        return self._state_with(f"ran {action.upper()}")

    def get_robot_state(self) -> dict:
        return self._state_with(None)

    def wait(self, seconds: float) -> dict:
        time.sleep(max(0.1, min(20.0, float(seconds))))
        return self._state_with(f"waited {seconds}s")

    def look_at_field(self) -> dict:
        image = field_view.render(self.robot, self.robot.landmarks())
        state = self.robot.state()
        pieces = self.robot.detected_game_pieces()
        return {
            "view": "top-down field map, attached as an image",
            "note": (
                "X increases to the right, Y upwards, grid lines every metre. Grey cells are "
                "obstacles the pathfinder will not cross."
            ),
            "fuel_seen_by_camera": [
                {"x": round(x, 2), "y": round(y, 2)} for x, y in pieces
            ],
            "robot": state,
            "_image": image,
        }

    def look_through_camera(self, camera: str | None = None) -> dict:
        streams = self.robot.camera_streams()
        if not streams:
            return {"error": "no cameras are publishing streams"}
        name = camera or _default_camera(streams)
        if name not in streams:
            return {"error": f"no camera {name!r}", "cameras": sorted(streams)}
        frame = grab_frame(streams[name])
        return {
            "camera": name,
            "note": "The image is attached. It is a live view through this camera.",
            "robot": self.robot.state(),
            "_image": frame,
        }

    # -- helpers -----------------------------------------------------------

    def _state_with(self, did: str | None) -> dict:
        state = self.robot.state()
        if did:
            state["requested"] = did
        return state


def _default_camera(streams: dict[str, str]) -> str:
    """The game-piece camera's processed stream, which has the detections drawn on."""
    def rank(name: str) -> tuple[int, int, str]:
        piece = "fuel" in name.lower() or "object" in name.lower()
        return (0 if piece else 1, 0 if name.endswith("-processed") else 1, name)

    return sorted(streams, key=rank)[0]


def encode_image(image: bytes) -> dict[str, Any]:
    """An image as an Interactions API image content block."""
    mime = "image/png" if image[:8] == b"\x89PNG\r\n\x1a\n" else "image/jpeg"
    return {
        "type": "image",
        "mime_type": mime,
        "data": base64.b64encode(image).decode("ascii"),
    }
