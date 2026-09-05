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

# Orange-blob detection on look_through_camera frames is a nice-to-have for whoever
# is watching, not something the model depends on - cv2 stays optional so a
# machine without it still gets the plain frame instead of a broken tool.
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

# How long a tool waits for the robot before it reports back anyway. A path that
# has not arrived by then is not an error - the model gets the live state and can
# wait again - but the bridge's own 20 s timeout will have reported a failure.
DRIVE_TIMEOUT_S = 30.0
ACTION_TIMEOUT_S = 30.0
# COLLECT_FUEL and GRAB_FUEL are whole autonomous manoeuvres, not one mechanism
# spinning up - they can legitimately run far longer than any other single action.
COLLECT_TIMEOUT_S = 90.0


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
                "field's obstacles. Returns the robot state once it arrives or gives up. Any "
                "action that does not need the drivetrain - the intake, for one - keeps running "
                "while the robot drives. While the intake is down the robot clamps you to 1.5 "
                "m/s no matter what max_speed you ask for, and fuel is only collected by "
                "driving over it with the intake down. That makes this the slow, manual way to "
                "pick up a piece - grab_fuel and collect_fuel do the same driving for you, so "
                "prefer those for fuel and save drive_to for getting somewhere or lining up a "
                "shot."
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
                "Run one of the robot's mechanism actions. Actions on different mechanisms run "
                "at the same time, so call this with wait=false and then drive_to to intake "
                "while driving; only actions that need the same mechanism replace each other. "
                "INTAKE lowers the intake and latches it down, rollers running, until "
                "STOW_INTAKE or STOP - it never folds itself, so there is no need to "
                "re-trigger it, and the intake's normal resting state is down everywhere on "
                "the field, not just while you are chasing a piece. The robot folds it by "
                "itself only to cross the trench and the bumps, lowering it again once clear "
                "- see folded_for in the robot state for when this is happening. AUTO_FOLD_OFF "
                "hands you manual control of that fold; AUTO_FOLD_ON gives it back. "
                "COLLECT_FUEL and GRAB_FUEL read how many pieces to gather from CollectTarget "
                "- call the collect_fuel/grab_fuel tools instead of this one for those, since "
                "they set it for you. Shooting actions take over the drivetrain and drive into "
                "the alliance zone first if the robot is not already there. STOP cancels "
                "everything."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions},
                    "wait": {
                        "type": "boolean",
                        "description": (
                            "Block until this action finishes (default true). Pass false to "
                            "leave it running and do something else in the meantime."
                        ),
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
                "through, the robot and its heading, every fuel piece on the field (faint) "
                "with the ones its camera can see highlighted, the field's zones including "
                "the depots (a filled gold box - a preloaded pile of fuel), the path it is "
                "following and the pose it was asked for. This is the view to use for "
                "anything about position, route or layout - for where fuel actually is, "
                "find_fuel has the numbers this map only draws."
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
        {
            "type": "function",
            "name": "find_fuel",
            "description": (
                "Where fuel actually is on the field right now - ground truth from the robot, "
                "not a guess from a camera frame. Per-zone counts and centres, the nearest "
                "pieces, the biggest clusters, what the camera itself can currently see, and a "
                "ready-made pickup pose and heading. Call this to decide where fuel is worth "
                "going for, instead of hunting for it with look_at_field or "
                "look_through_camera. It only tells you where fuel is - use grab_fuel for a "
                "single nearby piece, collect_fuel to sweep a whole line, or drive_to plus "
                "INTAKE for anywhere neither can reach."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "type": "function",
            "name": "grab_fuel",
            "description": (
                "The default reflex when there is fuel near you: the robot looks for it with "
                "its camera, drives itself onto the nearest piece and takes it. You do not "
                "need to work out a pose, a heading or a speed, and you do not need to place "
                "the ball on your left flank yourself - the robot does. Prefer this over "
                "drive_to for a piece or two right in front of you; use collect_fuel for a "
                "whole line of fuel, and find_fuel first if nothing is in reach yet."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "count": {
                        "type": "number",
                        "description": "How many pieces to grab before stopping, 1-20. Default 1.",
                    },
                    "wait": {
                        "type": "boolean",
                        "description": "Block until the grab finishes (default true).",
                    },
                },
            },
        },
        {
            "type": "function",
            "name": "collect_fuel",
            "description": (
                "The preferred way to gather a whole line of fuel: the robot lowers the "
                "intake, plans a slow sweep so the fuel passes down its left flank, and drives "
                "it - you do not need to drive or manage the intake yourself. Use find_fuel() "
                "first to see where a sweep is worth making, or grab_fuel() instead for a "
                "single nearby piece that does not need a whole sweep. Leaves the intake down "
                "when it finishes."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "count": {
                        "type": "number",
                        "description": (
                            "How many pieces to gather before stopping, 1-150 (hopper "
                            "capacity). Default 10."
                        ),
                    },
                    "wait": {
                        "type": "boolean",
                        "description": "Block until the sweep finishes (default true).",
                    },
                },
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
            "find_fuel": self.find_fuel,
            "grab_fuel": self.grab_fuel,
            "collect_fuel": self.collect_fuel,
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
            # Only the path: an intake or a spin-up running alongside it is not this
            # call's business, and waiting for it would undo the point of running both.
            self.robot.wait_until_arrived(timeout=DRIVE_TIMEOUT_S)
        else:
            time.sleep(0.3)
        return self._state_with("driving to (%.2f, %.2f, %.1f deg)" % (x, y, heading_deg))

    def run_action(self, action: str, wait: bool = True) -> dict:
        available = self.robot.available_actions()
        if action.upper() not in {a.upper() for a in available}:
            return {"error": f"no such action {action!r}", "available_actions": available}
        self.robot.set_action(action.upper())
        if wait and action.upper() != "STOP":
            self.robot.wait_until_action_done(action, timeout=ACTION_TIMEOUT_S)
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
        fuel = self.robot.fuel()
        return {
            "view": "top-down field map, attached as an image",
            "note": (
                "X increases to the right, Y upwards, grid lines every metre. Grey cells are "
                "obstacles the pathfinder will not cross, faint dots are every fuel piece on "
                "the field, bright orange dots are fuel the camera can actually see, dashed "
                "boxes are the field's zones (red ones fold the intake), and filled gold "
                "boxes are the depots - a preloaded pile of fuel against the wall."
            ),
            "fuel_seen_by_camera": [
                {"x": round(x, 2), "y": round(y, 2)} for x, y in pieces
            ],
            "fuel_by_zone": state.get("fuel_by_zone", {}),
            # From the Fuel JSON directly: bearing/range detections plus the label
            # for where they came from - never let a sim stand-in pass as the real
            # detector.
            "seen_by_camera": fuel.get("seen_by_camera", []),
            "vision_source": fuel.get("vision_source"),
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
            "_image": _annotate_orange_blobs(frame),
        }

    def find_fuel(self) -> dict:
        fuel = self.robot.fuel()
        if not fuel or not fuel.get("available", False):
            return {
                "available": False,
                "note": (
                    "the robot is not publishing ground-truth fuel positions - older robot "
                    "code, or a real field - so use look_at_field / look_through_camera to "
                    "find fuel visually instead"
                ),
            }
        return {
            "available": True,
            "source": fuel.get("source"),
            "vision_source": fuel.get("vision_source"),
            "total_on_field": fuel.get("total_on_field"),
            "on_board": fuel.get("on_board"),
            "capacity": fuel.get("capacity"),
            "zones": fuel.get("zones", {}),
            "nearest": fuel.get("nearest", []),
            "clusters": fuel.get("clusters", []),
            "seen_by_camera": fuel.get("seen_by_camera", []),
            "pickup": fuel.get("pickup"),
        }

    def grab_fuel(self, count: float = 1, wait: bool = True) -> dict:
        return self._collect_action("GRAB_FUEL", count, wait)

    def collect_fuel(self, count: float = 10, wait: bool = True) -> dict:
        return self._collect_action("COLLECT_FUEL", count, wait)

    def _collect_action(self, action: str, count: float, wait: bool) -> dict:
        """The shared machinery behind grab_fuel and collect_fuel: both are just
        COLLECT_FUEL with a different fuel source on the robot side, and both need
        CollectTarget written before the trigger - the action reads it the moment
        it starts, not on some later tick."""
        available = {a.upper() for a in self.robot.available_actions()}
        if action not in available:
            return {"error": f"no such action {action!r}",
                     "available_actions": sorted(available)}
        self.robot.set_collect_target(count)
        self.robot.set_action(action)
        if wait:
            self.robot.wait_until_action_done(action, timeout=COLLECT_TIMEOUT_S)
        else:
            time.sleep(0.3)
        return self._state_with(f"{action} (target {count})")

    # -- helpers -----------------------------------------------------------

    def _state_with(self, did: str | None) -> dict:
        state = self.robot.state()
        if did:
            state["requested"] = did
        return state


def _annotate_orange_blobs(jpeg: bytes) -> bytes:
    """Circles anything orange-ish, so a human watching the tool call can see
    roughly what a fuel detector would be looking at. Purely cosmetic and
    best-effort: cv2 is optional (see the import at the top of this file), and
    any decode failure here falls back to the untouched frame rather than losing
    the tool call over a cosmetic overlay."""
    if cv2 is None:
        return jpeg
    try:
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return jpeg
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 120, 120), (25, 255, 255))  # "fuel orange"
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 40:
                continue  # noise, not a ball
            (x, y), radius = cv2.minEnclosingCircle(contour)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 140, 255), 2)
        ok, encoded = cv2.imencode(".jpg", frame)
        return encoded.tobytes() if ok else jpeg
    except Exception:
        return jpeg


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
