"""Draws the top-down field view the model looks at.

This is the agent's main sense. A camera frame shows a few metres of carpet; the
field view shows the whole field in the same coordinates the tools take, with a
one-metre grid, the obstacles PathPlanner will not drive through, where the
robot is and which way it faces, what the camera can see, and the path it is
following. Ask a spatial question about the map and the answer is on it.

It is drawn over the official 2026 field render - the same background the demo
video uses - so the hubs, trenches and alliance zones the model sees are the real
ones rather than a sketch. The render is the empty field, with no game pieces on
it, so every ball in the picture is one the simulation is really publishing. On
top of that go the obstacles from the robot's own navgrid.json, so the picture
and the pathfinder also agree about what is solid.
"""

from __future__ import annotations

import io
import json
import os
from functools import lru_cache
from typing import Any

import matplotlib

matplotlib.use("Agg")  # headless: no display anywhere near this
import matplotlib.image  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NAVGRID = os.path.join(_REPO, "src", "main", "deploy", "pathplanner", "navgrid.json")
FIELD_IMAGE = os.path.join(_REPO, "scripts", "assets", "field26.png")

ROBOT_SIZE_M = 0.9

# field26.png is framed like PathPlanner's images: 200 px/m, 0.5 m margin all round.
FIELD_IMAGE_PPM = 200.0
FIELD_IMAGE_MARGIN_M = 0.5


@lru_cache(maxsize=1)
def _load_field_image(path: str = FIELD_IMAGE):
    """The field image, or None if it is missing - the map still draws."""
    try:
        return matplotlib.image.imread(path)
    except (OSError, ValueError):
        return None


def load_navgrid(path: str = NAVGRID) -> dict[str, Any] | None:
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None


def render(robot, landmarks: dict[str, list[float]], dpi: int = 110) -> bytes:
    """Renders the field as a PNG, from whatever the robot is publishing now."""
    state = robot.state()
    size = robot.field_size() or [16.54, 8.07]
    grid = load_navgrid()
    field = _load_field_image()

    figure, axes = plt.subplots(figsize=(9.0, 9.0 * size[1] / size[0]))
    figure.patch.set_facecolor("#f4f4f6")
    axes.set_facecolor("#f4f4f6")

    if field is not None:
        height, width = field.shape[0], field.shape[1]
        pixels_per_metre = FIELD_IMAGE_PPM * width / 3508.0
        margin = FIELD_IMAGE_MARGIN_M
        axes.imshow(
            field, origin="upper", zorder=0,
            extent=[-margin, width / pixels_per_metre - margin,
                    -margin, height / pixels_per_metre - margin],
        )

    # Obstacles: the cells PathPlanner refuses to route through. Faint, because
    # the field image underneath already shows what they are.
    if grid:
        cell = grid["nodeSizeMeters"]
        for row_index, row in enumerate(grid["grid"]):
            for column_index, blocked in enumerate(row):
                if blocked:
                    axes.add_patch(
                        Rectangle(
                            (column_index * cell, row_index * cell), cell, cell,
                            facecolor="#2b2b33", alpha=0.16, edgecolor="none", zorder=1,
                        )
                    )

    # Landmarks, so the names in the prompt have somewhere to point.
    for name, pose in landmarks.items():
        axes.plot(pose[0], pose[1], marker="+", color="#3c3c50", markersize=8, zorder=2)
        axes.annotate(
            name, (pose[0], pose[1]), color="#3c3c50", fontsize=6.5, zorder=2,
            xytext=(0, 5), textcoords="offset points", ha="center",
        )

    # What the game-piece camera can see right now.
    pieces = robot.detected_game_pieces()
    if pieces:
        axes.scatter(
            [p[0] for p in pieces], [p[1] for p in pieces],
            s=26, color="#ff8c00", edgecolors="#5a3000", linewidths=0.5,
            zorder=4, label=f"fuel seen by the camera ({len(pieces)})",
        )

    # The path being followed.
    path = robot.trajectory()
    if path and state["navigating"]:
        axes.plot([p[0] for p in path], [p[1] for p in path],
                  color="#0a68d8", linewidth=1.8, alpha=0.95, zorder=3, label="active path")

    # Where the robot was told to go.
    target = state.get("active_target")
    if target:
        axes.plot(target["x"], target["y"], marker="x", color="#0f9d58",
                  markersize=11, markeredgewidth=2.4, zorder=5, label="requested pose")

    # The robot itself, with its heading.
    pose = state.get("pose")
    if pose:
        _draw_robot(axes, pose["x"], pose["y"], pose["heading_deg"], state["rotation_locked"])

    axes.set_xlim(0, size[0])
    axes.set_ylim(0, size[1])
    axes.set_aspect("equal")
    axes.set_xticks(range(0, int(size[0]) + 1))
    axes.set_yticks(range(0, int(size[1]) + 1))
    axes.grid(color="#2b2b33", linewidth=0.4, alpha=0.28)
    axes.tick_params(colors="#3c3c50", labelsize=7)
    for spine in axes.spines.values():
        spine.set_color("#8a8a99")
    axes.set_xlabel("X (m) - blue wall at 0", color="#3c3c50", fontsize=8)
    axes.set_ylabel("Y (m) - scoring table at 0", color="#3c3c50", fontsize=8)
    axes.set_title(_title(state), color="#1c1c26", fontsize=9)

    handles, _labels = axes.get_legend_handles_labels()
    if handles:
        legend = axes.legend(loc="upper right", fontsize=6.5, facecolor="#ffffff",
                             edgecolor="#8a8a99", labelcolor="#1c1c26")
        legend.get_frame().set_alpha(0.92)

    buffer = io.BytesIO()
    figure.savefig(buffer, format="png", dpi=dpi, bbox_inches="tight",
                   facecolor=figure.get_facecolor())
    plt.close(figure)
    return buffer.getvalue()


def _draw_robot(axes, x: float, y: float, heading_deg: float, rotation_locked: bool) -> None:
    """The robot, drawn the way the demo video draws it: a bumper square, a red
    nose for the front, and a red ring while the shooter owns the drivetrain."""
    import math

    heading = math.radians(heading_deg)
    half = ROBOT_SIZE_M / 2.0
    corner = (
        x - half * math.cos(heading) + half * math.sin(heading),
        y - half * math.sin(heading) - half * math.cos(heading),
    )
    axes.add_patch(
        Rectangle(corner, ROBOT_SIZE_M, ROBOT_SIZE_M, angle=heading_deg,
                  facecolor="#1f6feb", alpha=0.9, edgecolor="#101018",
                  linewidth=1.2, zorder=6, label="robot")
    )
    axes.add_patch(
        Polygon(
            [
                (x + 0.62 * math.cos(heading), y + 0.62 * math.sin(heading)),
                (x + 0.30 * math.cos(heading + 2.4), y + 0.30 * math.sin(heading + 2.4)),
                (x + 0.30 * math.cos(heading - 2.4), y + 0.30 * math.sin(heading - 2.4)),
            ],
            closed=True, facecolor="#f85149", edgecolor="none", zorder=7,
        )
    )
    if rotation_locked:
        axes.add_patch(
            Circle((x, y), 0.75, facecolor="none", edgecolor="#e0362c", linewidth=2.0,
                   zorder=5, label="shooter owns the drivetrain")
        )


def _title(state: dict) -> str:
    pose = state.get("pose") or {}
    where = (
        f"robot ({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}) "
        f"facing {pose.get('heading_deg', 0):.0f} deg"
    )
    zone = "in the shooting zone" if state["in_shooting_zone"] else "outside the shooting zone"
    return f"{where} - {zone}\n{state['status']}"
