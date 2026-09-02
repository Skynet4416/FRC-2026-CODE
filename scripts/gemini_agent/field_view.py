"""Draws the top-down field view the model looks at.

This is the agent's main sense. A camera frame shows a few metres of carpet; the
field view shows the whole field in the same coordinates the tools take, with a
one-metre grid, the obstacles PathPlanner will not drive through, where the
robot is and which way it faces, what the camera can see, and the path it is
following. Ask a spatial question about the map and the answer is on it.

The obstacles come from the robot's own navgrid.json, so the picture and the
pathfinder agree about what is solid.
"""

from __future__ import annotations

import io
import json
import os
from typing import Any

import matplotlib

matplotlib.use("Agg")  # headless: no display anywhere near this
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

NAVGRID = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "src", "main", "deploy", "pathplanner", "navgrid.json",
)

ROBOT_SIZE_M = 0.9


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

    figure, axes = plt.subplots(figsize=(9.0, 9.0 * size[1] / size[0]))
    axes.set_facecolor("#1b1b1b")
    figure.patch.set_facecolor("#1b1b1b")

    # Obstacles: the cells PathPlanner refuses to route through.
    if grid:
        cell = grid["nodeSizeMeters"]
        for row_index, row in enumerate(grid["grid"]):
            for column_index, blocked in enumerate(row):
                if blocked:
                    axes.add_patch(
                        Rectangle(
                            (column_index * cell, row_index * cell), cell, cell,
                            facecolor="#4a4a52", edgecolor="none",
                        )
                    )

    # Landmarks, so the names in the prompt have somewhere to point.
    for name, pose in landmarks.items():
        axes.plot(pose[0], pose[1], marker="+", color="#8a8aa0", markersize=8)
        axes.annotate(
            name, (pose[0], pose[1]), color="#8a8aa0", fontsize=6.5,
            xytext=(0, 5), textcoords="offset points", ha="center",
        )

    # What the game-piece camera can see right now.
    pieces = robot.detected_game_pieces()
    if pieces:
        axes.scatter(
            [p[0] for p in pieces], [p[1] for p in pieces],
            s=26, color="#ff9d2e", edgecolors="#7a4400", linewidths=0.5,
            zorder=4, label=f"fuel seen by the camera ({len(pieces)})",
        )

    # The path being followed.
    path = robot.trajectory()
    if path and state["navigating"]:
        axes.plot([p[0] for p in path], [p[1] for p in path],
                  color="#54b7ff", linewidth=1.4, alpha=0.9, zorder=3, label="active path")

    # Where the robot was told to go.
    target = state.get("active_target")
    if target:
        axes.plot(target["x"], target["y"], marker="x", color="#3ddc84",
                  markersize=11, markeredgewidth=2.2, zorder=5, label="requested pose")

    # The robot itself, with its heading.
    pose = state.get("pose")
    if pose:
        _draw_robot(axes, pose["x"], pose["y"], pose["heading_deg"], state["rotation_locked"])

    axes.set_xlim(0, size[0])
    axes.set_ylim(0, size[1])
    axes.set_aspect("equal")
    axes.set_xticks(range(0, int(size[0]) + 1))
    axes.set_yticks(range(0, int(size[1]) + 1))
    axes.grid(color="#3a3a44", linewidth=0.5)
    axes.tick_params(colors="#b9b9c8", labelsize=7)
    for spine in axes.spines.values():
        spine.set_color("#5a5a68")
    axes.set_xlabel("X (m) - blue wall at 0", color="#b9b9c8", fontsize=8)
    axes.set_ylabel("Y (m) - scoring table at 0", color="#b9b9c8", fontsize=8)
    axes.set_title(_title(state), color="#e8e8f0", fontsize=9)

    handles, _labels = axes.get_legend_handles_labels()
    if handles:
        legend = axes.legend(loc="upper right", fontsize=6.5, facecolor="#26262e",
                             edgecolor="#5a5a68", labelcolor="#d8d8e4")
        legend.get_frame().set_alpha(0.9)

    buffer = io.BytesIO()
    figure.savefig(buffer, format="png", dpi=dpi, bbox_inches="tight", facecolor=figure.get_facecolor())
    plt.close(figure)
    return buffer.getvalue()


def _draw_robot(axes, x: float, y: float, heading_deg: float, rotation_locked: bool) -> None:
    import math

    colour = "#ff5555" if rotation_locked else "#54b7ff"
    half = ROBOT_SIZE_M / 2.0
    axes.add_patch(
        Rectangle((x - half, y - half), ROBOT_SIZE_M, ROBOT_SIZE_M, angle=heading_deg,
                  rotation_point="center", facecolor=colour, alpha=0.55,
                  edgecolor=colour, linewidth=1.4, zorder=6, label="robot")
    )
    heading = math.radians(heading_deg)
    axes.arrow(x, y, math.cos(heading) * 0.8, math.sin(heading) * 0.8,
               width=0.05, color=colour, zorder=7, length_includes_head=True)


def _title(state: dict) -> str:
    pose = state.get("pose") or {}
    where = (
        f"robot ({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}) "
        f"facing {pose.get('heading_deg', 0):.0f} deg"
    )
    zone = "in the shooting zone" if state["in_shooting_zone"] else "outside the shooting zone"
    return f"{where} - {zone}\n{state['status']}"
