"""Draws the top-down field view the model looks at.

This is the agent's main sense. A camera frame shows a few metres of carpet; the
field view shows the whole field in the same coordinates the tools take, with a
one-metre grid, the obstacles PathPlanner will not drive through, the field's
zones, every fuel piece on the ground (dim) with what the camera can see
highlighted, where the robot is and which way it faces, and the path it is
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

# A match can have hundreds of fuel on the field at once. Drawing every one of
# them as a full marker turns the map into a smear of dots that hides the robot,
# the path and the camera-seen pieces underneath it - so the ground-truth layer
# is capped and drawn as small, faint dots, evenly sampled rather than just the
# first N so a sparse corner of the field is not left looking empty.
MAX_FUEL_DOTS = 220

# The rectangles the intake folds for - drawn in a different colour from the
# rest of the zones so it is visually obvious why the intake goes up there.
FOLD_HAZARD_ZONES = {"left_trench", "right_trench", "left_bump", "right_bump"}

# Depots are a fuel source, not a hazard or a plain scoring zone, and get their
# own look: a filled tint rather than another dashed outline. They were missing
# from the map entirely until a depot went unmarked and the model never went
# near it - see REGISTRATION.md.
DEPOT_ZONES = {"our_depot", "opponent_depot"}


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


def _subsample(points: list, cap: int) -> list:
    """Evenly thins `points` down to at most `cap` entries, so a dense cluster in
    one corner of the field does not use up the whole budget and leave the rest
    of the field looking emptier than it is."""
    if len(points) <= cap:
        return points
    step = len(points) / cap
    return [points[int(i * step)] for i in range(cap)]


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

    # Field zones - alliance zones, depots, trenches, bumps, hub. The trench and
    # bump rectangles are also where the intake folds, and the depots are a fuel
    # source, so both get their own look rather than blending into a generic box.
    for name, box in robot.zones().items():
        try:
            x0, y0 = float(box["x_min"]), float(box["y_min"])
            width, height = float(box["x_max"]) - x0, float(box["y_max"]) - y0
        except (KeyError, TypeError, ValueError):
            continue  # a malformed zone should not cost the rest of the map
        if name in DEPOT_ZONES:
            # Filled, not outlined: a depot is a pile of fuel sitting against the
            # wall, and a tint reads as "fuel is here" the way a dashed box does
            # not. Ground-truth fuel dots (drawn after this) show through on top.
            axes.add_patch(
                Rectangle(
                    (x0, y0), width, height, facecolor="#d4a017", alpha=0.32,
                    edgecolor="#8a6100", linewidth=1.2, zorder=1.5,
                )
            )
            axes.annotate(
                name.replace("_", " ") + "\n(fuel depot)", (x0 + width / 2, y0 + height / 2),
                color="#6b5000", fontsize=6.5, fontweight="bold", alpha=0.95,
                ha="center", va="center", zorder=1.55,
            )
            continue
        hazard = name in FOLD_HAZARD_ZONES
        color = "#e0362c" if hazard else "#5b8def"
        axes.add_patch(
            Rectangle(
                (x0, y0), width, height, facecolor="none", edgecolor=color,
                linewidth=1.4 if hazard else 1.0, linestyle="-" if hazard else "--",
                alpha=0.7 if hazard else 0.5, zorder=1.5,
            )
        )
        label = name.replace("_", " ") + (" (folds intake)" if hazard else "")
        axes.annotate(
            label, (x0 + width / 2, y0 + height / 2), color=color, fontsize=6,
            alpha=0.85, ha="center", va="center", zorder=1.5,
        )

    # Every fuel piece on the field, ground truth - dim, small, and subsampled so
    # a few hundred balls do not drown out everything else on the map. Falls back
    # to the AdvantageKit pose-struct path for a robot that does not yet publish
    # the flat FuelPositions array.
    all_fuel = robot.field_fuel() or robot.field_game_pieces()
    if all_fuel:
        shown = _subsample(all_fuel, MAX_FUEL_DOTS)
        axes.scatter(
            [p[0] for p in shown], [p[1] for p in shown],
            s=4, color="#c8a028", alpha=0.45, edgecolors="none",
            zorder=1.6, label=f"fuel on the field ({len(all_fuel)})",
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


class _ReferenceField:
    """A stand-in for RobotConnection with no live robot behind it, so the map
    can be rendered and its registration checked from nothing but this repo.

    The numbers are the field geometry measured against field26.png itself in
    REGISTRATION.md, not anything render() computes - this is the answer key a
    self-check compares the real code's output against, and what --save draws
    when there is no robot to ask.
    """

    SIZE = [16.541, 8.069]
    HUB_CENTRE = (4.619, 4.035)
    ZONES: dict[str, dict[str, Any]] = {
        "our_alliance_zone": {"x_min": 0.000, "y_min": 0.0, "x_max": 4.022,
                               "y_max": 8.069, "what": "score here"},
        "neutral_zone": {"x_min": 5.223, "y_min": 0.0, "x_max": 11.319,
                          "y_max": 8.069, "what": "shared fuel"},
        "opponent_alliance_zone": {"x_min": 12.519, "y_min": 0.0, "x_max": 16.541,
                                    "y_max": 8.069, "what": ""},
        "hub": {"x_min": 4.022, "y_min": 3.438, "x_max": 5.216,
                "y_max": 4.632, "what": "score fuel here"},
        "opponent_hub": {"x_min": 11.312, "y_min": 3.438, "x_max": 12.506,
                          "y_max": 4.632, "what": ""},
        "left_trench": {"x_min": 0.0, "y_min": 6.790, "x_max": 16.541,
                         "y_max": 8.069, "what": "low crossing"},
        "right_trench": {"x_min": 0.0, "y_min": 0.0, "x_max": 16.541,
                          "y_max": 1.279, "what": "low crossing"},
        "left_bump": {"x_min": 4.619 - 0.927, "y_min": 4.632, "x_max": 4.619 + 0.927,
                       "y_max": 6.486, "what": "bump crossing"},
        "right_bump": {"x_min": 4.619 - 0.927, "y_min": 1.583, "x_max": 4.619 + 0.927,
                        "y_max": 3.438, "what": "bump crossing"},
        "our_depot": {"x_min": 0.000, "y_min": 5.430, "x_max": 0.686,
                       "y_max": 6.497, "what": "preloaded fuel"},
        "opponent_depot": {"x_min": 15.855, "y_min": 1.572, "x_max": 16.541,
                            "y_max": 2.639, "what": "preloaded fuel"},
    }

    def state(self) -> dict[str, Any]:
        return {
            "navigating": False, "active_target": None, "pose": None,
            "rotation_locked": False, "in_shooting_zone": False,
            "status": "reference geometry - no live robot",
        }

    def field_size(self) -> list[float]:
        return list(self.SIZE)

    def zones(self) -> dict[str, dict[str, Any]]:
        return {name: dict(box) for name, box in self.ZONES.items()}

    def field_fuel(self) -> list[tuple[float, float]]:
        return []

    def field_game_pieces(self) -> list[tuple[float, float]]:
        return []

    def detected_game_pieces(self) -> list[tuple[float, float]]:
        return []

    def trajectory(self) -> list[tuple[float, float]]:
        return []


def check_registration(check, field_size: list[float] | None = None) -> None:
    """The self-check that would have caught the depot bug: recomputes the same
    field->pixel mapping render() uses and checks it against known geometry,
    rather than trusting that the image, FIELD_IMAGE_PPM/MARGIN_M, and the
    field's own numbers stay in agreement forever. Takes a `check(label, ok,
    detail="")` callback in the same shape selftest.py's checks use, and needs
    no live robot - only the repo's own field26.png and the reference geometry
    measured in REGISTRATION.md.
    """
    size = field_size or list(_ReferenceField.SIZE)
    image = _load_field_image()
    check("field26.png loads", image is not None)
    if image is None:
        return

    height, width = image.shape[0], image.shape[1]
    pixels_per_metre = FIELD_IMAGE_PPM * width / 3508.0
    margin = FIELD_IMAGE_MARGIN_M

    # The span the image implies once FIELD_IMAGE_PPM is applied has to equal
    # the declared field size plus a margin on each side - this is the whole
    # image/field/constants agreement render()'s extent depends on.
    span_x, span_y = width / pixels_per_metre, height / pixels_per_metre
    check(
        "image extent matches the declared field size to the millimetre",
        abs(span_x - (size[0] + 2 * margin)) < 2e-3
        and abs(span_y - (size[1] + 2 * margin)) < 2e-3,
        f"image implies {span_x:.4f} x {span_y:.4f} m, expected "
        f"{size[0] + 2 * margin:.4f} x {size[1] + 2 * margin:.4f} m",
    )

    def to_pixel(x: float, y: float) -> tuple[float, float]:
        # The inverse of imshow(extent=[-margin, ...], origin="upper"): field
        # metres -> (row, col) in the source image array.
        return height - (y + margin) * pixels_per_metre, (x + margin) * pixels_per_metre

    hub_row, hub_col = to_pixel(*_ReferenceField.HUB_CENTRE)
    check(
        "the hub centre maps inside the image's own pixel bounds",
        0 <= hub_row < height and 0 <= hub_col < width,
        f"(row={hub_row:.1f}, col={hub_col:.1f}) of {height}x{width}",
    )

    hub_box = _ReferenceField.ZONES["hub"]
    row_max, col_min = to_pixel(hub_box["x_min"], hub_box["y_min"])
    row_min, col_max = to_pixel(hub_box["x_max"], hub_box["y_max"])
    check(
        "the hub centre's pixel lands inside the hub zone's own pixel box",
        row_min <= hub_row <= row_max and col_min <= hub_col <= col_max,
        f"hub centre pixel ({hub_row:.1f}, {hub_col:.1f}) vs box "
        f"rows [{row_min:.1f}, {row_max:.1f}] cols [{col_min:.1f}, {col_max:.1f}]",
    )

    for name in ("our_depot", "opponent_depot"):
        box = _ReferenceField.ZONES[name]
        corners = [to_pixel(box["x_min"], box["y_min"]), to_pixel(box["x_max"], box["y_max"])]
        rows, cols = [r for r, _c in corners], [c for _r, c in corners]
        inside = all(0 <= r < height for r in rows) and all(0 <= c < width for c in cols)
        check(
            f"{name} maps inside the image's pixel bounds",
            inside,
            f"rows {[round(r, 1) for r in rows]} cols {[round(c, 1) for c in cols]} "
            f"of {height}x{width}",
        )


def _run_registration_check() -> bool:
    """Standalone runner for `--check-overlay`: prints PASS/FAIL the way
    selftest.py does, and returns whether everything passed."""
    failures = 0

    def check(label: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        if not ok:
            failures += 1
        print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f"   [{detail}]" if not ok and detail else ""))

    check_registration(check)
    print()
    print("all overlay registration checks passed" if not failures else f"{failures} check(s) FAILED")
    return failures == 0


def _main() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Check the field map's registration, or save a render to look at by eye."
    )
    parser.add_argument(
        "--check-overlay", action="store_true",
        help="check the field image lines up with the field's own geometry, and exit",
    )
    parser.add_argument(
        "--save", metavar="PATH",
        help="render the field view and save it as a PNG - the reference geometry by "
             "default, or a live robot if --host is given - so registration can be "
             "eyeballed without running the whole agent",
    )
    parser.add_argument("--host", help="render a live robot/simulator instead of the reference scene")
    parser.add_argument("--port", type=int, default=5810)
    args = parser.parse_args()

    if not args.check_overlay and not args.save:
        parser.print_help()
        return 1

    ok = True
    if args.check_overlay:
        ok = _run_registration_check()

    if args.save:
        robot, landmarks, live = _ReferenceField(), {}, None
        if args.host:
            from nt4 import RobotConnection

            live = RobotConnection(args.host, args.port).connect()
            robot, landmarks = live, live.landmarks()
        try:
            image = render(robot, landmarks)
        finally:
            if live is not None:
                live.close()
        with open(args.save, "wb") as handle:
            handle.write(image)
        print(f"saved {args.save} ({len(image)} bytes)")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_main())
