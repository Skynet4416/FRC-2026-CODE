#!/usr/bin/env python3
"""Mirror Choreo auto paths across the FRC field.

FRC 2026 (REBUILT) auto routines are usually drawn on one side of the field
and then a mirror image is needed for the other side. Rather than re-drawing
every path by hand in Choreo, this script reads the ``.traj`` files in the
deploy folder and writes out mirrored copies.

Coordinate system (WPILib / Choreo): the origin is the bottom-left corner of
the field as seen from the blue alliance driver station. ``x`` runs along the
field *length* (0 -> field length) and ``y`` runs along the field *width*
(0 -> field width). From the driver's point of view, +y is to the left and
-y is to the right.

Mirror axes
-----------
* ``y`` (default): reflect across the line ``y = width / 2`` that runs down the
  *length* of the field. This turns a left-side auto into the matching
  right-side auto for the same alliance ("mirror it to the right").
* ``x``: reflect across ``x = length / 2`` (the alliance flip: blue <-> red).
* ``both``: 180 degree rotation about the field center (x and y flip).

Every field that carries orientation is transformed: waypoint poses, the
solved trajectory samples (pose, velocity, acceleration and per-module
forces), and geometric constraints (KeepIn/Out circles & rectangles, PointAt).

Usage
-----
    # mirror every path in the default deploy folder to the right
    python scripts/mirror_auto_paths.py

    # mirror specific files, alliance flip instead of left/right
    python scripts/mirror_auto_paths.py --axis x path/to/My.traj

    # preview without writing anything
    python scripts/mirror_auto_paths.py --dry-run
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any, Callable

# Field dimensions for the 2026 REBUILT field, in metres. These match the
# values Choreo uses (and src/main/deploy/apriltags/.../2026-official.json).
DEFAULT_FIELD_LENGTH = 16.541
DEFAULT_FIELD_WIDTH = 8.0692

# Directory that holds the Choreo trajectory files, relative to the repo root.
DEFAULT_CHOREO_DIR = Path("src/main/deploy/choreo")

# Choreo stores the four swerve module force arrays in the order
# [front-left, front-right, back-left, back-right]. A mirror swaps modules from
# one side to the other, so we re-order the arrays to match.
MODULE_ORDER = {
    # reflect across the length -> left/right modules swap (FL<->FR, BL<->BR)
    "y": (1, 0, 3, 2),
    # reflect across the width -> front/back modules swap (FL<->BL, FR<->BR)
    "x": (2, 3, 0, 1),
    # 180 degree rotation -> diagonally opposite modules swap
    "both": (3, 2, 1, 0),
}


def wrap_angle(angle: float) -> float:
    """Wrap a heading (radians) into the range (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Mirror:
    """Encapsulates the field mirror transform for a chosen axis."""

    def __init__(self, axis: str, length: float, width: float) -> None:
        self.axis = axis
        self.length = length
        self.width = width

    # --- scalar coordinate transforms -------------------------------------
    def x(self, x: float) -> float:
        return (self.length - x) if self.axis in ("x", "both") else x

    def y(self, y: float) -> float:
        return (self.width - y) if self.axis in ("y", "both") else y

    def heading(self, h: float) -> float:
        if self.axis == "y":
            return wrap_angle(-h)
        if self.axis == "x":
            return wrap_angle(math.pi - h)
        return wrap_angle(h + math.pi)  # both

    # velocity / acceleration components flip the same way the position does
    def flip_vx(self, v: float) -> float:
        return -v if self.axis in ("x", "both") else v

    def flip_vy(self, v: float) -> float:
        return -v if self.axis in ("y", "both") else v

    def flip_omega(self, w: float) -> float:
        # angular rate keeps its sign only for a pure 180 rotation
        return w if self.axis == "both" else -w

    # --- per-module force arrays ------------------------------------------
    def module_forces(self, fx: list, fy: list) -> tuple[list, list]:
        order = MODULE_ORDER[self.axis]
        if len(fx) == len(fy) == len(order):
            fx = [fx[i] for i in order]
            fy = [fy[i] for i in order]
        new_fx = [self.flip_vx(v) for v in fx]
        new_fy = [self.flip_vy(v) for v in fy]
        return new_fx, new_fy


# --- Choreo expression helpers -------------------------------------------
_NUM_RE = re.compile(r"^\s*[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?\s*")


def _unit_of(exp: str) -> str:
    """Return the trailing unit of a Choreo expression like '7.44 m' -> 'm'."""
    m = _NUM_RE.match(exp)
    if m is None:
        return ""
    return exp[m.end():].strip()


def mirror_expr(obj: Any, transform: Callable[[float], float]) -> Any:
    """Mirror a Choreo ``{"exp": "...", "val": ...}`` expression object.

    The value is transformed numerically and the expression string is rewritten
    as a plain number carrying the original unit (any variable reference is
    resolved to its number in the process, which keeps the file valid).
    """
    if not (isinstance(obj, dict) and "val" in obj and "exp" in obj):
        return obj
    new_val = transform(obj["val"])
    unit = _unit_of(str(obj["exp"]))
    new_exp = f"{new_val} {unit}".strip()
    return {"exp": new_exp, "val": new_val}


def mirror_waypoint(wp: dict, m: Mirror) -> dict:
    """Mirror a waypoint whose x/y/heading may be floats or expression objects."""
    wp = dict(wp)
    for key, fn in (("x", m.x), ("y", m.y), ("heading", m.heading)):
        if key not in wp:
            continue
        value = wp[key]
        if isinstance(value, dict):
            wp[key] = mirror_expr(value, fn)
        else:
            wp[key] = fn(value)
    return wp


def mirror_constraint(constraint: dict, m: Mirror) -> dict:
    """Mirror the geometry of a constraint's props, where it has any."""
    constraint = json.loads(json.dumps(constraint))  # deep copy
    data = constraint.get("data")
    if not isinstance(data, dict):
        return constraint
    ctype = data.get("type")
    props = data.get("props")
    if not isinstance(props, dict):
        return constraint

    if ctype in ("KeepInCircle", "KeepOutCircle", "PointAt"):
        if "x" in props:
            props["x"] = mirror_expr(props["x"], m.x)
        if "y" in props:
            props["y"] = mirror_expr(props["y"], m.y)
    elif ctype == "KeepInRectangle":
        # (x, y) is the bottom-left corner with size (w, h). Mirroring the
        # rectangle moves the corner so the box still covers the mirrored area.
        w = props.get("w", {}).get("val", 0.0) if isinstance(props.get("w"), dict) else 0.0
        h = props.get("h", {}).get("val", 0.0) if isinstance(props.get("h"), dict) else 0.0
        if "x" in props and m.axis in ("x", "both"):
            props["x"] = mirror_expr(props["x"], lambda x: m.length - x - w)
        if "y" in props and m.axis in ("y", "both"):
            props["y"] = mirror_expr(props["y"], lambda y: m.width - y - h)
    return constraint


def mirror_sample(sample: dict, m: Mirror) -> dict:
    """Mirror a single solved trajectory sample."""
    s = dict(sample)
    if "x" in s:
        s["x"] = m.x(s["x"])
    if "y" in s:
        s["y"] = m.y(s["y"])
    if "heading" in s:
        s["heading"] = m.heading(s["heading"])
    if "vx" in s:
        s["vx"] = m.flip_vx(s["vx"])
    if "vy" in s:
        s["vy"] = m.flip_vy(s["vy"])
    if "omega" in s:
        s["omega"] = m.flip_omega(s["omega"])
    if "ax" in s:
        s["ax"] = m.flip_vx(s["ax"])
    if "ay" in s:
        s["ay"] = m.flip_vy(s["ay"])
    if "alpha" in s:
        s["alpha"] = m.flip_omega(s["alpha"])
    if isinstance(s.get("fx"), list) and isinstance(s.get("fy"), list):
        s["fx"], s["fy"] = m.module_forces(s["fx"], s["fy"])
    return s


def mirror_trajectory_block(traj: dict, m: Mirror) -> dict:
    """Mirror the solved ``trajectory`` block (waypoints + samples)."""
    traj = dict(traj)
    if isinstance(traj.get("waypoints"), list):
        # these are scalar timestamps of each waypoint along the path
        traj["waypoints"] = list(traj["waypoints"])
    if isinstance(traj.get("samples"), list):
        traj["samples"] = [mirror_sample(s, m) for s in traj["samples"]]
    return traj


def mirror_document(doc: dict, m: Mirror, new_name: str) -> dict:
    """Return a mirrored copy of a full Choreo ``.traj`` document."""
    out = json.loads(json.dumps(doc))  # deep copy
    out["name"] = new_name

    for section in ("snapshot", "params"):
        block = out.get(section)
        if isinstance(block, dict):
            if isinstance(block.get("waypoints"), list):
                block["waypoints"] = [mirror_waypoint(w, m) for w in block["waypoints"]]
            if isinstance(block.get("constraints"), list):
                block["constraints"] = [mirror_constraint(c, m) for c in block["constraints"]]

    if isinstance(out.get("trajectory"), dict):
        out["trajectory"] = mirror_trajectory_block(out["trajectory"], m)

    return out


def resolve_targets(paths: list[str], choreo_dir: Path) -> list[Path]:
    """Expand the user-provided paths into a list of .traj files."""
    if not paths:
        return sorted(choreo_dir.glob("*.traj"))
    files: list[Path] = []
    for p in paths:
        path = Path(p)
        if path.is_dir():
            files.extend(sorted(path.glob("*.traj")))
        else:
            files.append(path)
    return files


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mirror Choreo auto paths across the FRC 2026 field.",
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="Specific .traj files or directories. Defaults to "
        f"{DEFAULT_CHOREO_DIR}.",
    )
    parser.add_argument(
        "--axis",
        choices=["y", "x", "both"],
        default="y",
        help="Mirror axis: 'y' = left/right across the field length (default, "
        "'to the right'); 'x' = alliance flip; 'both' = 180 rotation.",
    )
    parser.add_argument(
        "--length",
        type=float,
        default=DEFAULT_FIELD_LENGTH,
        help=f"Field length in metres (default {DEFAULT_FIELD_LENGTH}).",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=DEFAULT_FIELD_WIDTH,
        help=f"Field width in metres (default {DEFAULT_FIELD_WIDTH}).",
    )
    parser.add_argument(
        "--suffix",
        default="_Mirrored",
        help="Suffix added to the mirrored file/path name (default '_Mirrored').",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite an existing mirrored file instead of skipping it.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report what would be written without creating any files.",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    choreo_dir = (repo_root / DEFAULT_CHOREO_DIR)
    targets = resolve_targets(args.paths, choreo_dir)

    if not targets:
        print("No .traj files found to mirror.")
        return 1

    def display(path: Path) -> str:
        path = path.resolve()
        try:
            return str(path.relative_to(repo_root))
        except ValueError:
            return str(path)

    mirror = Mirror(args.axis, args.length, args.width)
    written = 0
    for src in targets:
        if not src.exists():
            print(f"skip (missing): {src}")
            continue
        # avoid mirroring an already-mirrored file back on itself
        if src.stem.endswith(args.suffix):
            print(f"skip (already mirrored): {src.name}")
            continue

        new_name = f"{src.stem}{args.suffix}"
        dst = src.with_name(f"{new_name}{src.suffix}")

        if dst.exists() and not args.overwrite and not args.dry_run:
            print(f"skip (exists, use --overwrite): {dst.name}")
            continue

        doc = json.loads(src.read_text())
        mirrored = mirror_document(doc, mirror, new_name)

        if args.dry_run:
            print(f"would write: {display(dst)}")
            continue

        # Choreo writes compact JSON with a trailing newline; match it closely.
        dst.write_text(json.dumps(mirrored, indent=1) + "\n")
        print(f"wrote: {display(dst)}")
        written += 1

    if not args.dry_run:
        print(f"\nDone. {written} mirrored path(s) written (axis='{args.axis}').")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
