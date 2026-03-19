#!/usr/bin/env python3
"""
Mirrors a PathPlanner auto and its referenced paths from one side of the
field to the other (left <-> right) within the same alliance.

This mirrors all coordinates across the field's lengthwise centerline
(Y = field_width / 2), which is the transformation PathPlanner does NOT
handle automatically (it only handles blue <-> red alliance flipping).

Mirror math:
  new_x = old_x                       (distance from alliance wall unchanged)
  new_y = field_width - old_y          (mirrored across centerline)
  new_rotation = -rotation             (heading reflected)

Output files swap Left<->Right in their names and are prefixed with
"MIRRORED " so they're easy to identify as generated.

Usage:
  python scripts/mirror_autos.py "LeftStart_SimpleShot"        # mirror auto + its paths
  python scripts/mirror_autos.py --dry-run "RightStart_Sprint"  # preview without writing

No external dependencies required - uses only the Python standard library.
"""

import argparse
import copy
import json
import os
import sys
from pathlib import Path


# --- Field dimensions (meters) for 2026 REBUILT ---
# Read from navgrid.json if available, otherwise use these defaults.
DEFAULT_FIELD_LENGTH = 16.54
DEFAULT_FIELD_WIDTH = 8.07


def find_deploy_dir():
    """Locate the pathplanner deploy directory relative to the script or cwd."""
    candidates = [
        Path(__file__).resolve().parent.parent / "src" / "main" / "deploy" / "pathplanner",
        Path.cwd() / "src" / "main" / "deploy" / "pathplanner",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    print("ERROR: Could not find src/main/deploy/pathplanner directory.")
    print("Run this script from the project root or the scripts/ folder.")
    sys.exit(1)


def read_field_dimensions(deploy_dir):
    """Read field dimensions from navgrid.json, falling back to defaults."""
    navgrid_path = deploy_dir / "navgrid.json"
    if navgrid_path.exists():
        with open(navgrid_path, "r") as f:
            navgrid = json.load(f)
        field_size = navgrid.get("field_size", {})
        length = field_size.get("x", DEFAULT_FIELD_LENGTH)
        width = field_size.get("y", DEFAULT_FIELD_WIDTH)
        return length, width
    return DEFAULT_FIELD_LENGTH, DEFAULT_FIELD_WIDTH


MIRRORED_PREFIX = "MIRRORED "


# --- Name mirroring ---

def swap_side_in_name(name):
    """Swap Left<->Right tags in a name."""
    # Use placeholder tokens to avoid double-swapping
    name = name.replace("Left", "<<LEFT>>").replace("Right", "<<RIGHT>>")
    name = name.replace("left", "<<left>>").replace("right", "<<right>>")
    name = name.replace("[L]", "<<L>>").replace("[R]", "<<R>>")
    name = name.replace("<<LEFT>>", "Right").replace("<<RIGHT>>", "Left")
    name = name.replace("<<left>>", "right").replace("<<right>>", "left")
    name = name.replace("<<L>>", "[R]").replace("<<R>>", "[L]")
    return name


def make_mirrored_name(name):
    """Add 'MIRRORED ' prefix and swap Left/Right tags."""
    if name.startswith(MIRRORED_PREFIX):
        inner = name[len(MIRRORED_PREFIX):]
        return MIRRORED_PREFIX + swap_side_in_name(inner)
    return MIRRORED_PREFIX + swap_side_in_name(name)


# --- Coordinate mirroring ---

def wrap_rotation(degrees):
    """Wrap a rotation in degrees to the (-180, 180] range."""
    while degrees > 180:
        degrees -= 360
    while degrees <= -180:
        degrees += 360
    return round(degrees, 4)


def mirror_point_y(x, y, field_width):
    """Mirror a point across the field's lengthwise centerline (Y axis only)."""
    return round(x, 6), round(field_width - y, 6)


def mirror_rotation(degrees):
    """Mirror heading across the centerline (negate the angle)."""
    return wrap_rotation(-degrees)


# --- Path mirroring ---

def mirror_path_data(path_data, field_width):
    """Mirror all coordinates and rotations in a PathPlanner .path file."""
    data = copy.deepcopy(path_data)

    for wp in data.get("waypoints", []):
        for key in ("anchor", "prevControl", "nextControl"):
            point = wp.get(key)
            if point is not None:
                point["x"], point["y"] = mirror_point_y(
                    point["x"], point["y"], field_width
                )

    for rt in data.get("rotationTargets", []):
        if "rotationDegrees" in rt:
            rt["rotationDegrees"] = mirror_rotation(rt["rotationDegrees"])

    for ptz in data.get("pointTowardsZones", []):
        pos = ptz.get("fieldPosition")
        if pos is not None:
            pos["x"], pos["y"] = mirror_point_y(
                pos["x"], pos["y"], field_width
            )

    goal = data.get("goalEndState")
    if goal and "rotation" in goal:
        goal["rotation"] = mirror_rotation(goal["rotation"])

    start = data.get("idealStartingState")
    if start and "rotation" in start:
        start["rotation"] = mirror_rotation(start["rotation"])

    # Mirror the folder name too if present
    folder = data.get("folder")
    if folder:
        data["folder"] = swap_side_in_name(folder)

    return data


# --- Auto mirroring ---

def mirror_auto_command(command, mirror_name_fn):
    """Recursively update path name references in an auto command tree."""
    cmd = copy.deepcopy(command)

    if cmd.get("type") == "path":
        path_name = cmd.get("data", {}).get("pathName")
        if path_name:
            cmd["data"]["pathName"] = mirror_name_fn(path_name)

    nested = cmd.get("data", {}).get("commands")
    if nested:
        cmd["data"]["commands"] = [
            mirror_auto_command(c, mirror_name_fn) for c in nested
        ]

    return cmd


def mirror_auto_data(auto_data, mirror_name_fn):
    """Mirror a PathPlanner .auto file by updating all path references."""
    data = copy.deepcopy(auto_data)
    if data.get("command"):
        data["command"] = mirror_auto_command(data["command"], mirror_name_fn)
    return data


def collect_path_names_from_command(command):
    """Recursively collect all path names referenced in an auto command tree."""
    names = []
    if command.get("type") == "path":
        name = command.get("data", {}).get("pathName")
        if name:
            names.append(name)
    for child in command.get("data", {}).get("commands", []):
        names.extend(collect_path_names_from_command(child))
    return names


# --- File I/O ---

def read_json(file_path):
    with open(file_path, "r") as f:
        return json.load(f)


def write_json(file_path, data, dry_run=False):
    if dry_run:
        print(f"  [DRY RUN] Would write: {file_path}")
        return
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", newline="\n") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    print(f"  Wrote: {file_path}")


# --- Main logic ---

def mirror_single_path(paths_dir, path_name, field_width, dry_run):
    """Mirror a single .path file and write as a MIRRORED copy."""
    src = paths_dir / f"{path_name}.path"
    if not src.exists():
        print(f"  WARNING: Path file not found: {src}")
        return False

    data = read_json(src)
    mirrored = mirror_path_data(data, field_width)
    dest_name = make_mirrored_name(path_name)
    dest = paths_dir / f"{dest_name}.path"
    write_json(dest, mirrored, dry_run)
    return True


def mirror_single_auto(autos_dir, paths_dir, auto_name, field_width, dry_run):
    """Mirror a single .auto file and its referenced paths."""
    src = autos_dir / f"{auto_name}.auto"
    if not src.exists():
        print(f"  ERROR: Auto file not found: {src}")
        return False

    auto_data = read_json(src)

    # Mirror referenced paths
    path_names = collect_path_names_from_command(auto_data.get("command", {}))
    unique_paths = list(dict.fromkeys(path_names))  # deduplicate, preserve order
    if unique_paths:
        print(f"  Mirroring {len(unique_paths)} referenced path(s)...")
        for pn in unique_paths:
            mirror_single_path(paths_dir, pn, field_width, dry_run)

    # Mirror the auto
    mirrored = mirror_auto_data(auto_data, make_mirrored_name)
    dest_name = make_mirrored_name(auto_name)
    dest = autos_dir / f"{dest_name}.auto"
    write_json(dest, mirrored, dry_run)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Mirror a PathPlanner auto and its paths from left to right (or vice versa) within the same alliance."
    )
    parser.add_argument(
        "auto",
        help="Name of the auto to mirror (without .auto extension). Also mirrors its referenced paths.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview what would be written without actually writing files.",
    )
    args = parser.parse_args()

    deploy_dir = find_deploy_dir()
    _, field_width = read_field_dimensions(deploy_dir)
    paths_dir = deploy_dir / "paths"
    autos_dir = deploy_dir / "autos"

    print(f"PathPlanner dir: {deploy_dir}")
    print(f"Field width: {field_width}m (mirroring across Y = {field_width / 2:.3f}m)")
    print()

    print(f"Mirroring auto: {args.auto}")
    mirror_single_auto(autos_dir, paths_dir, args.auto, field_width, args.dry_run)

    print()
    print("Done.")


if __name__ == "__main__":
    main()
