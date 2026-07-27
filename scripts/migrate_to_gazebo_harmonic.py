#!/usr/bin/env python3
"""Replace Gazebo Fortress / Ignition identifiers with Gazebo Harmonic names.

The script is intended for generated worlds, generated model.sdf files, private
model directories, and converter source files that may not be present in the
Git repository checkout used to prepare the migration commit.

Examples:

  # Inspect the repository without changing files.
  python3 scripts/migrate_to_gazebo_harmonic.py .

  # Apply replacements to tracked and locally generated files.
  python3 scripts/migrate_to_gazebo_harmonic.py --write .

  # Check only generated assets.
  python3 scripts/migrate_to_gazebo_harmonic.py models models_private worlds

Exit status is 1 in check mode when legacy identifiers are found.
"""

import argparse
import os
import sys
from pathlib import Path


REPLACEMENTS = (
    ("libignition-gazebo-imu-system.so", "gz-sim-imu-system"),
    ("libignition-gazebo-navsat-system.so", "gz-sim-navsat-system"),
    ("ignition-gazebo-physics-system", "gz-sim-physics-system"),
    ("ignition-gazebo-user-commands-system", "gz-sim-user-commands-system"),
    (
        "ignition-gazebo-scene-broadcaster-system",
        "gz-sim-scene-broadcaster-system",
    ),
    ("ignition-gazebo-contact-system", "gz-sim-contact-system"),
    ("ignition-gazebo-sensors-system", "gz-sim-sensors-system"),
    ("ignition-gazebo-label-system", "gz-sim-label-system"),
    ("ignition::gazebo::systems::", "gz::sim::systems::"),
    ("<depend>ros_ign_gazebo</depend>", "<depend>ros_gz_sim</depend>"),
)

TEXT_SUFFIXES = {
    ".sdf",
    ".world",
    ".urdf",
    ".xacro",
    ".xml",
    ".py",
    ".md",
    ".yaml",
    ".yml",
}

SKIP_DIRS = {
    ".git",
    ".vscode",
    "build",
    "install",
    "log",
    "__pycache__",
}

SELF_PATH = Path(__file__).resolve()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Find or replace Gazebo Fortress / Ignition identifiers."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        default=["."],
        help="Files or directories to scan. Default: current directory.",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write replacements. Without this option the script only reports matches.",
    )
    parser.add_argument(
        "--backup-suffix",
        default="",
        help="Optional suffix for a copy of each file before writing, for example .fortress.bak.",
    )
    return parser.parse_args()


def iter_text_files(paths):
    yielded = set()
    for raw_path in paths:
        path = Path(raw_path).expanduser()
        if not path.exists():
            print(f"warning: path does not exist: {path}", file=sys.stderr)
            continue

        if path.is_file():
            candidates = [path]
        else:
            candidates = []
            for root, dirs, files in os.walk(path):
                dirs[:] = [name for name in dirs if name not in SKIP_DIRS]
                root_path = Path(root)
                for name in files:
                    candidates.append(root_path / name)

        for candidate in candidates:
            if candidate.suffix.lower() not in TEXT_SUFFIXES:
                continue
            resolved = candidate.resolve()
            if resolved == SELF_PATH:
                continue
            if resolved in yielded:
                continue
            yielded.add(resolved)
            yield candidate


def migrate_text(text):
    result = text
    counts = []
    for old, new in REPLACEMENTS:
        count = result.count(old)
        if count:
            result = result.replace(old, new)
            counts.append((old, new, count))
    return result, counts


def write_backup(path, original, suffix):
    if not suffix:
        return
    backup_path = Path(str(path) + suffix)
    if backup_path.exists():
        raise RuntimeError(f"backup already exists: {backup_path}")
    backup_path.write_text(original, encoding="utf-8")


def process_file(path, write, backup_suffix):
    try:
        original = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        print(f"warning: could not read {path}: {exc}", file=sys.stderr)
        return False, 0

    migrated, counts = migrate_text(original)
    if not counts:
        return False, 0

    total = sum(count for _, _, count in counts)
    action = "updated" if write else "needs update"
    print(f"{action}: {path} ({total} replacements)")
    for old, new, count in counts:
        print(f"  {count:4d}  {old} -> {new}")

    if write:
        write_backup(path, original, backup_suffix)
        path.write_text(migrated, encoding="utf-8")

    return True, total


def main():
    args = parse_args()
    matched_files = 0
    replacement_count = 0

    for path in iter_text_files(args.paths):
        matched, count = process_file(path, args.write, args.backup_suffix)
        if matched:
            matched_files += 1
            replacement_count += count

    mode = "write" if args.write else "check"
    print(
        f"Gazebo Harmonic migration {mode}: "
        f"files={matched_files}, replacements={replacement_count}"
    )

    if not args.write and matched_files:
        print("Run again with --write to apply these replacements.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
