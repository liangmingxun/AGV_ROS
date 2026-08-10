#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

from multi_agv_analysis.calibration import (
    calibration_candidate,
    solve_tag_to_target,
    solve_world_alignment,
)
from multi_agv_analysis.io_utils import (
    atomic_dump_yaml,
    load_yaml,
    read_csv,
    sha256_file,
    write_csv,
)


POLICY_TEMPLATE = {
    "schema_version": 1,
    # The load interface stays unresolved until its tag height and rigid
    # transform are physically measured.
    "required_entities": ["agv1", "agv2", "agv3"],
    "minimum_samples_per_entity": None,
    "minimum_world_samples": None,
    "maximum_translation_residual": None,
    "maximum_yaw_residual": None,
    "maximum_world_point_residual": None,
    "notes": "Fill from the signed physical calibration protocol.",
}


def initialise(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    atomic_dump_yaml(root / "calibration_policy.yaml", POLICY_TEMPLATE)
    write_csv(root / "tag_to_target_samples.csv", [], [
        "entity", "tag_x", "tag_y", "tag_yaw",
        "target_x", "target_y", "target_yaw", "stamp", "notes"])
    write_csv(root / "world_alignment_samples.csv", [], [
        "source_x", "source_y", "world_x", "world_y", "stamp", "notes"])
    print("CALIBRATION WORKSPACE CREATED: {}".format(root))


def solve(root, output, dynamic_ground_reference=False):
    root = Path(root)
    policy_path = root / "calibration_policy.yaml"
    tag_path = root / "tag_to_target_samples.csv"
    world_path = root / "world_alignment_samples.csv"
    policy = load_yaml(policy_path)
    tag = solve_tag_to_target(read_csv(tag_path), policy)
    if dynamic_ground_reference:
        world = {
            "passed": True,
            "mode": "ground_aruco_dynamic",
            "surveyed_world_coordinates_required": False,
            "physical_accuracy_authorized": False,
            "note": (
                "world_T_camera is solved online from fixed ground markers; "
                "runtime epoch/residual evidence must still be archived"),
        }
    else:
        world = solve_world_alignment(read_csv(world_path), policy)
    result = calibration_candidate(tag, world, {
        policy_path.name: sha256_file(policy_path),
        tag_path.name: sha256_file(tag_path),
        world_path.name: sha256_file(world_path),
    })
    atomic_dump_yaml(output, result)
    print("CALIBRATION CANDIDATE: {} ({})".format(
        output, result["status"]))
    return 0 if result["numeric_policy_passed"] else 4


def append_row(path, row, fields):
    rows = read_csv(path)
    rows.append(row)
    write_csv(path, rows, fields)


TAG_FIELDS = [
    "entity", "tag_x", "tag_y", "tag_yaw",
    "target_x", "target_y", "target_yaw", "stamp", "notes"]
WORLD_FIELDS = [
    "source_x", "source_y", "world_x", "world_y", "stamp", "notes"]


def main():
    parser = argparse.ArgumentParser(
        description="Create or solve a fail-closed planar calibration bundle.")
    sub = parser.add_subparsers(dest="command", required=True)
    create = sub.add_parser("init")
    create.add_argument("directory")
    run = sub.add_parser("solve")
    run.add_argument("directory")
    run.add_argument("output")
    run.add_argument(
        "--dynamic-ground-reference", action="store_true",
        help=(
            "use the current four-fixed-marker online world frame instead "
            "of requiring surveyed source/world point pairs"))
    tag = sub.add_parser("add-tag-pair")
    tag.add_argument("directory")
    tag.add_argument("entity", choices=["agv1", "agv2", "agv3", "load"])
    tag.add_argument("tag_x", type=float)
    tag.add_argument("tag_y", type=float)
    tag.add_argument("tag_yaw", type=float)
    tag.add_argument("target_x", type=float)
    tag.add_argument("target_y", type=float)
    tag.add_argument("target_yaw", type=float)
    tag.add_argument("--stamp", default="")
    tag.add_argument("--notes", default="")
    world = sub.add_parser("add-world-pair")
    world.add_argument("directory")
    world.add_argument("source_x", type=float)
    world.add_argument("source_y", type=float)
    world.add_argument("world_x", type=float)
    world.add_argument("world_y", type=float)
    world.add_argument("--stamp", default="")
    world.add_argument("--notes", default="")
    args = parser.parse_args()
    try:
        if args.command == "init":
            initialise(args.directory)
            return 0
        if args.command == "add-tag-pair":
            append_row(
                Path(args.directory) / "tag_to_target_samples.csv",
                {field: getattr(args, field) for field in TAG_FIELDS},
                TAG_FIELDS)
            print("TAG PAIR APPENDED")
            return 0
        if args.command == "add-world-pair":
            append_row(
                Path(args.directory) / "world_alignment_samples.csv",
                {field: getattr(args, field) for field in WORLD_FIELDS},
                WORLD_FIELDS)
            print("WORLD PAIR APPENDED")
            return 0
        return solve(
            args.directory, args.output,
            bool(args.dynamic_ground_reference))
    except Exception as error:
        print("CALIBRATION REFUSED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())
