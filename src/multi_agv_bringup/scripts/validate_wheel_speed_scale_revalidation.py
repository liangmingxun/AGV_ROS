#!/usr/bin/env python3
"""Validate one completed Stage-D physical wheel-scale revalidation run."""

import argparse
import csv
import json
import math
import os
import sys


SIDES = ("left", "right")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate installed wheel command/feedback scales.")
    parser.add_argument("run_dir")
    parser.add_argument("--maximum-physical-error", type=float, default=0.010)
    parser.add_argument("--maximum-feedback-error", type=float, default=0.008)
    parser.add_argument("--maximum-firmware-target-error", type=float,
                        default=0.002)
    return parser.parse_args(argv)


def read_rows(path):
    with open(path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("speed_point_metrics.csv contains no rows")
    return rows


def finite(row, name):
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError("{} is non-finite in {}".format(name, row.get("label")))
    return value


def validate_rows(rows, physical_limit, feedback_limit, firmware_limit):
    failures = []
    results = []
    observed = set()
    for row in rows:
        target = finite(row, "physical_target_point_mps")
        repetition = int(float(row["repetition"]))
        direction = row["direction"]
        observed.add((round(abs(target), 6), direction, repetition))
        segment = {"label": row["label"], "target_mps": target}
        for side in SIDES:
            camera = finite(row, "camera_wheel_{}_physical_mps".format(side))
            actual = finite(row, "ros_wheel_{}_actual_physical_mps".format(side))
            firmware_target = finite(
                row, "firmware_wheel_{}_target_nominal_mps".format(side))
            command_scale = finite(row, "active_command_scale_{}".format(side))
            physical_error = camera - target
            feedback_error = actual - camera
            firmware_error = firmware_target - command_scale * target
            segment[side] = {
                "camera_physical_mps": camera,
                "ros_actual_physical_mps": actual,
                "firmware_target_nominal_mps": firmware_target,
                "physical_error_mps": physical_error,
                "feedback_error_mps": feedback_error,
                "firmware_target_error_mps": firmware_error,
            }
            for name, error, limit in (
                    ("physical_error", physical_error, physical_limit),
                    ("feedback_error", feedback_error, feedback_limit),
                    ("firmware_target_error", firmware_error, firmware_limit)):
                if abs(error) > limit:
                    failures.append(
                        "{} {} {}={:+.6f} exceeds {:.6f}".format(
                            row["label"], side, name, error, limit))
        results.append(segment)

    speeds = sorted({item[0] for item in observed})
    repetitions = sorted({item[2] for item in observed})
    expected = {
        (speed, direction, repetition)
        for speed in speeds
        for direction in ("forward", "reverse")
        for repetition in repetitions
    }
    missing = sorted(expected - observed)
    if missing:
        failures.append("missing speed/direction/repetition segments: {}".format(
            missing))
    return {
        "schema_version": 1,
        "status": "PASSED" if not failures else "FAILED",
        "thresholds_mps": {
            "maximum_physical_error": physical_limit,
            "maximum_feedback_error": feedback_limit,
            "maximum_firmware_target_error": firmware_limit,
        },
        "speed_points_mps": speeds,
        "repetitions": repetitions,
        "failures": failures,
        "segments": results,
    }


def main(argv=None):
    args = parse_args(argv)
    run_dir = os.path.abspath(args.run_dir)
    result = validate_rows(
        read_rows(os.path.join(run_dir, "speed_point_metrics.csv")),
        args.maximum_physical_error,
        args.maximum_feedback_error,
        args.maximum_firmware_target_error,
    )
    output = os.path.join(run_dir, "stage_d_revalidation.json")
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print("STAGE_D_REVALIDATION_STATUS={}".format(result["status"]))
    print("REPORT={}".format(output))
    for failure in result["failures"]:
        print("FAILURE={}".format(failure), file=sys.stderr)
    return 0 if result["status"] == "PASSED" else 1


if __name__ == "__main__":
    sys.exit(main())
