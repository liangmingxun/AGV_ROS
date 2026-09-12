#!/usr/bin/env python3
"""Regenerate review-only wheel-scale candidates from one completed run."""

import argparse
import csv
import importlib.util
import json
import math
import os
import pathlib
import sys


IMPLEMENTATION_PATH = pathlib.Path(__file__).resolve().with_name(
    "wheel_speed_scale_calibration.py"
)
IMPLEMENTATION_SPEC = importlib.util.spec_from_file_location(
    "wheel_speed_scale_calibration_implementation", str(IMPLEMENTATION_PATH)
)
IMPLEMENTATION = importlib.util.module_from_spec(IMPLEMENTATION_SPEC)
IMPLEMENTATION_SPEC.loader.exec_module(IMPLEMENTATION)
annotate_segment_residuals = IMPLEMENTATION.annotate_segment_residuals
fit_candidates = IMPLEMENTATION.fit_candidates
generate_fit_plots = IMPLEMENTATION.generate_fit_plots


TEXT_FIELDS = {"label", "direction"}
INTEGER_FIELDS = {"repetition", "feedback_sample_count", "camera_sample_count"}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Re-fit a completed wheel speed scale calibration without motion."
    )
    parser.add_argument("run_dir")
    return parser.parse_args(argv)


def read_metadata(path):
    result = {}
    if not os.path.isfile(path):
        return result
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            key, separator, value = line.rstrip("\n").partition("=")
            if separator:
                result[key] = value
    return result


def read_segments(path):
    result = []
    with open(path, newline="", encoding="utf-8") as stream:
        for source in csv.DictReader(stream):
            row = {}
            for key, value in source.items():
                if key in TEXT_FIELDS:
                    row[key] = value
                elif key in INTEGER_FIELDS:
                    row[key] = int(float(value))
                else:
                    row[key] = float(value)
            result.append(row)
    if not result:
        raise ValueError("speed_point_metrics.csv contains no segments")
    return result


def write_outputs(run_dir, segments, fit, metadata):
    metrics_path = os.path.join(run_dir, "speed_point_metrics.csv")
    with open(metrics_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(segments[0]))
        writer.writeheader()
        writer.writerows(segments)

    robot = metadata.get("robot_id", os.path.basename(run_dir).split("_", 1)[0])
    report = {
        "schema_version": 1,
        "status": fit["status"],
        "robot_id": robot,
        "operator": metadata.get("operator", "unknown"),
        "source": "offline speed_point_metrics.csv refit",
        "speed_points_mps": sorted(set(
            abs(row["physical_target_point_mps"]) for row in segments
        )),
        "repetitions": max(row["repetition"] for row in segments),
        "active_feedback_scale": {
            "left": segments[0].get("active_feedback_scale_left", math.nan),
            "right": segments[0].get("active_feedback_scale_right", math.nan),
        },
        "fit": fit,
        "segments": segments,
    }
    report_path = os.path.join(run_dir, "scale_fit_report.json")
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, ensure_ascii=False)
        stream.write("\n")

    candidate_path = os.path.join(run_dir, "scale_candidate.yaml")
    with open(candidate_path, "w", encoding="utf-8") as stream:
        stream.write("# REVIEW ONLY. Never applied automatically.\n")
        stream.write("status: {}\n".format(fit["status"]))
        stream.write("robot_id: {}\n".format(robot))
        stream.write("candidate:\n")
        for side in ("left", "right"):
            result = fit["wheels"][side]
            stream.write(
                "  wheel_feedback_scale_{}: {:.9f}\n".format(
                    side, result["wheel_feedback_scale_candidate"]
                )
            )
            stream.write(
                "  physical_to_firmware_command_scale_{}: {:.9f}\n".format(
                    side, result["physical_to_firmware_command_scale_candidate"]
                )
            )
        stream.write("automatically_applied: false\n")
        if fit["quality_reasons"]:
            stream.write("quality_reasons:\n")
            for reason in fit["quality_reasons"]:
                stream.write("  - {}\n".format(reason))
        else:
            stream.write("quality_reasons: []\n")
    generate_fit_plots(segments, fit, run_dir)
    return metrics_path, report_path, candidate_path


def main(argv=None):
    args = parse_args(argv)
    run_dir = os.path.abspath(args.run_dir)
    segments = read_segments(os.path.join(run_dir, "speed_point_metrics.csv"))
    repetitions = max(row["repetition"] for row in segments)
    fit = fit_candidates(segments, repetitions)
    annotate_segment_residuals(segments, fit)
    paths = write_outputs(
        run_dir, segments, fit,
        read_metadata(os.path.join(run_dir, "metadata.txt")),
    )
    print("OFFLINE WHEEL SCALE ANALYSIS COMPLETE: {}".format(fit["status"]))
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
