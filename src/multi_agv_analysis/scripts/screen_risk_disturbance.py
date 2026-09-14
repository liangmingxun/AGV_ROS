#!/usr/bin/env python3
"""Screen Exp2c disturbance exposure without pretending to replay the loop."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path


LEVELS = (0.30, 0.45, 0.60, 0.70)


def smoothstep5(value):
    value = min(1.0, max(0.0, value))
    return value ** 3 * (10.0 + value * (-15.0 + 6.0 * value))


def disturbance(path_s, velocity, elapsed, available_accel, level):
    if not (2.0 < path_s < 2.8):
        return 0.0, 0.0
    window = (smoothstep5((path_s - 2.0) / 0.15) *
              smoothstep5((2.8 - path_s) / 0.15))
    scale = -window * available_accel * level
    total = scale * (0.60 + 0.25 * min(abs(velocity) / 0.10, 1.0) +
                     0.15 * math.sin(2.0 * math.pi * 0.30 * elapsed))
    return window, total


def finite(row, name):
    try:
        value = float(row.get(name, "nan"))
        return value if math.isfinite(value) else math.nan
    except (TypeError, ValueError):
        return math.nan


def screen(run_dir):
    aligned = run_dir / "converted" / "aligned_samples.csv"
    if not aligned.is_file():
        raise RuntimeError("missing converted/aligned_samples.csv")
    with aligned.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    usable = []
    for row in rows:
        s = finite(row, "agv2_s_tracking_actual")
        v = finite(row, "agv2_s_dot_actual")
        stamp = finite(row, "stamp")
        if math.isfinite(s) and math.isfinite(v) and math.isfinite(stamp):
            usable.append((stamp, s, v))
    if not usable:
        raise RuntimeError("no finite Robot2 path-state samples")
    origin = usable[0][0]
    candidates = {}
    for level in LEVELS:
        values = [disturbance(s, v, stamp - origin, 1.0, level)
                  for stamp, s, v in usable]
        active = [total for window, total in values if window > 1e-9]
        candidates["0p{:02d}".format(round(level * 100))] = {
            "active_samples": len(active),
            "active_duration_estimate_s": len(active) * 0.01,
            "peak_acceleration_at_unit_available_accel":
                max((abs(value) for value in active), default=0.0),
            "absolute_acceleration_impulse_at_unit_available_accel":
                sum(abs(value) for value in active) * 0.01,
        }
    return {
        "schema_version": 1,
        "source_run": str(run_dir),
        "selection_status": "INSUFFICIENT_PAIRED_RESPONSE_EVIDENCE",
        "selected_level": None,
        "candidates": candidates,
        "reason": (
            "The source is an undisturbed M1 run. It establishes spatial "
            "coverage and bounded excitation only; it contains neither M1b "
            "response nor closed-loop response to risk_disturbance_v1, so it "
            "cannot establish that M1 wins."),
        "required_to_select": (
            "paired M1_R1 and M1b_R1 software/fake or pilot runs using the "
            "same frozen level, initialisation, path, calibration and gates"),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        report = screen(args.run_dir.resolve())
    except RuntimeError as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report["selected_level"] is not None else 4


if __name__ == "__main__":
    sys.exit(main())
