#!/usr/bin/env python3
"""Select an exploratory Exp2c level from completed fake M1/M1b pairs."""

import argparse
import json
import math
import sys
from pathlib import Path


LEVELS = ("0p30", "0p45", "0p60", "0p70")


def load_metrics(path):
    with path.open(encoding="utf-8") as stream:
        summary = json.load(stream)
    metrics = summary.get("risk_disturbance_v1", {})
    required = (
        "active_samples", "robot2_progress_rmse",
        "robot2_progress_max_absolute", "robot2_velocity_rmse",
        "peak_injected_acceleration", "minimum_hard_velocity_margin")
    if any(metrics.get(name) is None for name in required):
        raise RuntimeError("incomplete risk metrics in {}".format(path))
    if int(metrics["active_samples"]) <= 0:
        raise RuntimeError("no active disturbance samples in {}".format(path))
    return summary, metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("screen_root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.screen_root.resolve()
    candidates = {}
    eligible = []
    try:
        for level in LEVELS:
            level_dir = root / level
            m1_summary, m1 = load_metrics(
                level_dir / "M1_R1" / "summary_metrics.json")
            m1b_summary, m1b = load_metrics(
                level_dir / "M1b_R1" / "summary_metrics.json")
            # A spatially triggered disturbance lasts longer if a method
            # slows down. Requiring identical sample counts would remove part
            # of the closed-loop response. The runner locks the profile and
            # level; sampled peak agreement checks that lock in the data.
            same_exposure = math.isclose(
                float(m1["peak_injected_acceleration"]),
                float(m1b["peak_injected_acceleration"]),
                rel_tol=0.02, abs_tol=1.0e-6)
            progress_ratio = (float(m1["robot2_progress_rmse"]) /
                              float(m1b["robot2_progress_rmse"]))
            maximum_ratio = (float(m1["robot2_progress_max_absolute"]) /
                             float(m1b["robot2_progress_max_absolute"]))
            velocity_ratio = (float(m1["robot2_velocity_rmse"]) /
                              float(m1b["robot2_velocity_rmse"]))
            passes = (
                same_exposure and progress_ratio <= 0.90 and
                maximum_ratio <= 0.95 and velocity_ratio <= 1.05 and
                float(m1["minimum_hard_velocity_margin"]) >= -1.0e-6 and
                float(m1b["minimum_hard_velocity_margin"]) >= -1.0e-6)
            result = {
                "m1_run_id": m1_summary.get("run_id"),
                "m1b_run_id": m1b_summary.get("run_id"),
                "same_disturbance_exposure": same_exposure,
                "m1_active_samples": m1["active_samples"],
                "m1b_active_samples": m1b["active_samples"],
                "m1_robot2_progress_rmse": m1["robot2_progress_rmse"],
                "m1b_robot2_progress_rmse": m1b["robot2_progress_rmse"],
                "progress_rmse_ratio_m1_over_m1b": progress_ratio,
                "progress_max_ratio_m1_over_m1b": maximum_ratio,
                "velocity_rmse_ratio_m1_over_m1b": velocity_ratio,
                "m1_risk_contraction_peak": m1.get("risk_contraction_peak"),
                "m1b_risk_contraction_peak": m1b.get("risk_contraction_peak"),
                "passes_exploratory_screen": passes,
            }
            candidates[level] = result
            if passes:
                eligible.append((progress_ratio, level))
    except (OSError, ValueError, RuntimeError, ZeroDivisionError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2

    selected = min(eligible)[1] if eligible else None
    report = {
        "schema_version": 1,
        "scope": "software_fake_exploratory_screen_only",
        "selection_rule": (
            "same exposure; M1 Robot2 progress RMSE <= 90% of M1b; "
            "progress maximum <= 95%; velocity RMSE <= 105%; both hard "
            "capability margins nonnegative; choose the lowest RMSE ratio"),
        "selected_level": selected,
        "selection_status": (
            "SELECTED_FOR_FURTHER_SOFTWARE_REVIEW" if selected else
            "NO_LEVEL_MET_THE_PREDECLARED_RULE"),
        "candidates": candidates,
        "physical_authorization_changed": False,
        "warning": (
            "Selection is exploratory fake evidence and does not establish "
            "physical performance or authorize a vehicle run."),
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    output = args.output or root / "selection_report.json"
    output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if selected else 4


if __name__ == "__main__":
    sys.exit(main())
