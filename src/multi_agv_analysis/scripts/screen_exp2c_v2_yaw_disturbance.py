#!/usr/bin/env python3
"""Select one Exp2c-v2 yaw disturbance using M1b fake runs only."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path


CANDIDATE_SETS = {
    "round1": (("0p004", 0.004), ("0p007", 0.007),
               ("0p010", 0.010), ("0p012", 0.012)),
    "round2": (("0p016", 0.016), ("0p020", 0.020),
               ("0p024", 0.024)),
}
CANDIDATES = CANDIDATE_SETS["round1"]
WHEEL_MARGIN_SCALE = 0.010
CENTRAL_EXCITATION_THRESHOLD = 0.50


def number(row, name):
    try:
        value = float(row.get(name, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def flag(row, name):
    return str(row.get(name, "")).strip().lower() in (
        "1", "1.0", "true", "yes", "on")


def mean(values):
    values = [value for value in values if math.isfinite(value)]
    return sum(values) / len(values) if values else None


def rmse(values):
    values = [value for value in values if math.isfinite(value)]
    return (math.sqrt(sum(value * value for value in values) / len(values))
            if values else None)


def percentile(values, fraction):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    position = fraction * (len(values) - 1)
    lower, upper = int(math.floor(position)), int(math.ceil(position))
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def load_json(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def evaluate_candidate(run_dir, amplitude):
    validation = load_json(run_dir / "validation.json")
    summary = load_json(run_dir / "summary_metrics.json")
    rows = load_rows(run_dir / "converted" / "aligned_samples.csv")
    active = [row for row in rows
              if flag(row, "yaw_drive_disturbance_active")]
    central = [row for row in active if abs(
        number(row, "yaw_drive_disturbance_window") * math.sin(
            number(row, "yaw_drive_disturbance_phase"))) >=
        CENTRAL_EXCITATION_THRESHOLD]

    causal_margins = []
    dominant_samples = 0
    prior_nominal = None
    for row in rows:
        left = number(row, "yaw_drive_disturbance_left_nominal")
        right = number(row, "yaw_drive_disturbance_right_nominal")
        if prior_nominal is not None and row in central:
            limit = min(number(row, "agv2_wheel_left_reported_limit"),
                        number(row, "agv2_wheel_right_reported_limit"))
            margin = max(0.0, min(1.0, (
                limit - max(abs(prior_nominal[0]), abs(prior_nominal[1]))) /
                WHEEL_MARGIN_SCALE))
            causal_margins.append(margin)
            robust = number(row, "agv2_robust_margin")
            if (margin < 1.0 - 2.0e-3 and math.isfinite(robust) and
                    abs(robust - margin) <= 2.0e-3):
                dominant_samples += 1
        if math.isfinite(left) and math.isfinite(right):
            prior_nominal = (left, right)

    actual_velocity = [number(row, "agv2_s_dot_actual") for row in active]
    velocity_error = [number(row, "agv2_s_dot_actual") -
                      number(row, "common_velocity_reference")
                      for row in active]
    nominal_wheels = [number(row, name) for row in active for name in (
        "yaw_drive_disturbance_left_nominal",
        "yaw_drive_disturbance_right_nominal")]
    disturbed_wheels = [number(row, name) for row in active for name in (
        "yaw_drive_disturbance_left_disturbed",
        "yaw_drive_disturbance_right_disturbed")]
    longitudinal = [number(
        row, "yaw_drive_disturbance_mean_longitudinal_delta") for row in active]
    differential_nominal = [
        number(row, "yaw_drive_disturbance_right_nominal") -
        number(row, "yaw_drive_disturbance_left_nominal") for row in active]
    speed_limited = [row for row in active if any(flag(
        row, "agv2_wheel_{}_speed_limit_active".format(side))
        for side in ("left", "right"))]
    hard_margins = [number(row, "agv2_hard_inner_upper") -
                    abs(number(row, "agv2_s_dot_actual")) for row in active]
    mapped_capability = [number(
        row, "agv2_mapped_path_velocity_upper") for row in active]
    capability_ranges = summary.get("capability", {}).get(
        "reported_constancy_range", {})
    capability_constant = all(
        value is not None and abs(float(value)) <= 1.0e-12
        for value in capability_ranges.values())
    no_derating = not any(flag(row, "agv2_capability_derating_active")
                           for row in rows)
    central_minimum = min(causal_margins) if causal_margins else None
    dominant_fraction = (float(dominant_samples) / len(causal_margins)
                         if causal_margins else 0.0)
    speed_limit_duration = len(speed_limited) * float(
        summary.get("sample_period", 0.01))
    actual_mean = mean(actual_velocity)
    maximum_controller_raw = max(
        (abs(value) for value in nominal_wheels if math.isfinite(value)),
        default=None)
    minimum_hard_margin = min(
        (value for value in hard_margins if math.isfinite(value)),
        default=None)
    mapped_minimum = min(
        (value for value in mapped_capability if math.isfinite(value)),
        default=None)
    mapped_maximum = max(
        (value for value in mapped_capability if math.isfinite(value)),
        default=None)
    task_complete = bool(validation.get("task_completion", {}).get("complete"))
    valid = bool(validation.get("valid"))
    passes = all((
        bool(active), bool(central), capability_constant, no_derating, valid,
        task_complete, central_minimum is not None,
        mapped_minimum is not None and mapped_minimum > 0.10,
        0.40 <= central_minimum <= 0.70,
        dominant_fraction > 0.0,
        maximum_controller_raw is not None and maximum_controller_raw < 0.160,
        speed_limit_duration == 0.0,
        actual_mean is not None and 0.097 <= actual_mean <= 0.103,
        minimum_hard_margin is not None and minimum_hard_margin > 0.0,
        (max((abs(value) for value in longitudinal
              if math.isfinite(value)), default=math.inf) <= 1.0e-12),
    ))
    return {
        "amplitude_mps": amplitude,
        "run_dir": str(run_dir),
        "method_id": summary.get("method_id"),
        "active_samples": len(active),
        "central_samples": len(central),
        "agv2_actual_velocity_mean_mps": actual_mean,
        "agv2_velocity_error_rmse_mps": rmse(velocity_error),
        "agv2_velocity_error_mean_signed_mps": mean(velocity_error),
        "agv2_velocity_error_max_absolute_mps": max(
            (abs(value) for value in velocity_error if math.isfinite(value)),
            default=None),
        "controller_nominal_wheel_peak_mps": maximum_controller_raw,
        "controller_nominal_left_peak_mps": max(
            (abs(number(row, "yaw_drive_disturbance_left_nominal"))
             for row in active), default=None),
        "controller_nominal_right_peak_mps": max(
            (abs(number(row, "yaw_drive_disturbance_right_nominal"))
             for row in active), default=None),
        "central_causal_wheel_margin_minimum": central_minimum,
        "central_causal_wheel_margin_p05": percentile(causal_margins, 0.05),
        "central_causal_wheel_margin_mean": mean(causal_margins),
        "central_wheel_margin_below_1_fraction": mean(
            [float(value < 1.0) for value in causal_margins]),
        "central_wheel_margin_below_0p7_fraction": mean(
            [float(value < 0.7) for value in causal_margins]),
        "central_wheel_margin_dominant_fraction": dominant_fraction,
        "nominal_wheel_differential_peak_mps": max(
            (abs(value) for value in differential_nominal
             if math.isfinite(value)), default=None),
        "post_disturbance_wheel_peak_mps": max(
            (abs(value) for value in disturbed_wheels if math.isfinite(value)),
            default=None),
        "longitudinal_delta_max_absolute_mps": max(
            (abs(value) for value in longitudinal if math.isfinite(value)),
            default=None),
        "longitudinal_delta_rmse_mps": rmse(longitudinal),
        "speed_limiter_duration_seconds": speed_limit_duration,
        "minimum_hard_capability_margin_mps": minimum_hard_margin,
        "mapped_capability_minimum_mps": mapped_minimum,
        "mapped_capability_maximum_mps": mapped_maximum,
        "rigid_fit_residual_rmse_m": summary.get("geometry", {}).get(
            "rigid_fit_residual_rmse"),
        "agv2_heading_rmse_rad": summary.get("geometry", {}).get(
            "vehicle_heading", {}).get("agv2", {}).get("rmse"),
        "agv2_support_position_rmse_m": summary.get("geometry", {}).get(
            "support", {}).get("agv2", {}).get("position_rmse"),
        "capability_report_constant": capability_constant,
        "derating_absent": no_derating,
        "validation_valid": valid,
        "validation_issue_codes": [item.get("code") for item in
                                    validation.get("issues", [])],
        "task_complete": task_complete,
        "passes_predeclared_m1b_screen": passes,
    }


def rejected_candidate(run_dir, amplitude, error):
    validation_path = run_dir / "validation.json"
    if not validation_path.exists():
        raise RuntimeError(
            "candidate has neither metrics nor validation: {} ({})".format(
                run_dir, error))
    validation = load_json(validation_path)
    return {
        "amplitude_mps": amplitude,
        "run_dir": str(run_dir),
        "method_id": "M1b_R1",
        "active_samples": None,
        "central_samples": None,
        "mapped_capability_minimum_mps": None,
        "mapped_capability_maximum_mps": None,
        "capability_report_constant": None,
        "derating_absent": None,
        "validation_valid": bool(validation.get("valid")),
        "validation_issue_codes": [item.get("code") for item in
                                    validation.get("issues", [])],
        "task_complete": bool(
            validation.get("task_completion", {}).get("complete")),
        "passes_predeclared_m1b_screen": False,
        "rejection_reason": "incomplete_or_invalid_run: {}".format(error),
    }


def render_markdown(report):
    lines = [
        "# Exp2c-v2 M1b fake yaw-disturbance screen", "",
        "M1 results consulted: **no**. Hardware authorization changed: **no**.", "",
        "| amplitude (m/s) | min wheel margin | dominant | mean v2 (m/s) | "
        "speed-limit (s) | pass |",
        "|---:|---:|---:|---:|---:|:---:|",
    ]
    for item in report["candidates"]:
        def show(name):
            value = item.get(name)
            return "n/a" if value is None else "{:.6f}".format(value)
        lines.append("| {amplitude_mps:.3f} | {margin} | {dominant} | "
                     "{velocity} | {limit} | {passed} |".format(
                         amplitude_mps=item["amplitude_mps"],
                         margin=show("central_causal_wheel_margin_minimum"),
                         dominant=show("central_wheel_margin_dominant_fraction"),
                         velocity=show("agv2_actual_velocity_mean_mps"),
                         limit=show("speed_limiter_duration_seconds"),
                         passed="yes" if item[
                             "passes_predeclared_m1b_screen"] else "no"))
    lines.extend(("", "Selected amplitude: **{}**".format(
        "none" if report["selected_amplitude_mps"] is None else
        "{:.3f} m/s".format(report["selected_amplitude_mps"])), "",
        "Status: `{}`".format(report["selection_status"]), ""))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("screen_root", type=Path)
    parser.add_argument("--candidate-set", choices=tuple(CANDIDATE_SETS),
                        default="round1")
    args = parser.parse_args()
    root = args.screen_root.resolve()
    candidates = CANDIDATE_SETS[args.candidate_set]
    results = []
    try:
        for name, amplitude in candidates:
            run_dir = root / name / "M1b_R1"
            try:
                result = evaluate_candidate(run_dir, amplitude)
            except (OSError, ValueError, RuntimeError, KeyError) as error:
                result = rejected_candidate(run_dir, amplitude, error)
            if result["method_id"] != "M1b_R1":
                raise RuntimeError("candidate is not M1b_R1: {}".format(run_dir))
            results.append(result)
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    mapped_minima = [item["mapped_capability_minimum_mps"]
                     for item in results
                     if item["mapped_capability_minimum_mps"] is not None]
    mapped_maxima = [item["mapped_capability_maximum_mps"]
                     for item in results
                     if item["mapped_capability_maximum_mps"] is not None]
    mapped_consistent = (
        len(mapped_minima) >= 2 and len(mapped_maxima) >= 2 and
        max(mapped_minima) - min(mapped_minima) <= 1.0e-5 and
        max(mapped_maxima) - min(mapped_maxima) <= 1.0e-5)
    if not mapped_consistent:
        for item in results:
            item["passes_predeclared_m1b_screen"] = False
    selected = next((item for item in results
                     if item["passes_predeclared_m1b_screen"]), None)
    report = {
        "schema_version": 1,
        "experiment": "Exp2c-v2 Robot2 local asymmetric drive disturbance",
        "scope": "software_fake_m1b_candidate_screen_only",
        "candidate_set": args.candidate_set,
        "m1_results_consulted": False,
        "selection_order_mps": [value for _, value in candidates],
        "mapped_capability_consistent_across_candidates": mapped_consistent,
        "selected_amplitude_mps": (
            selected["amplitude_mps"] if selected else None),
        "selection_status": (
            "SELECTED_FOR_PAIRED_FAKE_REVIEW" if selected else
            "NO_CANDIDATE_MET_PREDECLARED_M1B_RULE"),
        "candidates": results,
        "m1_run_started": False,
        "physical_authorization_changed": False,
        "warning": "Fake evidence does not authorize serial or hardware execution.",
    }
    (root / "selection_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (root / "selection_report.md").write_text(
        render_markdown(report), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if selected else 4


if __name__ == "__main__":
    sys.exit(main())
