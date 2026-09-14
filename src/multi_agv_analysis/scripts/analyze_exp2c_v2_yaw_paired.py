#!/usr/bin/env python3
"""Compare fresh M1/M1b Exp2c-v2 fake runs at frozen A=0.020 m/s."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path


WHEEL_MARGIN_SCALE = 0.010


def num(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def flag(row, key):
    return str(row.get(key, "")).lower() in ("1", "1.0", "true", "yes")


def finite(values):
    return [value for value in values if math.isfinite(value)]


def mean(values):
    values = finite(values)
    return sum(values) / len(values) if values else None


def rms(values):
    values = finite(values)
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def maximum_absolute(values):
    values = finite(values)
    return max((abs(value) for value in values), default=None)


def percentile(values, fraction):
    values = sorted(finite(values))
    if not values:
        return None
    position = fraction * (len(values) - 1)
    lower, upper = int(math.floor(position)), int(math.ceil(position))
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def distance(row, first, second, suffix):
    ax, ay = num(row, "agv{}_{}_x".format(first, suffix)), num(
        row, "agv{}_{}_y".format(first, suffix))
    bx, by = num(row, "agv{}_{}_x".format(second, suffix)), num(
        row, "agv{}_{}_y".format(second, suffix))
    return math.hypot(ax - bx, ay - by) if all(
        math.isfinite(v) for v in (ax, ay, bx, by)) else math.nan


def load_run(run_dir):
    validation = json.loads((run_dir / "validation.json").read_text())
    summary = json.loads((run_dir / "summary_metrics.json").read_text())
    with (run_dir / "converted" / "aligned_samples.csv").open(
            newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return validation, summary, rows


def run_metrics(run_dir):
    validation, summary, rows = load_run(run_dir)
    active = [row for row in rows if flag(row, "yaw_drive_disturbance_active")]
    margins, dominant = [], 0
    previous = None
    for row in rows:
        left = num(row, "yaw_drive_disturbance_left_nominal")
        right = num(row, "yaw_drive_disturbance_right_nominal")
        if previous is not None and flag(row, "yaw_drive_disturbance_active"):
            limit = min(num(row, "agv2_wheel_left_reported_limit"),
                        num(row, "agv2_wheel_right_reported_limit"))
            margin = max(0.0, min(1.0, (
                limit - max(abs(previous[0]), abs(previous[1]))) /
                WHEEL_MARGIN_SCALE))
            margins.append(margin)
            robust = num(row, "agv2_robust_margin")
            dominant += int(margin < 0.998 and math.isfinite(robust) and
                            abs(margin - robust) <= 2.0e-3)
        if math.isfinite(left) and math.isfinite(right):
            previous = (left, right)

    raw_left = [num(row, "yaw_drive_disturbance_left_nominal") for row in active]
    raw_right = [num(row, "yaw_drive_disturbance_right_nominal") for row in active]
    raw_all = raw_left + raw_right
    differential = [right - left for left, right in zip(raw_left, raw_right)]
    robust = [num(row, "agv2_robust_margin") for row in active]
    risk_factor = [num(row, "agv2_risk_factor") for row in active]
    risk_signal = [num(row, "agv2_risk_signal") for row in active]
    contraction = [num(row, "risk_contraction") for row in active]
    reference = [num(row, "common_velocity_reference") for row in active]
    actual_velocity = [num(row, "agv2_s_dot_actual") for row in active]
    observed_amplitude = [num(
        row, "yaw_drive_disturbance_amplitude") for row in active]
    mapped_capability = [num(
        row, "agv2_mapped_path_velocity_upper") for row in active]
    baseline = [num(row, "baseline_common_upper") for row in active]
    reference_drop = [max(0.0, base - ref) for base, ref in
                      zip(baseline, reference) if math.isfinite(base + ref)]
    wheel_ratios = []
    for row in active:
        for side in ("left", "right"):
            demand = num(row, "yaw_drive_disturbance_{}_nominal".format(side))
            limit = num(row, "agv2_wheel_{}_reported_limit".format(side))
            if math.isfinite(demand) and math.isfinite(limit) and limit > 0:
                wheel_ratios.append(abs(demand) / limit)
    speed_limited = sum(any(flag(row, "agv2_wheel_{}_speed_limit_active".format(side))
                            for side in ("left", "right")) for row in active)
    hard_margin = [num(row, "agv2_hard_inner_upper") -
                   abs(num(row, "agv2_s_dot_actual")) for row in active]
    heading = [wrap(num(row, "agv2_robot_pose_yaw") -
                    num(row, "agv2_support_reference_yaw")) for row in active]
    side_errors = []
    for row in active:
        for first, second in ((1, 2), (1, 3), (2, 3)):
            actual = distance(row, first, second, "support_pose")
            expected = distance(row, first, second, "support_reference")
            side_errors.append(actual - expected)
    sample_period = float(summary.get("sample_period", 0.01))
    geometry = summary.get("geometry", {})
    path = summary.get("path", {}).get("per_robot", {}).get("agv2", {})
    return {
        "validation_valid": bool(validation.get("valid")),
        "validation_issue_codes": [x.get("code") for x in validation.get("issues", [])],
        "active_samples": len(active),
        "robust_margin_minimum": min(finite(robust), default=None),
        "robust_margin_mean": mean(robust),
        "wheel_margin_minimum": min(margins, default=None),
        "wheel_margin_mean": mean(margins),
        "wheel_margin_p05": percentile(margins, 0.05),
        "wheel_margin_dominant_fraction": float(dominant) / len(margins) if margins else None,
        "risk_factor_peak": maximum_absolute(risk_factor),
        "risk_factor_mean": mean(risk_factor),
        "risk_signal_peak": maximum_absolute(risk_signal),
        "risk_signal_mean": mean(risk_signal),
        "risk_contraction_peak": max(finite(contraction), default=None),
        "risk_contraction_integral": sum(finite(contraction)) * sample_period,
        "common_reference_velocity_minimum_mps": min(finite(reference), default=None),
        "common_reference_velocity_mean_mps": mean(reference),
        "robot2_actual_velocity_mean_mps": mean(actual_velocity),
        "reference_drop_peak_mps": max(reference_drop, default=None),
        "reference_drop_mean_mps": mean(reference_drop),
        "controller_wheel_raw_peak_mps": maximum_absolute(raw_all),
        "controller_wheel_raw_rms_mps": rms(raw_all),
        "controller_differential_wheel_peak_mps": maximum_absolute(differential),
        "controller_differential_wheel_rms_mps": rms(differential),
        "wheel_demand_capability_ratio_peak": max(wheel_ratios, default=None),
        "speed_limiter_duration_seconds": speed_limited * sample_period,
        "hard_capability_margin_minimum_mps": min(finite(hard_margin), default=None),
        "mapped_capability_minimum_mps": min(
            finite(mapped_capability), default=None),
        "mapped_capability_maximum_mps": max(
            finite(mapped_capability), default=None),
        "capability_report_constancy_range_mps": summary.get(
            "capability", {}).get("reported_constancy_range", {}).get("agv2"),
        "derating_active_samples": sum(flag(
            row, "agv2_capability_derating_active") for row in active),
        "disturbance_amplitude_minimum_mps": min(
            finite(observed_amplitude), default=None),
        "disturbance_amplitude_maximum_mps": max(
            finite(observed_amplitude), default=None),
        "robot2_heading_error_rmse_rad": rms(heading),
        "robot2_heading_error_max_rad": maximum_absolute(heading),
        "robot2_support_position_rmse_m": geometry.get("support", {}).get("agv2", {}).get("position_rmse"),
        "robot2_support_position_max_m": geometry.get("support", {}).get("agv2", {}).get("position_max"),
        "rigid_fit_residual_rmse_m": geometry.get("rigid_fit_residual_rmse"),
        "rigid_fit_residual_max_m": geometry.get("rigid_fit_residual_max"),
        "pairwise_side_error_rmse_m": rms(side_errors),
        "pairwise_side_error_max_m": maximum_absolute(side_errors),
        "equivalent_load_position_rmse_m": geometry.get("load_position_rmse"),
        "equivalent_load_yaw_rmse_rad": geometry.get("load_yaw_rmse"),
        "robot2_progress_rmse_m": path.get("progress_rmse"),
        "robot2_velocity_rmse_mps": path.get("velocity_rmse"),
        "task_completion_time_seconds": summary.get("task", {}).get("completion_time"),
        "reference_drop_definition": "baseline_common_upper_minus_common_velocity_reference",
    }, active


def interpolate(samples, coordinate, fields, grid):
    points = [(num(row, coordinate), row) for row in samples
              if math.isfinite(num(row, coordinate))]
    points.sort(key=lambda item: item[0])
    output, cursor = [], 0
    for value in grid:
        while cursor + 1 < len(points) and points[cursor + 1][0] < value:
            cursor += 1
        if not points or value < points[0][0] or value > points[-1][0] or cursor + 1 >= len(points):
            continue
        x0, row0 = points[cursor]
        x1, row1 = points[cursor + 1]
        weight = 0.0 if x1 == x0 else (value - x0) / (x1 - x0)
        result = {coordinate: value}
        valid = True
        for field in fields:
            y0, y1 = num(row0, field), num(row1, field)
            if not math.isfinite(y0 + y1):
                valid = False
                break
            result[field] = y0 + weight * (y1 - y0)
        if valid:
            output.append(result)
    return output


def aligned_comparison(m1_rows, m1b_rows, domain, output_path):
    fields = ("common_velocity_reference", "agv2_robust_margin",
              "yaw_drive_disturbance_left_nominal",
              "yaw_drive_disturbance_right_nominal", "agv2_s_dot_actual")
    if domain == "time":
        for rows in (m1_rows, m1b_rows):
            origin = num(rows[0], "stamp")
            for row in rows:
                row["paired_relative_time"] = num(row, "stamp") - origin
        coordinate = "paired_relative_time"
    else:
        coordinate = "yaw_drive_disturbance_path_progress"
    low = max(min(num(row, coordinate) for row in m1_rows),
              min(num(row, coordinate) for row in m1b_rows))
    high = min(max(num(row, coordinate) for row in m1_rows),
               max(num(row, coordinate) for row in m1b_rows))
    count = 401
    grid = [low + (high - low) * index / (count - 1) for index in range(count)]
    m1 = interpolate(m1_rows, coordinate, fields, grid)
    m1b = interpolate(m1b_rows, coordinate, fields, grid)
    rows = []
    for left, right in zip(m1, m1b):
        row = {coordinate: left[coordinate]}
        for field in fields:
            row["m1_" + field] = left[field]
            row["m1b_" + field] = right[field]
            row["delta_m1_minus_m1b_" + field] = left[field] - right[field]
        rows.append(row)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    return {
        "samples": len(rows), "domain_minimum": low, "domain_maximum": high,
        "common_reference_delta_mean_mps": mean([
            row["delta_m1_minus_m1b_common_velocity_reference"] for row in rows]),
        "common_reference_delta_minimum_mps": min([
            row["delta_m1_minus_m1b_common_velocity_reference"] for row in rows]),
        "differential_wheel_burden_delta_rms_mps": rms([
            abs(row["m1_yaw_drive_disturbance_right_nominal"] -
                row["m1_yaw_drive_disturbance_left_nominal"]) -
            abs(row["m1b_yaw_drive_disturbance_right_nominal"] -
                row["m1b_yaw_drive_disturbance_left_nominal"])
            for row in rows]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("paired_root", type=Path)
    args = parser.parse_args()
    root = args.paired_root.resolve()
    try:
        m1b, m1b_rows = run_metrics(root / "M1b_R1")
        m1, m1_rows = run_metrics(root / "M1_R1")
        time_alignment = aligned_comparison(
            m1_rows, m1b_rows, "time", root / "paired_time_domain.csv")
        progress_alignment = aligned_comparison(
            m1_rows, m1b_rows, "progress", root / "paired_robot2_progress_domain.csv")
    except Exception as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    report = {
        "schema_version": 1, "amplitude_mps": 0.020,
        "disturbance_window_m": [2.0, 2.8],
        "methods": {"M1b_R1": m1b, "M1_R1": m1},
        "time_domain_alignment": time_alignment,
        "robot2_actual_progress_domain_alignment": progress_alignment,
        "m1_triggered": (m1["risk_contraction_peak"] or 0.0) > 1.0e-9,
        "m1_lowered_reference": (m1["common_reference_velocity_mean_mps"] <
                                 m1b["common_reference_velocity_mean_mps"] - 1.0e-5),
        "m1_reduced_differential_wheel_burden": (
            m1["controller_differential_wheel_rms_mps"] <
            m1b["controller_differential_wheel_rms_mps"]),
        "m1_improved_wheel_margin": m1["wheel_margin_minimum"] > m1b["wheel_margin_minimum"],
        "m1_improved_heading_rmse": m1["robot2_heading_error_rmse_rad"] < m1b["robot2_heading_error_rmse_rad"],
        "m1_improved_support_rmse": m1["robot2_support_position_rmse_m"] < m1b["robot2_support_position_rmse_m"],
        "m1_improved_rigid_fit_rmse": m1["rigid_fit_residual_rmse_m"] < m1b["rigid_fit_residual_rmse_m"],
        "task_time_cost_m1_minus_m1b_seconds": (
            m1["task_completion_time_seconds"] - m1b["task_completion_time_seconds"]),
        "longitudinal_velocity_synchronisation_normal": {
            "M1b_R1": 0.097 <= m1b["robot2_actual_velocity_mean_mps"] <= 0.103,
            "M1_R1": 0.097 <= m1["robot2_actual_velocity_mean_mps"] <= 0.103,
        },
        "physical_authorization_changed": False,
    }
    (root / "paired_comparison.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# Exp2c-v2 paired fake comparison", "", "Frozen A = 0.020 m/s", "",
             "| metric | M1b+R1 | M1+R1 |", "|---|---:|---:|"]
    for key in m1b:
        if isinstance(m1b[key], (int, float)) and not isinstance(m1b[key], bool):
            lines.append("| {} | {} | {} |".format(key, m1b[key], m1.get(key)))
    (root / "paired_comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
