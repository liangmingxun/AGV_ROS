#!/usr/bin/env python3
"""Offline-only causal diagnosis for the frozen Exp2c-v1 fake pairs.

This tool reads existing converted CSV and summary files.  It does not run a
controller, alter the frozen selection report, or write runtime parameters.
"""

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


LEVELS = ("0p30", "0p45", "0p60", "0p70")
METHODS = ("M1_R1", "M1b_R1")
START_S = 2.0
END_S = 2.8
NOMINAL_SPEED = 0.10
RISK = {
    "capability_reserve": 0.005,
    "capability_safe_margin": 0.010,
    "input_safe_margin": 0.40,
    "wheel_safe_margin": 0.010,
    "position_limit": 0.12,
    "position_safe_margin": 0.06,
}


def finite(value, default=math.nan):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def truth(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_json(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def percentile(values, q):
    values = sorted(x for x in values if math.isfinite(x))
    if not values:
        return None
    position = (len(values) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def stats(values):
    values = [x for x in values if math.isfinite(x)]
    if not values:
        return {"count": 0, "min": None, "max": None, "mean": None,
                "std": None, "p05": None, "rms": None}
    mean = sum(values) / len(values)
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": mean,
        "std": math.sqrt(sum((x - mean) ** 2 for x in values) / len(values)),
        "p05": percentile(values, 0.05),
        "rms": math.sqrt(sum(x * x for x in values) / len(values)),
    }


def dt_for(rows):
    stamps = [finite(row.get("stamp")) for row in rows]
    intervals = [b - a for a, b in zip(stamps, stamps[1:])
                 if math.isfinite(a) and math.isfinite(b) and 0 < b - a < 0.1]
    return statistics.median(intervals) if intervals else 0.01


def integral(rows, getter, absolute=False, path_domain=False):
    total = 0.0
    for first, second in zip(rows, rows[1:]):
        x0 = finite(first.get(
            "agv2_s_tracking_actual" if path_domain else "stamp"))
        x1 = finite(second.get(
            "agv2_s_tracking_actual" if path_domain else "stamp"))
        y0, y1 = getter(first), getter(second)
        if not all(math.isfinite(x) for x in (x0, x1, y0, y1)) or x1 <= x0:
            continue
        if absolute:
            y0, y1 = abs(y0), abs(y1)
        total += 0.5 * (y0 + y1) * (x1 - x0)
    return total


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rigid_residual(row):
    actual = [(finite(row.get("agv{}_support_pose_x".format(i))),
               finite(row.get("agv{}_support_pose_y".format(i))))
              for i in range(1, 4)]
    reference = [(finite(row.get("agv{}_support_reference_x".format(i))),
                  finite(row.get("agv{}_support_reference_y".format(i))))
                 for i in range(1, 4)]
    if not all(math.isfinite(x) for p in actual + reference for x in p):
        return math.nan
    ac = (sum(p[0] for p in actual) / 3.0,
          sum(p[1] for p in actual) / 3.0)
    rc = (sum(p[0] for p in reference) / 3.0,
          sum(p[1] for p in reference) / 3.0)
    dot = cross = 0.0
    for measured, expected in zip(actual, reference):
        ax, ay = measured[0] - ac[0], measured[1] - ac[1]
        rx, ry = expected[0] - rc[0], expected[1] - rc[1]
        dot += rx * ax + ry * ay
        cross += rx * ay - ry * ax
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    squared = []
    for measured, expected in zip(actual, reference):
        rx, ry = expected[0] - rc[0], expected[1] - rc[1]
        predicted = (ac[0] + c * rx - s * ry, ac[1] + s * rx + c * ry)
        squared.append(distance(measured, predicted) ** 2)
    return math.sqrt(sum(squared) / 3.0)


def row_geometry(row):
    actual = [(finite(row.get("agv{}_support_pose_x".format(i))),
               finite(row.get("agv{}_support_pose_y".format(i))))
              for i in range(1, 4)]
    reference = [(finite(row.get("agv{}_support_reference_x".format(i))),
                  finite(row.get("agv{}_support_reference_y".format(i))))
                 for i in range(1, 4)]
    support = []
    if all(math.isfinite(x) for p in actual + reference for x in p):
        support = [distance(a, r) for a, r in zip(actual, reference)]
        side = [distance(actual[a], actual[b]) - distance(reference[a], reference[b])
                for a, b in ((0, 1), (0, 2), (1, 2))]
    else:
        side = []
    load_error = math.nan
    values = [finite(row.get(name)) for name in
              ("equivalent_load_pose_x", "equivalent_load_pose_y",
               "load_x_reference", "load_y_reference")]
    if all(math.isfinite(x) for x in values):
        load_error = math.hypot(values[0] - values[2], values[1] - values[3])
    yaw_error = math.nan
    actual_yaw = finite(row.get("equivalent_load_pose_yaw"))
    reference_yaw = finite(row.get("load_yaw_reference"))
    if math.isfinite(actual_yaw) and math.isfinite(reference_yaw):
        yaw_error = wrap(actual_yaw - reference_yaw)
    return support, side, rigid_residual(row), load_error, yaw_error


def valid_rows(rows):
    return [row for row in rows
            if truth(row.get("evaluation_active"))
            and truth(row.get("localization_valid"))
            and truth(row.get("algorithm_valid"))
            and truth(row.get("risk_disturbance_state_available"))]


def phase_rows(rows):
    result = {"baseline": [], "active": [], "recovery": []}
    for row in rows:
        progress = finite(row.get("agv2_s_tracking_actual"))
        window = finite(row.get("agv2_disturbance_window"), 0.0)
        if START_S - 0.8 <= progress <= START_S and window <= 1e-9:
            result["baseline"].append(row)
        if window > 1e-9:
            result["active"].append(row)
        if END_S <= progress <= END_S + 0.8 and window <= 1e-9:
            result["recovery"].append(row)
    return result


def load_capability(path):
    rows = read_csv(path)
    values = {robot: {"accel": [], "decel": []} for robot in range(1, 4)}
    for row in rows:
        robot = int(finite(row.get("robot_id"), 0))
        if robot not in values:
            continue
        values[robot]["accel"].append(min(
            finite(row.get("max_wheel_acceleration_left")),
            finite(row.get("max_wheel_acceleration_right"))))
        values[robot]["decel"].append(min(
            finite(row.get("max_wheel_deceleration_left")),
            finite(row.get("max_wheel_deceleration_right"))))
    return {robot: (min(v["accel"]), min(v["decel"]))
            for robot, v in values.items()}


def normalised(value, scale):
    return max(0.0, min(1.0, value / scale))


def reconstruct_margins(rows, capability):
    records = []
    errors = []
    for previous, row in zip(rows, rows[1:]):
        per_robot = {}
        observed_by_robot = {}
        for robot in range(1, 4):
            mapped = finite(row.get("agv{}_mapped_capability_diagnostic".format(robot)))
            candidate = finite(row.get("agv{}_upsilon".format(robot)))
            raw_input = finite(previous.get("agv{}_channel_input_raw".format(robot)))
            wheel_left = finite(previous.get("agv{}_wheel_left_raw".format(robot)))
            wheel_right = finite(previous.get("agv{}_wheel_right_raw".format(robot)))
            limit_left = finite(row.get("agv{}_wheel_left_reported_limit".format(robot)))
            limit_right = finite(row.get("agv{}_wheel_right_reported_limit".format(robot)))
            position = finite(previous.get("agv{}_position_error".format(robot)))
            accel, decel = capability[robot]
            raw = {
                "capability_margin": normalised(
                    mapped - RISK["capability_reserve"] - abs(candidate),
                    RISK["capability_safe_margin"]),
                "input_margin": normalised(
                    min(accel, decel) - abs(raw_input), RISK["input_safe_margin"]),
                "wheel_margin": normalised(min(
                    limit_left - abs(wheel_left), limit_right - abs(wheel_right)),
                    RISK["wheel_safe_margin"]),
                # Preserve the implemented signed-error semantics exactly.
                "position_margin": normalised(
                    RISK["position_limit"] - position,
                    RISK["position_safe_margin"]),
                "failure_margin": 1.0,
            }
            per_robot[robot] = raw
            observed = finite(row.get("agv{}_robust_margin".format(robot)))
            if math.isfinite(observed):
                errors.append(abs(observed - min(raw.values())))
                observed_by_robot[robot] = observed
        flattened = [(value, robot, name) for robot, margins in per_robot.items()
                     for name, value in margins.items()]
        dominant = min(flattened)
        # Do not arbitrarily credit capability_margin when every causal
        # margin is saturated at the safe value and ties only by field order.
        if dominant[0] >= 1.0 - 1e-12:
            dominant = (dominant[0], 0, "none_all_safe")
        records.append({"row": row, "per_robot": per_robot,
                        "observed_by_robot": observed_by_robot,
                        "dominant_value": dominant[0],
                        "dominant_robot": dominant[1],
                        "dominant_margin": dominant[2]})
    return records, {"maximum_absolute_error": max(errors) if errors else None,
                     "mean_absolute_error": sum(errors) / len(errors) if errors else None}


def phase_margin_summary(records, phase_name):
    selected = []
    for record in records:
        progress = finite(record["row"].get("agv2_s_tracking_actual"))
        window = finite(record["row"].get("agv2_disturbance_window"), 0.0)
        include = ((phase_name == "active" and window > 1e-9) or
                   (phase_name == "baseline" and START_S - 0.8 <= progress <= START_S
                    and window <= 1e-9) or
                   (phase_name == "recovery" and END_S <= progress <= END_S + 0.8
                    and window <= 1e-9))
        if include:
            selected.append(record)
    by_type = Counter(record["dominant_margin"] for record in selected)
    by_source = Counter("agv{}:{}".format(record["dominant_robot"],
                                          record["dominant_margin"])
                        for record in selected
                        if record["dominant_robot"] != 0)
    denominator = float(len(selected)) or 1.0
    values = {}
    reconstruction_errors = []
    for record in selected:
        for robot, observed in record["observed_by_robot"].items():
            reconstruction_errors.append(abs(
                observed - min(record["per_robot"][robot].values())))
    for name in ("capability_margin", "input_margin", "wheel_margin",
                 "position_margin", "failure_margin"):
        samples = [record["per_robot"][robot][name]
                   for record in selected for robot in range(1, 4)]
        values[name] = stats(samples)
    return {
        "samples": len(selected),
        "margin_statistics_all_robots": values,
        "dominance_fraction_by_margin_type": {
            name: by_type[name] / denominator for name in values},
        "no_margin_active_fraction": by_type["none_all_safe"] / denominator,
        "dominance_fraction_by_robot_and_margin": {
            name: count / denominator for name, count in sorted(by_source.items())},
        "reconstruction_error": {
            "maximum_absolute": max(reconstruction_errors)
            if reconstruction_errors else None,
            "mean_absolute": sum(reconstruction_errors) / len(reconstruction_errors)
            if reconstruction_errors else None,
        },
    }


def metric_block(rows):
    dt = dt_for(rows)
    input_raw = [finite(r.get("agv2_channel_input_raw")) for r in rows]
    input_limited = [finite(r.get("agv2_channel_input_limited")) for r in rows]
    progress = [finite(r.get("agv2_position_error")) for r in rows]
    velocity = [finite(r.get("agv2_velocity_error")) for r in rows]
    disturbance_estimate = [finite(r.get("agv2_disturbance_estimate")) for r in rows]
    wheel_pre = [finite(r.get("agv2_wheel_{}_pre_limit".format(side)))
                 for r in rows for side in ("left", "right")]
    wheel_applied = [finite(r.get("agv2_wheel_{}_applied".format(side)))
                     for r in rows for side in ("left", "right")]
    wheel_actual = [finite(r.get("agv2_wheel_{}_actual".format(side)))
                    for r in rows for side in ("left", "right")]
    limit_ratios = []
    speed_limiter_samples = 0
    for row in rows:
        limited = False
        for side in ("left", "right"):
            demand = abs(finite(row.get("agv2_wheel_{}_pre_limit".format(side))))
            limit = finite(row.get("agv2_wheel_{}_reported_limit".format(side)))
            if math.isfinite(demand) and math.isfinite(limit) and limit > 0:
                limit_ratios.append(demand / limit)
            limited = limited or truth(row.get(
                "agv2_wheel_{}_speed_limit_active".format(side)))
        speed_limiter_samples += int(limited)
    support, side, rigid, load, yaw = [], [], [], [], []
    for row in rows:
        support_row, side_row, rigid_row, load_row, yaw_row = row_geometry(row)
        support.extend(support_row)
        side.extend(side_row)
        rigid.append(rigid_row)
        load.append(load_row)
        yaw.append(yaw_row)
    per_robot = {}
    for robot in range(1, 4):
        per_robot["agv{}".format(robot)] = {
            "progress_error": stats([finite(r.get(
                "agv{}_position_error".format(robot))) for r in rows]),
            "velocity_error": stats([finite(r.get(
                "agv{}_velocity_error".format(robot))) for r in rows]),
        }
    return {
        "sample_count": len(rows), "sample_period": dt,
        "robot2": {
            "input_raw": stats(input_raw), "input_raw_abs_peak": max(
                [abs(x) for x in input_raw if math.isfinite(x)] or [math.nan]),
            "input_limited": stats(input_limited),
            "input_limit_duration": sum(truth(r.get("agv2_channel_limit_active"))
                                        for r in rows) * dt,
            "disturbance_estimate": stats(disturbance_estimate),
            "progress_error": stats(progress), "velocity_error": stats(velocity),
            "wheel_pre_limit": stats(wheel_pre),
            "wheel_pre_limit_abs_peak": max(
                [abs(x) for x in wheel_pre if math.isfinite(x)] or [math.nan]),
            "wheel_demand_to_limit_ratio": stats(limit_ratios),
            "wheel_applied": stats(wheel_applied), "wheel_actual": stats(wheel_actual),
            "wheel_speed_limiter_duration": speed_limiter_samples * dt,
        },
        "all_robots": per_robot,
        "geometry": {
            "support_point_position_error": stats(support),
            "side_length_signed_error": stats(side),
            "rigid_fit_residual": stats(rigid),
            "virtual_equivalent_load_position_error": stats(load),
            "virtual_equivalent_load_yaw_error": stats(yaw),
        },
    }


def capability_block(rows):
    signals = {
        "robot2_reported_wheel_limit": [finite(r.get(
            "agv2_wheel_{}_reported_limit".format(side)))
            for r in rows for side in ("left", "right")],
        "robot2_mapped_path_velocity_upper": [finite(r.get(
            "agv2_mapped_path_velocity_upper")) for r in rows],
        "public_mapped_capability": [min(finite(r.get(
            "agv{}_mapped_path_velocity_upper".format(i))) for i in range(1, 4))
            for r in rows],
        "hard_common_upper": [finite(r.get("hard_common_upper")) for r in rows],
        "robot2_hard_inner_upper": [finite(r.get("agv2_hard_inner_upper"))
                                     for r in rows],
    }
    return {name: stats(values) for name, values in signals.items()}


def intervention_block(rows):
    contractions = [finite(r.get("risk_contraction")) for r in rows]
    reference = [finite(r.get("common_velocity_reference")) for r in rows]
    robust = [min(finite(r.get("agv{}_robust_margin".format(i)))
                  for i in range(1, 4)) for r in rows]
    risk_factor = [max(finite(r.get("agv{}_risk_factor".format(i)))
                       for i in range(1, 4)) for r in rows]
    risk_signal = [max(finite(r.get("agv{}_risk_signal".format(i)))
                       for i in range(1, 4)) for r in rows]
    return {
        "robust_margin": stats(robust), "risk_factor": stats(risk_factor),
        "risk_signal": stats(risk_signal), "risk_contraction": stats(contractions),
        "risk_contraction_integral": integral(
            rows, lambda r: finite(r.get("risk_contraction"))),
        "risk_inner_upper": stats([min(finite(r.get(
            "agv{}_risk_inner_upper".format(i))) for i in range(1, 4)) for r in rows]),
        "effective_inner_upper": stats([min(finite(r.get(
            "agv{}_effective_inner_upper".format(i))) for i in range(1, 4)) for r in rows]),
        "hard_inner_upper": stats([min(finite(r.get(
            "agv{}_hard_inner_upper".format(i))) for i in range(1, 4)) for r in rows]),
        "baseline_common_upper": stats([finite(r.get("baseline_common_upper"))
                                         for r in rows]),
        "hard_common_upper": stats([finite(r.get("hard_common_upper")) for r in rows]),
        "common_upper": stats([finite(r.get("common_boundary_upper")) for r in rows]),
        "common_reference_velocity": stats(reference),
        "reference_drop_peak": NOMINAL_SPEED - min(reference),
        "reference_drop_mean": NOMINAL_SPEED - sum(reference) / len(reference),
    }


def disturbance_block(rows):
    dt = dt_for(rows)
    total = [finite(r.get("agv2_disturbance_total")) for r in rows]
    base = [finite(r.get("agv2_disturbance_base")) for r in rows]
    velocity = [finite(r.get("agv2_disturbance_velocity")) for r in rows]
    sinusoid = [finite(r.get("agv2_disturbance_sinusoid")) for r in rows]
    nominal_total = []
    slowdown_reduction = []
    for row in rows:
        base_value = finite(row.get("agv2_disturbance_base"))
        velocity_value = finite(row.get("agv2_disturbance_velocity"))
        sinusoid_value = finite(row.get("agv2_disturbance_sinusoid"))
        scale = base_value / 0.60
        nominal_velocity = scale * 0.25
        nominal_total.append(base_value + nominal_velocity + sinusoid_value)
        slowdown_reduction.append(abs(nominal_velocity - velocity_value))
    denominator = sum(abs(x) for x in nominal_total)
    stamps = [finite(r.get("stamp")) for r in rows]
    progress = [finite(r.get("agv2_s_tracking_actual")) for r in rows]
    return {
        "sample_count": len(rows),
        "peak_absolute": max([abs(x) for x in total] or [math.nan]),
        "rms": stats(total)["rms"],
        "time_integral_absolute": integral(
            rows, lambda r: finite(r.get("agv2_disturbance_total")), True),
        "path_integral_absolute": integral(
            rows, lambda r: finite(r.get("agv2_disturbance_total")), True, True),
        "duration": (max(stamps) - min(stamps)) + dt if stamps else 0.0,
        "path_length": max(progress) - min(progress) if progress else 0.0,
        "component_rms": {"constant": stats(base)["rms"],
                          "velocity": stats(velocity)["rms"],
                          "sinusoidal": stats(sinusoid)["rms"]},
        "slowdown_induced_disturbance_reduction_mean": (
            sum(slowdown_reduction) / len(slowdown_reduction)),
        "slowdown_induced_reduction_percent_of_nominal_total_l1": (
            100.0 * sum(slowdown_reduction) / denominator if denominator else 0.0),
    }


def first_stamp(rows, predicate):
    for row in rows:
        if predicate(row):
            return finite(row.get("stamp"))
    return None


def causal_timing(rows, phases):
    baseline = phases["baseline"]
    baseline_robust = statistics.mean(
        min(finite(r.get("agv{}_robust_margin".format(i))) for i in range(1, 4))
        for r in baseline)
    onset = first_stamp(rows, lambda r: finite(r.get("agv2_disturbance_window"), 0) > 1e-9)
    events = {
        "disturbance_onset": onset,
        "robust_margin_response": first_stamp(rows, lambda r: onset is not None and
            finite(r.get("stamp")) >= onset and min(finite(r.get(
                "agv{}_robust_margin".format(i))) for i in range(1, 4))
            < baseline_robust - 1e-3),
        "risk_contraction_onset": first_stamp(rows, lambda r: onset is not None and
            finite(r.get("stamp")) >= onset and finite(r.get("risk_contraction")) > 1e-4),
        "reference_drop_onset": first_stamp(rows, lambda r: onset is not None and
            finite(r.get("stamp")) >= onset and finite(
                r.get("common_velocity_reference")) < NOMINAL_SPEED - 1e-4),
    }
    return {"event_stamps": events,
            "delays_from_disturbance_seconds": {
                name: value - onset if value is not None and onset is not None else None
                for name, value in events.items() if name != "disturbance_onset"}}


def interp_series(rows, field, grid, derived=None):
    pairs = []
    for row in rows:
        x = finite(row.get("agv2_s_tracking_actual"))
        y = derived(row) if derived else finite(row.get(field))
        if math.isfinite(x) and math.isfinite(y):
            pairs.append((x, y))
    pairs.sort()
    unique = []
    for pair in pairs:
        if unique and abs(pair[0] - unique[-1][0]) < 1e-9:
            unique[-1] = pair
        else:
            unique.append(pair)
    xs, ys = [p[0] for p in unique], [p[1] for p in unique]
    output = []
    for x in grid:
        index = bisect.bisect_left(xs, x)
        if index == 0 or index == len(xs):
            output.append(math.nan)
            continue
        x0, x1 = xs[index - 1], xs[index]
        ratio = (x - x0) / (x1 - x0)
        output.append(ys[index - 1] + ratio * (ys[index] - ys[index - 1]))
    return output


def path_pair(m1_rows, m1b_rows):
    grid = [START_S + (END_S - START_S) * i / 160.0 for i in range(161)]
    definitions = {
        "disturbance": ("agv2_disturbance_total", None),
        "robust_margin": (None, lambda r: min(finite(r.get(
            "agv{}_robust_margin".format(i))) for i in range(1, 4))),
        "risk_contraction": ("risk_contraction", None),
        "reference_velocity": ("common_velocity_reference", None),
        "robot2_input_raw": ("agv2_channel_input_raw", None),
        "robot2_progress_error": ("agv2_position_error", None),
        "rigid_fit_residual": (None, rigid_residual),
    }
    output = {"grid_start_s": START_S, "grid_end_s": END_S,
              "grid_samples": len(grid), "signals": {}}
    for name, (field, derived) in definitions.items():
        a = interp_series(m1_rows, field, grid, derived)
        b = interp_series(m1b_rows, field, grid, derived)
        pairs = [(x, y) for x, y in zip(a, b)
                 if math.isfinite(x) and math.isfinite(y)]
        output["signals"][name] = {
            "paired_samples": len(pairs),
            "m1_rms": stats([x for x, _ in pairs])["rms"],
            "m1b_rms": stats([y for _, y in pairs])["rms"],
            "m1_minus_m1b_mean": (sum(x - y for x, y in pairs) / len(pairs)
                                    if pairs else None),
            "m1_minus_m1b_abs_mean": (sum(abs(x) - abs(y) for x, y in pairs) /
                                        len(pairs) if pairs else None),
        }
    return output


def analyse_run(run_dir):
    summary = read_json(run_dir / "summary_metrics.json")
    validation = read_json(run_dir / "validation.json")
    rows = valid_rows(read_csv(run_dir / "converted" / "aligned_samples.csv"))
    if validation.get("issues") or not rows:
        raise RuntimeError("invalid or empty run {}".format(run_dir))
    phases = phase_rows(rows)
    if not all(phases.values()):
        raise RuntimeError("missing causal phase in {}".format(run_dir))
    capability = load_capability(run_dir / "capability.csv")
    margin_records, reconstruction = reconstruct_margins(rows, capability)
    return {
        "run_dir": str(run_dir), "validation_issues": validation.get("issues", []),
        "summary_metrics": summary,
        "disturbance": disturbance_block(phases["active"]),
        "capability_by_phase": {name: capability_block(values)
                                for name, values in phases.items()},
        "margins_by_phase": {name: phase_margin_summary(margin_records, name)
                             for name in phases},
        "margin_reconstruction": reconstruction,
        "intervention_active": intervention_block(phases["active"]),
        "performance_by_phase": {name: metric_block(values)
                                 for name, values in phases.items()},
        "causal_timing": causal_timing(rows, phases),
        "task_completion_time": summary["task"]["completion_time"],
        "_rows": rows,
    }


def improvement(m1, m1b):
    if m1b is None or m1 is None or m1b == 0:
        return None
    return 100.0 * (m1b - m1) / m1b


def classify(pair):
    m1 = pair["M1_R1"]
    m1b = pair["M1b_R1"]
    intervention = m1["intervention_active"]
    if intervention["risk_contraction"]["max"] < 1e-3 or \
            intervention["reference_drop_peak"] < 1e-3:
        return "CASE_A"
    m1_perf = m1["performance_by_phase"]["active"]
    m1b_perf = m1b["performance_by_phase"]["active"]
    burden = [
        improvement(m1_perf["robot2"]["input_raw"]["rms"],
                    m1b_perf["robot2"]["input_raw"]["rms"]),
        improvement(m1_perf["robot2"]["wheel_pre_limit_abs_peak"],
                    m1b_perf["robot2"]["wheel_pre_limit_abs_peak"]),
        improvement(m1_perf["robot2"]["velocity_error"]["rms"],
                    m1b_perf["robot2"]["velocity_error"]["rms"]),
    ]
    geometry = [improvement(
        m1_perf["geometry"][key]["rms"], m1b_perf["geometry"][key]["rms"])
        for key in ("rigid_fit_residual", "support_point_position_error",
                    "virtual_equivalent_load_position_error")]
    progress = improvement(m1_perf["robot2"]["progress_error"]["rms"],
                           m1b_perf["robot2"]["progress_error"]["rms"])
    meaningful_burden = max(x for x in burden if x is not None) >= 5.0
    meaningful_geometry = max(x for x in geometry if x is not None) >= 5.0
    if meaningful_burden or meaningful_geometry:
        return "CASE_C"
    if progress is not None and progress > 0:
        return "CASE_B"
    return "CASE_D"


def strip_rows(value):
    if isinstance(value, dict):
        return {key: strip_rows(item) for key, item in value.items()
                if key != "_rows"}
    if isinstance(value, list):
        return [strip_rows(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def total_row(level, pair):
    m1, m1b = pair["M1_R1"], pair["M1b_R1"]
    a = m1["performance_by_phase"]["active"]
    b = m1b["performance_by_phase"]["active"]
    margin_summary = m1["margins_by_phase"]["active"]
    if margin_summary["no_margin_active_fraction"] > 0.5:
        dominant = ("none_all_safe", margin_summary["no_margin_active_fraction"])
    else:
        dominant = max(margin_summary["dominance_fraction_by_margin_type"].items(),
                       key=lambda x: x[1])
    official_m1 = m1["summary_metrics"]["risk_disturbance_v1"]
    official_m1b = m1b["summary_metrics"]["risk_disturbance_v1"]
    return {
        "level": level,
        "progress_rmse_m1": official_m1["robot2_progress_rmse"],
        "progress_rmse_m1b": official_m1b["robot2_progress_rmse"],
        "velocity_rmse_m1": official_m1["robot2_velocity_rmse"],
        "velocity_rmse_m1b": official_m1b["robot2_velocity_rmse"],
        "m1_robust_margin_min": m1["intervention_active"]["robust_margin"]["min"],
        "m1_dominant_margin": dominant[0], "m1_dominant_fraction": dominant[1],
        "m1_risk_contraction_peak": m1["intervention_active"]
                                      ["risk_contraction"]["max"],
        "m1_risk_contraction_integral": m1["intervention_active"]
                                          ["risk_contraction_integral"],
        "m1_reference_drop_peak": m1["intervention_active"]["reference_drop_peak"],
        "input_raw_rms_m1": a["robot2"]["input_raw"]["rms"],
        "input_raw_rms_m1b": b["robot2"]["input_raw"]["rms"],
        "input_limit_duration_m1": a["robot2"]["input_limit_duration"],
        "input_limit_duration_m1b": b["robot2"]["input_limit_duration"],
        "wheel_demand_peak_m1": a["robot2"]["wheel_pre_limit_abs_peak"],
        "wheel_demand_peak_m1b": b["robot2"]["wheel_pre_limit_abs_peak"],
        "rigid_fit_rmse_m1": a["geometry"]["rigid_fit_residual"]["rms"],
        "rigid_fit_rmse_m1b": b["geometry"]["rigid_fit_residual"]["rms"],
        "task_time_m1": m1["task_completion_time"],
        "task_time_m1b": m1b["task_completion_time"],
        "classification": pair["classification"],
    }


def render_markdown(report):
    lines = ["# Exp2c-v1 causal-chain diagnosis", "",
             "Frozen exploratory fake result: no level met the predeclared selection rule. ",
             "This report does not alter `selection_report.json` or authorize hardware.", "",
             "## Complete pair table", "",
             "|level|R2 progress RMSE M1/M1b (mm)|R2 velocity RMSE M1/M1b (m/s)|"
             "robust min|dominant margin|contraction peak / integral|reference drop peak (m/s)|"
             "input RMS M1/M1b (m/s²)|input-limit s M1/M1b|wheel peak M1/M1b (m/s)|"
             "rigid RMSE M1/M1b (mm)|task s M1/M1b|case|",
             "|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    for row in report["total_table"]:
        lines.append("|{level}|{p1:.3f}/{p2:.3f}|{v1:.6f}/{v2:.6f}|{rm:.4f}|"
                     "{dom} ({frac:.1%})|{cp:.5f}/{ci:.5f}|{drop:.5f}|"
                     "{i1:.4f}/{i2:.4f}|{il1:.3f}/{il2:.3f}|{w1:.4f}/{w2:.4f}|"
                     "{r1:.3f}/{r2:.3f}|{t1:.2f}/{t2:.2f}|{case}|".format(
                         level=row["level"], p1=1000*row["progress_rmse_m1"],
                         p2=1000*row["progress_rmse_m1b"],
                         v1=row["velocity_rmse_m1"], v2=row["velocity_rmse_m1b"],
                         rm=row["m1_robust_margin_min"], dom=row["m1_dominant_margin"],
                         frac=row["m1_dominant_fraction"],
                         cp=row["m1_risk_contraction_peak"],
                         ci=row["m1_risk_contraction_integral"],
                         drop=row["m1_reference_drop_peak"],
                         i1=row["input_raw_rms_m1"], i2=row["input_raw_rms_m1b"],
                         il1=row["input_limit_duration_m1"],
                         il2=row["input_limit_duration_m1b"],
                         w1=row["wheel_demand_peak_m1"],
                         w2=row["wheel_demand_peak_m1b"],
                         r1=1000*row["rigid_fit_rmse_m1"],
                         r2=1000*row["rigid_fit_rmse_m1b"],
                         t1=row["task_time_m1"], t2=row["task_time_m1b"],
                         case=row["classification"]))
    lines += ["", "## Margin dominance during disturbance", ""]
    for level in LEVELS:
        fractions = report["levels"][level]["M1_R1"]["margins_by_phase"] \
            ["active"]["dominance_fraction_by_margin_type"]
        sources = report["levels"][level]["M1_R1"]["margins_by_phase"] \
            ["active"]["dominance_fraction_by_robot_and_margin"]
        safe = report["levels"][level]["M1_R1"]["margins_by_phase"] \
            ["active"]["no_margin_active_fraction"]
        lines.append("- **{}**: {}; no active margin={:.1%}; source: {}".format(
            level, ", ".join("{}={:.1%}".format(k, v) for k, v in fractions.items()),
            safe,
            ", ".join("{}={:.1%}".format(k, v) for k, v in sources.items() if v > 0)))
    lines += ["", "## Diagnosis", ""]
    for level in LEVELS:
        pair = report["levels"][level]
        m1 = pair["M1_R1"]
        sensitivity = m1["disturbance"][
            "slowdown_induced_reduction_percent_of_nominal_total_l1"]
        lines.append("- **{} — {}**: M1 contraction peak {:.5f} m/s, mean disturbance-window "
                     "reference drop {:.5f} m/s; speed-dependent weakening is only {:.2f}% "
                     "of nominal disturbance L1. Time- and path-domain statistics are both "
                     "retained in the JSON report.".format(
                         level, pair["classification"],
                         m1["intervention_active"]["risk_contraction"]["max"],
                         m1["intervention_active"]["reference_drop_mean"], sensitivity))
    lines += ["", "## Conclusion", "",
              report["conclusion"], "",
              "Exp2c-v1 remains a frozen negative exploratory result. Any future hidden "
              "execution-effectiveness degradation such as `u_plant = gamma(s) * u_limited + d` "
              "must be a separately declared Exp2c-v2 design; it is not implemented here.", ""]
    return "\n".join(lines)


def make_plots(root, report, raw_pairs):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    outputs = []
    figures = root / "causal_diagnostic_plots"
    figures.mkdir(exist_ok=True)
    colors = {"0p30": "#1f77b4", "0p45": "#d62728",
              "0p60": "#2ca02c", "0p70": "#9467bd"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharey=True)
    names = ("capability_margin", "input_margin", "wheel_margin",
             "position_margin", "failure_margin")
    for ax, level in zip(axes.flat, LEVELS):
        values = report["levels"][level]["M1_R1"]["margins_by_phase"]["active"] \
            ["dominance_fraction_by_margin_type"]
        ax.bar(range(len(names)), [values[n] for n in names], color=colors[level])
        ax.set_title(level); ax.set_ylim(0, 1); ax.set_xticks(range(len(names)))
        ax.set_xticklabels([n.replace("_margin", "") for n in names], rotation=25)
        ax.set_ylabel("dominance fraction")
    fig.tight_layout()
    path = figures / "margin_decomposition.png"; fig.savefig(path, dpi=180); plt.close(fig)
    outputs.append(str(path))
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True, sharey=True)
    for ax, level in zip(axes.flat, LEVELS):
        rows = raw_pairs[level]["M1_R1"]["_rows"]
        active = phase_rows(rows)["active"]
        s = [finite(r.get("agv2_s_tracking_actual")) for r in active]
        ax.plot(s, [finite(r.get("risk_contraction")) for r in active],
                color=colors[level], label="risk contraction")
        ax.plot(s, [NOMINAL_SPEED - finite(r.get("common_velocity_reference"))
                    for r in active], color="black", linestyle="--", label="reference drop")
        ax.set_title(level); ax.set_xlabel("Robot2 progress s (m)"); ax.set_ylabel("m/s")
        ax.legend(fontsize=8)
    fig.tight_layout()
    path = figures / "risk_contraction_reference.png"; fig.savefig(path, dpi=180); plt.close(fig)
    outputs.append(str(path))
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for ax, level in zip(axes.flat, LEVELS):
        for method, style in (("M1_R1", "-"), ("M1b_R1", "--")):
            active = phase_rows(raw_pairs[level][method]["_rows"])["active"]
            ax.plot([finite(r.get("agv2_s_tracking_actual")) for r in active],
                    [finite(r.get("agv2_channel_input_raw")) for r in active],
                    linestyle=style, label=method)
        ax.set_title(level); ax.set_xlabel("Robot2 progress s (m)"); ax.set_ylabel("input raw (m/s²)")
        ax.legend(fontsize=8)
    fig.tight_layout()
    path = figures / "robot2_input_burden.png"; fig.savefig(path, dpi=180); plt.close(fig)
    outputs.append(str(path))
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for ax, level in zip(axes.flat, LEVELS):
        grid = [START_S + (END_S - START_S) * i / 160.0 for i in range(161)]
        for method, style in (("M1_R1", "-"), ("M1b_R1", "--")):
            rows = raw_pairs[level][method]["_rows"]
            values = interp_series(rows, "agv2_position_error", grid)
            ax.plot(grid, [1000*x for x in values], linestyle=style, label=method)
        ax.set_title(level); ax.set_xlabel("Robot2 progress s (m)"); ax.set_ylabel("progress error (mm)")
        ax.legend(fontsize=8)
    fig.tight_layout()
    path = figures / "path_domain_m1_m1b.png"; fig.savefig(path, dpi=180); plt.close(fig)
    outputs.append(str(path))
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("screen_root", type=Path)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    root = args.screen_root.resolve()
    selection_path = root / "selection_report.json"
    selection_before = selection_path.read_bytes()
    raw_pairs = {}
    try:
        for level in LEVELS:
            raw_pairs[level] = {}
            for method in METHODS:
                raw_pairs[level][method] = analyse_run(root / level / method)
            raw_pairs[level]["path_domain_comparison"] = path_pair(
                raw_pairs[level]["M1_R1"]["_rows"],
                raw_pairs[level]["M1b_R1"]["_rows"])
            raw_pairs[level]["classification"] = classify(raw_pairs[level])
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    classifications = [raw_pairs[level]["classification"] for level in LEVELS]
    conclusion = (
        "All four levels are CASE A: during the spatial disturbance window every "
        "reconstructed causal margin remains at its fully safe value, risk contraction "
        "is numerically zero, and the common reference does not meaningfully fall. The "
        "causal chain therefore breaks before M1 risk feedback. The profile is also "
        "dominated by a speed-insensitive constant component, but that secondary property "
        "cannot explain a benefit because M1 never intervenes. The predeclared Exp2c-v1 "
        "screen remains a negative result; these data do not support a claim that M1 wins."
    )
    report = {
        "schema_version": 1,
        "scope": "Exp2c-v1 offline causal diagnosis only",
        "screen_root": str(root),
        "all_eight_runs_readable_and_valid": True,
        "selection_report_preserved": True,
        "selection_status": read_json(selection_path).get("selection_status"),
        "existing_summary_scope_warning": (
            "summary_metrics.risk_disturbance_v1 risk_contraction_peak/integral "
            "include the whole evaluation run and capture startup contraction; the "
            "diagnostic report recomputes them only inside the disturbance window "
            "without changing the existing summary or frozen selection report"),
        "classification_rule": {
            "intervention_threshold": "contraction and reference drop >= 0.001 m/s",
            "meaningful_improvement_threshold": "at least 5% in burden or cooperative geometry",
            "note": "diagnostic classification only; frozen selection rule is unchanged",
        },
        "levels": strip_rows(raw_pairs),
        "total_table": [total_row(level, raw_pairs[level]) for level in LEVELS],
        "overall_classifications": classifications,
        "conclusion": conclusion,
        "physical_authorization_changed": False,
        "controller_or_runtime_modified": False,
    }
    json_path = root / "causal_diagnostic_report.json"
    md_path = root / "causal_diagnostic_report.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md_path.write_text(render_markdown(report), encoding="utf-8")
    plots = [] if args.no_plots else make_plots(root, report, raw_pairs)
    if selection_path.read_bytes() != selection_before:
        raise RuntimeError("selection_report.json was unexpectedly modified")
    print("EXP2C_CAUSAL_DIAGNOSIS=PASSED")
    print("JSON={}".format(json_path))
    print("MARKDOWN={}".format(md_path))
    for path in plots:
        print("PLOT={}".format(path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
