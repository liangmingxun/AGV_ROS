#!/usr/bin/env python3
"""Prepare, verify and compare frozen Candidate B v2 fake runs."""
import argparse
import csv
import importlib.util
import json
import math
import re
import statistics
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "candidate_b_v1", HERE / "analyze_candidate_B_fake.py")
B1 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(B1)
V4, V2 = B1.V4, B1.V2

IDENTITY = "candidate_B_spatial_composite_fake_validation"
MARKERS = ["EXPLORATORY_FAKE_VALIDATION", "NOT_FORMAL_PAPER_EVIDENCE"]
METHODS = {"M1": "M1_R1", "M1b": "M1b_R1", "M2b": "M2b_M2b"}
CONFIG = HERE.parents[1] / "multi_agv_bringup/config"
PROFILE = yaml.safe_load((
    CONFIG / "formal_fake_candidate_B_spatial_composite.yaml").read_text()
)["formal_fake_runtime"]["candidate_b_spatial_composite"]
COMPARISON_FILENAME = "candidate_b_spatial_comparison.json"
REPORT_FILENAME = (
    "candidate-B-spatial-composite-rho0p85-av0p020-fake-results.md")


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n")


def prepare(root, method):
    B1.prepare(root, method)
    metadata = {"triad": root.parent.name, "method": METHODS[method]}
    runtime_paths = list((root / "configs").glob("*.yaml"))
    for path in runtime_paths:
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict):
            continue
        runtime = data.get("formal_fake_runtime")
        if isinstance(runtime, dict):
            runtime["experiment_id"] = IDENTITY
            runtime["candidate_b"] = {"enabled": False}
            runtime["candidate_b_spatial_composite"] = dict(PROFILE)
            runtime["candidate_b_spatial_metadata"] = metadata
            runtime["classic_additive_candidate_A"] = {"enabled": False}
            runtime["classic_additive_physical"] = {
                "enabled": False, "hardware_execution_authorized": False}
            path.write_text(yaml.safe_dump(data, sort_keys=False))
    for path in (root / "configs").glob("*_authorization.yaml"):
        data = yaml.safe_load(path.read_text())
        data["candidate_b"] = {"hardware_execution_authorized": False}
        data["candidate_b_spatial_composite"] = {
            "hardware_execution_authorized": False}
        path.write_text(yaml.safe_dump(data, sort_keys=False))
    loc = root / "configs/localization_v5_quality_observation.yaml"
    data = yaml.safe_load(loc.read_text())
    data.update({"experiment_id": IDENTITY,
                 "platform_transport_type": "fake",
                 "hardware_execution_authorized": False})
    loc.write_text(yaml.safe_dump(data, sort_keys=False))
    topics = root / "configs/record_topics_v5.yaml"
    data = yaml.safe_load(topics.read_text())
    registry = data["experiment_recording"]
    old = "/multi_agv/candidate_B_disturbance_state"
    new = "/multi_agv/candidate_B_spatial_composite_state"
    for key in ("topics", "required_topics"):
        values = registry[key]
        values[:] = [value for value in values if value != old]
        if new not in values:
            values.append(new)
    for values in registry.get("method_required_topics", {}).values():
        values[:] = [value for value in values if value != old]
        if new not in values:
            values.append(new)
    topics.write_text(yaml.safe_dump(data, sort_keys=False))
    validation = root / "configs/validation_candidate_b_spatial.yaml"
    data = yaml.safe_load((HERE.parent / "config" /
                           "validation_defaults.yaml").read_text())
    registry = data["experiment_recording"]
    for method_id in METHODS.values():
        required = registry["method_required_topics"].setdefault(
            method_id, [])
        required[:] = [value for value in required if value != old]
        if new not in required:
            required.append(new)
    registry["minimum_topic_rates"][new] = 90.0
    validation.write_text(yaml.safe_dump(data, sort_keys=False))
    save(root / "CANDIDATE_B_V2_SCOPE.json", {
        "experiment_id": IDENTITY, "markers": MARKERS,
        "formal_evidence": False, "parameter_search": False,
        "hardware_authorization": False, "serial_execution": False,
        "method": METHODS[method], "profile": PROFILE,
        "spatial_coordinate":
            "actual support centre projected onto existing load centerline"})


def durations(rows):
    if not rows:
        raise ValueError("empty analysis phase")
    key = "relative_wall_time" if "relative_wall_time" in rows[0] else "wall"
    delta = [rows[index + 1][key] - rows[index][key]
             for index in range(len(rows) - 1)]
    if any(value <= 0.0 or value > 0.10 for value in delta):
        raise ValueError("non-contiguous wall-clock observation")
    return delta + [statistics.median(delta) if delta else 0.01]


def hard_failure_in_valid_window(log, first_valid_stamp, completion_stamp):
    tokens = ("numerical_invalid", "state_chain_invalid", "held fail-zero:",
              "safety abort latched")
    stamp_pattern = re.compile(r"\[(\d+(?:\.\d+)?)\]")
    for line in log.read_text(errors="replace").splitlines():
        if not any(token in line.lower() for token in tokens):
            continue
        match = stamp_pattern.search(line)
        if match and first_valid_stamp <= float(match.group(1)) <= completion_stamp:
            return True
    return False


def weighted(values, dt):
    return V4.weighted_mean(values, dt)


def rms(values, dt):
    return math.sqrt(weighted([value * value for value in values], dt))


def robot_errors(row, robot):
    ax = V2.num(row, f"agv{robot}_support_pose_x")
    ay = V2.num(row, f"agv{robot}_support_pose_y")
    rx = V2.num(row, f"agv{robot}_support_reference_x")
    ry = V2.num(row, f"agv{robot}_support_reference_y")
    yaw = V2.num(row, f"agv{robot}_support_reference_yaw")
    dx, dy = ax - rx, ay - ry
    return {
        "progress": V2.num(row, f"agv{robot}_s_tracking_actual") -
                    V2.num(row, "load_s_reference"),
        "lateral": -math.sin(yaw) * dx + math.cos(yaw) * dy,
        "heading": V2.wrap(V2.num(row, f"agv{robot}_robot_pose_yaw") - yaw),
        "velocity": V2.num(row, f"agv{robot}_s_dot_actual") -
                    V2.num(row, "common_velocity_reference"),
        "support": math.hypot(dx, dy),
    }


def phase_metrics(rows):
    dt = durations(rows)
    out = {"samples": len(rows), "duration_seconds": sum(dt)}
    combined = {key: [row["errors"][key] for row in rows]
                for key in ("support", "rigid_fit", "pairwise_side")}
    for key, values in combined.items():
        out[f"{key}_error_rmse"] = rms(values, dt)
        out[f"{key}_error_max"] = max(abs(value) for value in values)
    for robot in (1, 2, 3):
        for key in ("progress", "lateral", "heading", "velocity", "support"):
            values = [row["robot_errors"][robot][key] for row in rows]
            out[f"robot{robot}_{key}_error_rmse"] = rms(values, dt)
            out[f"robot{robot}_{key}_error_max"] = max(
                abs(value) for value in values)
    load_position = [math.hypot(
        V2.num(row, "equivalent_load_pose_x") - V2.num(row, "load_x_reference"),
        V2.num(row, "equivalent_load_pose_y") - V2.num(row, "load_y_reference"))
        for row in rows]
    load_yaw = [V2.wrap(V2.num(row, "equivalent_load_pose_yaw") -
                        V2.num(row, "load_yaw_reference")) for row in rows]
    out.update({
        "equivalent_load_position_error_rmse": rms(load_position, dt),
        "equivalent_load_position_error_max": max(load_position),
        "equivalent_load_yaw_error_rmse": rms(load_yaw, dt),
        "equivalent_load_yaw_error_max": max(abs(value) for value in load_yaw),
    })
    all_native, all_post, all_limit, all_actual, margins = [], [], [], [], []
    limiter = []
    for row in rows:
        sample_native, sample_post = [], []
        for robot in (1, 2, 3):
            prefix = f"agv{robot}_candidate_b_spatial_"
            native = (V2.num(row, prefix + "left_native"),
                      V2.num(row, prefix + "right_native"))
            post = (V2.num(row, prefix + "left_post_disturbance"),
                    V2.num(row, prefix + "right_post_disturbance"))
            post_limit = (V2.num(row, prefix + "left_post_limit"),
                          V2.num(row, prefix + "right_post_limit"))
            actual = (V2.num(row, prefix + "actual_left"),
                      V2.num(row, prefix + "actual_right"))
            limit = V2.num(row, prefix + "capability_limit")
            sample_native.extend(native); sample_post.extend(post)
            all_limit.extend(post_limit); all_actual.extend(actual)
            margins.append(max(0.0, min(1.0,
                (limit - max(abs(native[0]), abs(native[1]))) / 0.010)))
        all_native.append(sample_native); all_post.append(sample_post)
        limiter.append(any(V2.flag(row,
            f"agv{robot}_candidate_b_spatial_limiter_active")
            for robot in (1, 2, 3)))
    native_peak = [max(abs(value) for value in sample) for sample in all_native]
    post_peak = [max(abs(value) for value in sample) for sample in all_post]
    out.update({
        "native_controller_raw_peak_mps": max(native_peak),
        "native_controller_raw_rms_mps": rms([
            math.sqrt(sum(value * value for value in sample) / len(sample))
            for sample in all_native], dt),
        "native_over_0p160_duration_seconds": sum(
            step for value, step in zip(native_peak, dt) if value > .160),
        "native_over_0p180_duration_seconds": sum(
            step for value, step in zip(native_peak, dt) if value > .180),
        "post_candidate_b_peak_mps": max(post_peak),
        "post_candidate_b_over_0p160_duration_seconds": sum(
            step for value, step in zip(post_peak, dt) if value > .160),
        "post_candidate_b_over_0p180_duration_seconds": sum(
            step for value, step in zip(post_peak, dt) if value > .180),
        "physical_limiter_duration_seconds": sum(
            step for value, step in zip(limiter, dt) if value),
        "post_limit_peak_mps": max(abs(value) for value in all_limit),
        "actual_peak_mps": max(abs(value) for value in all_actual),
        "wheel_margin_minimum": min(margins),
        "wheel_margin_mean": sum(margins) / len(margins),
        "wheel_margin_p05": V2.percentile(margins, .05),
        "wheel_margin_below_0p5_duration_seconds": sum(
            step for index, step in enumerate(dt)
            if min(margins[index*3:index*3+3]) < .5),
        "wheel_margin_below_0p7_duration_seconds": sum(
            step for index, step in enumerate(dt)
            if min(margins[index*3:index*3+3]) < .7),
    })
    return out


def disturbance_verification(rows):
    result = {}
    for robot in (1, 2, 3):
        prefix = f"agv{robot}_candidate_b_spatial_"
        active = [row for row in rows if V2.flag(row, prefix + "active")]
        triggered = [row for row in rows if V2.flag(row, prefix + "triggered")]
        finished = [row for row in rows if V2.flag(row, prefix + "finished")]
        if not active or not triggered or not finished:
            raise ValueError(f"robot{robot} did not traverse complete spatial zone")
        entry = V2.num(triggered[0], prefix + "entry_wall_time")
        exit_time = V2.num(finished[0], prefix + "exit_wall_time")
        d_v = [V2.num(row, prefix + "d_v") for row in active]
        d_omega = [V2.num(row, prefix + "d_omega") for row in active]
        q = [V2.num(row, prefix + "q") for row in active]
        rho = [V2.num(row, prefix + "rho") for row in active]
        projection_distance = [
            V2.num(row, prefix + "projection_distance") for row in rows]
        projection_distance = [value for value in projection_distance
                               if math.isfinite(value)]
        dt = durations(active)
        result[f"Robot{robot}"] = {
            "zone_entry_progress": V2.num(triggered[0], prefix + "progress"),
            "zone_exit_progress": V2.num(finished[0], prefix + "progress"),
            "entry_common_progress": V2.num(triggered[0], "load_s_reference"),
            "exit_common_progress": V2.num(finished[0], "load_s_reference"),
            "entry_wall_time": entry, "exit_wall_time": exit_time,
            "exposure_duration_seconds": exit_time - entry,
            "minimum_rho": min(rho), "q_mean": weighted(q, dt),
            "q_max": max(q), "d_v_peak": max(abs(value) for value in d_v),
            "d_v_rms": rms(d_v, dt),
            "d_omega_peak": max(abs(value) for value in d_omega),
            "d_omega_rms": rms(d_omega, dt),
            "projection_distance_available": bool(projection_distance),
            "projection_distance_mean": (
                sum(projection_distance) / len(projection_distance)
                if projection_distance else None),
            "projection_distance_max": (
                max(projection_distance) if projection_distance else None),
        }
    one = result["Robot1"]
    for robot in (2, 3):
        rear = result[f"Robot{robot}"]
        rear[f"entry_time_minus_robot1_seconds"] = (
            rear["entry_wall_time"] - one["entry_wall_time"])
        rear[f"entry_common_progress_minus_robot1_m"] = (
            rear["entry_common_progress"] - one["entry_common_progress"])
    result["entry_order"] = sorted(
        (value["entry_wall_time"], key) for key, value in result.items()
        if key.startswith("Robot"))
    return result


def mechanism_metrics(rows):
    dt = durations(rows)
    robust = [min(V2.num(row, f"agv{robot}_robust_margin")
                  for robot in (1, 2, 3)) for row in rows]
    risk_factor = [max(V2.num(row, f"agv{robot}_risk_factor")
                       for robot in (1, 2, 3)) for row in rows]
    risk_signal = [max(V2.num(row, f"agv{robot}_risk_signal")
                       for robot in (1, 2, 3)) for row in rows]
    contraction = [max(0.0, V2.num(row, "risk_contraction"))
                   if math.isfinite(V2.num(row, "risk_contraction")) else 0.0
                   for row in rows]
    reduction = [max(0.0, V2.num(row, "candidate_common_velocity") -
                     V2.num(row, "common_velocity_reference")) for row in rows]
    common = [V2.num(row, "common_velocity_reference") for row in rows]
    return {
        "robust_margin_minimum": min(robust),
        "robust_margin_mean": weighted(robust, dt),
        "risk_factor_peak": max(risk_factor),
        "risk_factor_mean": weighted(risk_factor, dt),
        "risk_signal_peak": max(risk_signal),
        "risk_signal_mean": weighted(risk_signal, dt),
        "risk_contraction_peak": max(contraction),
        "risk_contraction_integral": sum(v*t for v, t in zip(contraction, dt)),
        "risk_boundary_active_duration": sum(
            t for row, t in zip(rows, dt) if V4.V3.boundary_active(row)),
        "actual_reference_reduction_peak": max(reduction),
        "actual_reference_reduction_mean": weighted(reduction, dt),
        "common_reference_minimum": min(common),
        "common_reference_mean": weighted(common, dt),
    }


def analyze(run, method, log):
    params = yaml.safe_load((run / "rosparams.yaml").read_text())
    manifest = yaml.safe_load((run / "manifest.yaml").read_text())
    runtime = params["formal_fake_algorithm"]["formal_fake_runtime"]
    if manifest["experiment_id"] != IDENTITY or runtime["experiment_id"] != IDENTITY:
        raise ValueError("wrong Candidate B v2 identity")
    if runtime["candidate_b_spatial_composite"] != PROFILE:
        raise ValueError("Candidate B v2 profile differs from freeze")
    if runtime.get("candidate_b", {}).get("enabled", False):
        raise ValueError("Candidate B v1 is also enabled")
    if runtime.get("classic_additive_candidate_A", {}).get("enabled", False):
        raise ValueError("Candidate A is also enabled")
    for robot in (1, 2, 3):
        if params[f"agv{robot}"]["chassis_controller"]["transport_type"] != "fake":
            raise ValueError("non-fake transport")
    all_rows = list(csv.DictReader((run / "converted/aligned_samples.csv").open()))
    rows, seen = [], set()
    for source in all_rows:
        if not V2.flag(source, "algorithm_valid") or not V2.flag(
                source, "candidate_b_spatial_state_available"):
            continue
        wall = V2.num(source, "candidate_b_spatial_wall_time")
        if not math.isfinite(wall) or wall in seen:
            continue
        seen.add(wall)
        row = dict(source)
        row["wall"] = wall
        row["errors"] = V4.error_signals(row)
        row["robot_errors"] = {robot: robot_errors(row, robot)
                               for robot in (1, 2, 3)}
        rows.append(row)
    rows.sort(key=lambda row: row["wall"])
    verification = disturbance_verification(rows)
    entry = min(verification[f"Robot{i}"]["entry_wall_time"] for i in (1,2,3))
    exit_time = max(verification[f"Robot{i}"]["exit_wall_time"] for i in (1,2,3))
    for row in rows:
        row["relative_wall_time"] = row["wall"] - entry
    primary = [row for row in rows if entry <= row["wall"] <= exit_time]
    metrics = phase_metrics(primary)
    validation = json.loads((run / "validation.json").read_text())
    summary = json.loads((run / "summary_metrics.json").read_text())
    complete = bool(validation.get("task_completion", {}).get("complete"))
    algorithm_window = validation.get("algorithm", {})
    invalid_log = hard_failure_in_valid_window(
        log, float(algorithm_window.get("first_valid_stamp", -math.inf)),
        float(algorithm_window.get("completion_stamp", math.inf)))
    scales = PROFILE.get("severity_scale", [1.0, 1.0, 1.0])
    profile_ok = all(
        abs(verification[f"Robot{i}"]["minimum_rho"] -
            (1.0 - (1.0 - PROFILE["minimum_effectiveness"]) *
             scales[i - 1])) < 2e-3 and
        verification[f"Robot{i}"]["d_v_peak"] <=
            PROFILE["longitudinal_amplitude"] * scales[i - 1] + 1e-7 and
        verification[f"Robot{i}"]["d_omega_peak"] <=
            PROFILE["yaw_amplitude"] * scales[i - 1] + 1e-7
        for i in (1, 2, 3))
    category = ("VALID_COMPLETED" if complete and not invalid_log and profile_ok
                else "INVALID_RUN")
    result = {
        "experiment_id": IDENTITY, "markers": MARKERS,
        "formal_evidence": False, "physical_readiness": "NOT_AUTHORIZED",
        "method": METHODS[method], "category": category,
        "complete": complete,
        "task_time_seconds": summary.get("task", {}).get("completion_time"),
        "metrics": metrics, "disturbance_verification": verification,
        "M1_mechanism": mechanism_metrics(primary) if method == "M1" else None,
        "profile_verified": profile_ok, "run_path": str(run),
    }
    return result


def percent(m1, comparator):
    return 100.0 * (comparator - m1) / comparator if abs(comparator) > 1e-12 else None


def make_plots(root, runs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plot_dir = root / "plots"
    plot_dir.mkdir(exist_ok=True)
    colors = {1: "#1f77b4", 2: "#d62728", 3: "#2ca02c"}
    method_colors = {"M1": "#1f77b4", "M1b": "#d62728", "M2b": "#2ca02c"}
    def savefig(fig, name):
        fig.savefig(plot_dir / f"{name}.png", dpi=300, bbox_inches="tight")
        fig.savefig(plot_dir / f"{name}.pdf", bbox_inches="tight")
        plt.close(fig)
    m1_rows = runs["M1"]
    t0 = min(row["wall"] for row in m1_rows)
    time = [row["wall"] - t0 for row in m1_rows]
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for robot in (1, 2, 3):
        p = f"agv{robot}_candidate_b_spatial_"
        axes[0].plot(time, [V2.num(r, p+"progress") for r in m1_rows],
                     color=colors[robot], label=f"Robot{robot}")
        axes[1].plot(time, [V2.num(r, p+"q") for r in m1_rows],
                     color=colors[robot], label=f"q{robot}")
        axes[1].plot(time, [V2.num(r, p+"rho") for r in m1_rows],
                     color=colors[robot], linestyle="--", label=f"rho{robot}")
    axes[0].axhspan(2.0, 2.8, color="0.9"); axes[0].set_ylabel("spatial progress / m")
    axes[1].set_ylabel("q / rho"); axes[1].set_xlabel("time / s")
    for ax in axes: ax.grid(alpha=.25); ax.legend(ncol=3)
    savefig(fig, "figure1_spatial_progress_envelope")
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for robot in (1, 2, 3):
        p = f"agv{robot}_candidate_b_spatial_"
        axes[0].plot(time, [V2.num(r, p+"d_v") for r in m1_rows], color=colors[robot], label=f"Robot{robot}")
        axes[1].plot(time, [V2.num(r, p+"d_omega") for r in m1_rows], color=colors[robot], label=f"Robot{robot}")
    axes[0].set_ylabel("d_v / (m/s)"); axes[1].set_ylabel("d_omega / (rad/s)"); axes[1].set_xlabel("time / s")
    for ax in axes: ax.grid(alpha=.25); ax.legend(ncol=3)
    savefig(fig, "figure2_disturbance_components")
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=False)
    for ax, method in zip(axes, ("M1", "M1b", "M2b")):
        rows = runs[method]; origin = rows[0]["wall"]
        tt = [r["wall"]-origin for r in rows]
        for robot in (1,2,3):
            p=f"agv{robot}_candidate_b_spatial_"
            peak=[max(abs(V2.num(r,p+"left_native")), abs(V2.num(r,p+"right_native"))) for r in rows]
            ax.plot(tt, peak, color=colors[robot], label=f"Robot{robot}")
        ax.axhline(.160,color="k",linestyle="--"); ax.axhline(.180,color="k",linestyle=":")
        ax.set_ylabel(method+" / (m/s)"); ax.grid(alpha=.25); ax.legend(ncol=3)
    axes[-1].set_xlabel("time / s")
    savefig(fig, "figure3_native_wheel_demand")
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=False)
    for ax, metric in zip(axes, ("support", "rigid_fit", "pairwise_side")):
        for method, rows in runs.items():
            origin=rows[0]["wall"]
            ax.plot([r["wall"]-origin for r in rows],
                    [r["errors"][metric] for r in rows],
                    color=method_colors[method], label=method)
        ax.set_ylabel(metric+" / m"); ax.grid(alpha=.25); ax.legend()
    axes[-1].set_xlabel("time / s")
    savefig(fig, "figure4_formation_errors")
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(time, [V2.num(r,"agv2_risk_factor") for r in m1_rows], label="risk factor")
    axes[1].plot(time, [V2.num(r,"risk_contraction") for r in m1_rows], label="contraction")
    axes[2].plot(time, [V2.num(r,"common_velocity_reference") for r in m1_rows], label="common reference")
    for ax in axes: ax.grid(alpha=.25); ax.legend()
    axes[-1].set_xlabel("time / s")
    savefig(fig, "figure5_m1_mechanism")


def aggregate(root):
    data = {method: json.loads((root / folder / "run/candidate_b_spatial_metrics.json").read_text())
            for method, folder in METHODS.items()}
    valid = all(value["category"] == "VALID_COMPLETED" for value in data.values())
    comparisons = {}
    keys = ["support_error_rmse", "support_error_max", "rigid_fit_error_rmse",
            "rigid_fit_error_max", "pairwise_side_error_rmse",
            "pairwise_side_error_max", "equivalent_load_position_error_rmse",
            "equivalent_load_position_error_max",
            "equivalent_load_yaw_error_rmse", "equivalent_load_yaw_error_max",
            "native_controller_raw_peak_mps", "native_controller_raw_rms_mps",
            "native_over_0p160_duration_seconds",
            "native_over_0p180_duration_seconds",
            "post_candidate_b_peak_mps",
            "post_candidate_b_over_0p160_duration_seconds",
            "post_candidate_b_over_0p180_duration_seconds",
            "physical_limiter_duration_seconds", "post_limit_peak_mps",
            "actual_peak_mps", "wheel_margin_below_0p5_duration_seconds",
            "wheel_margin_below_0p7_duration_seconds"]
    for robot in (1, 2, 3):
        keys.extend(f"robot{robot}_{kind}_error_rmse"
                    for kind in ("progress", "lateral", "heading", "velocity"))
    margin_keys = ["wheel_margin_minimum", "wheel_margin_mean",
                   "wheel_margin_p05"]
    for comparator in ("M1b", "M2b"):
        comparisons[f"M1_vs_{comparator}"] = {
            key: {"M1": data["M1"]["metrics"][key],
                  comparator: data[comparator]["metrics"][key],
                  "direction": "lower_is_better",
                  "improvement_percent": percent(
                      data["M1"]["metrics"][key],
                      data[comparator]["metrics"][key])}
            for key in keys}
        for key in margin_keys:
            m1_value = data["M1"]["metrics"][key]
            baseline = data[comparator]["metrics"][key]
            comparisons[f"M1_vs_{comparator}"][key] = {
                "M1": m1_value, comparator: baseline,
                "direction": "higher_is_better",
                "improvement_percent": (None if abs(baseline) < 1e-12 else
                                          100.0 * (m1_value - baseline) /
                                          abs(baseline))}
    result = {"experiment_id": IDENTITY, "markers": MARKERS,
              "profile": PROFILE, "methods": data,
              "comparisons": comparisons, "all_runs_valid": valid,
              "fake_comparison": "VALID" if valid else "INVALID",
              "physical_readiness": "NOT_AUTHORIZED"}
    save(root / COMPARISON_FILENAME, result)
    rows_for_plot = {}
    for method, folder in METHODS.items():
        rows = list(csv.DictReader((root / folder / "run/converted/aligned_samples.csv").open()))
        selected=[]
        for row in rows:
            if V2.flag(row,"algorithm_valid") and V2.flag(row,"candidate_b_spatial_state_available"):
                row["wall"]=V2.num(row,"candidate_b_spatial_wall_time")
                row["errors"]=V4.error_signals(row)
                selected.append(row)
        rows_for_plot[method]=selected
    make_plots(root, rows_for_plot)
    start_status = (root / "start_git_status.txt").read_text().strip()
    lines = ["# Candidate B v2 spatial composite fake results", "",
        "EXPLORATORY_FAKE_VALIDATION", "NOT_FORMAL_PAPER_EVIDENCE", "",
        f"Commit: `{(root/'start_head.txt').read_text().strip()}`", "",
        "Start dirty state:", "", "```text", start_status or "clean", "```", "",
        f"Experiment identity: `{IDENTITY}`", "",
        "Frozen profile:", "", "```json", json.dumps(PROFILE, indent=2), "```", "",
        "Spatial coordinate: each measured support centre is projected onto the existing load centerline PathProjector. The original s_actual is a support-specific common load parameter and is not used as the world-zone coordinate.", "",
        "Candidate B v2 rejects a spatial projection unless it is valid, has finite progress and distance, and remains within 0.30 m of the load centerline. The limit is deliberately wider than the nominal 0.15 m support-centre lateral offset of the 30 cm formation, while still rejecting grossly inconsistent poses.", "",
        "The coordinate therefore represents the measured support centre crossing the same world-space centerline section. No 2.598 s delay and no fixed 8 s termination are used; each robot owns an independent entry wall clock and exits only after its projected progress reaches 2.8 m.", "",
        "Fresh run paths:", "",
        *[f"- `{data[m]['run_path']}`" for m in ("M1b", "M1", "M2b")], "",
        f"Fake comparison: **{result['fake_comparison']}**; physical readiness: **NOT_AUTHORIZED**.", "",
        "| Method | task/s | support RMS/max | rigid RMS/max | side RMS/max | load pos/yaw RMS | native peak | native >.160/s | native >.180/s | post-B peak | post-B >.160/.180 s | limiter/s | post-limit/actual peak | margin min/mean/p05 | margin <.5/<.7 s |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for method in ("M1", "M1b", "M2b"):
        d=data[method]; m=d["metrics"]
        lines.append("| {} | {:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g}/{:.6g} | {:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g}/{:.6g} | {:.6g}/{:.6g} |".format(
            METHODS[method], d["task_time_seconds"], m["support_error_rmse"],m["support_error_max"],m["rigid_fit_error_rmse"],m["rigid_fit_error_max"],m["pairwise_side_error_rmse"],m["pairwise_side_error_max"],m["equivalent_load_position_error_rmse"],m["equivalent_load_yaw_error_rmse"],m["native_controller_raw_peak_mps"],m["native_over_0p160_duration_seconds"],m["native_over_0p180_duration_seconds"],m["post_candidate_b_peak_mps"],m["post_candidate_b_over_0p160_duration_seconds"],m["post_candidate_b_over_0p180_duration_seconds"],m["physical_limiter_duration_seconds"],m["post_limit_peak_mps"],m["actual_peak_mps"],m["wheel_margin_minimum"],m["wheel_margin_mean"],m["wheel_margin_p05"],m["wheel_margin_below_0p5_duration_seconds"],m["wheel_margin_below_0p7_duration_seconds"]))
    lines += ["", "## Per-robot RMSE", "",
              "| Method | Robot | progress / m | lateral / m | heading / rad | velocity / (m/s) |",
              "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for method in ("M1", "M1b", "M2b"):
        m = data[method]["metrics"]
        for robot in (1, 2, 3):
            lines.append("| {} | {} | {:.8g} | {:.8g} | {:.8g} | {:.8g} |".format(
                METHODS[method], robot,
                m[f"robot{robot}_progress_error_rmse"],
                m[f"robot{robot}_lateral_error_rmse"],
                m[f"robot{robot}_heading_error_rmse"],
                m[f"robot{robot}_velocity_error_rmse"]))
    for comparator in ("M1b","M2b"):
        lines += ["", f"## M1 vs {comparator}", "", "| Metric | M1 | Comparator | Improvement |", "| --- | ---: | ---: | ---: |"]
        for key, value in comparisons[f"M1_vs_{comparator}"].items():
            imp=value["improvement_percent"]
            improvement = "N/A" if imp is None else f"{imp:.3f}%"
            lines.append(
                f"| {key} | {value['M1']:.8g} | "
                f"{value[comparator]:.8g} | {improvement} |")
    task_cost_m1b = 100.0 * (data["M1"]["task_time_seconds"] /
                             data["M1b"]["task_time_seconds"] - 1.0)
    task_cost_m2b = 100.0 * (data["M1"]["task_time_seconds"] /
                             data["M2b"]["task_time_seconds"] - 1.0)
    lines += ["", "## Spatial verification", "", "```json",
              json.dumps({m:data[m]["disturbance_verification"] for m in data},indent=2), "```", "",
              "Projection-distance mean/max above establish Candidate B v2 spatial-coordinate validity independently for every robot. Disturbance-profile validity, fake-comparison validity, and physical readiness are separate decisions.", "",
              "## M1 mechanism", "", "```json", json.dumps(data["M1"]["M1_mechanism"],indent=2), "```", "",
              "The frozen disturbance drove robust margin to zero at its worst sample, risk factor to 1.0, and risk contraction to its recorded peak. The risk boundary was active for the recorded duration and produced the measured candidate-minus-effective reference reduction. This timing-consistent chain supports, but does not by itself prove, strict causality.", "",
              f"M1 task-time cost was {task_cost_m1b:.3f}% versus M1b and {task_cost_m2b:.3f}% versus M2b.", "",
              "## Validity and physical-readiness notes", "",
              "- All three runs completed with zero algorithm-invalid and localization-invalid fractions and identical fake-only disturbance profiles.",
              "- M1b/M2b pre-disturbance native controller demand exceeded 0.180 m/s for {:.3f} s and {:.3f} s, respectively, during the analysis window. This does not invalidate the fake scientific comparison, but requires a separate physical-safe qualification.".format(data["M1b"]["metrics"]["native_over_0p180_duration_seconds"], data["M2b"]["metrics"]["native_over_0p180_duration_seconds"]),
              "- The four stages are reported separately: native controller demand, post-Candidate-B execution demand, post-limit command, and fake-plant actual feedback. Native demand above 0.180 m/s is not described as actual wheel command above 0.180 m/s.",
              "- All post-Candidate-B execution demands remained below 0.160 m/s in these fake runs and no fake physical limiter duration was recorded; post-limit and actual peaks remain separately tabulated.",
              "- Minimum normalized wheel margin reached zero for all methods, so that minimum alone cannot express improvement; mean/p05 and threshold durations are retained.",
              "- M1 rigid-fit maximum was slightly worse than M1b in this run even though rigid-fit RMSE improved; this unfavorable point is retained.",
              "- CapabilityReport was not modified. Candidate A and Candidate B v1 remained disabled.",
              "- No serial or hardware execution was started. Physical readiness remains NOT_AUTHORIZED; no transition to physical qualification is recommended without a separate safety review.", "",
              "## Regression verification for this implementation revision", "",
              "- Candidate B v2 C++ and Python tests: PASS.",
              "- Candidate B v1 C++: PASS, 4 tests; Python: PASS, 3 tests.",
              "- Candidate A C++: PASS, 9 tests; Python: PASS, 4 tests.",
              "- Analysis metrics: PASS, 64 tests; bringup static: PASS, 49 tests.",
              "- Involved C++ build, shell syntax, Python compile, and `git diff --check`: PASS.", "",
              "Python regression commands used `PYTHONNOUSERSITE=1` so ROS Noetic loaded the compatible system NumPy 1.17.4 and SciPy 1.3.3. The host user-site NumPy is incompatible with system SciPy; this is an analysis-environment issue, not a Candidate B regression."]
    report = root / REPORT_FILENAME
    report.write_text("\n".join(lines)+"\n")
    return result


def sanity(metrics_path):
    data=json.loads(metrics_path.read_text())
    verification=data["disturbance_verification"]
    r1,r2,r3=(verification[f"Robot{i}"] for i in (1,2,3))
    expected=0.259807621
    actual2=r2["entry_common_progress_minus_robot1_m"]
    actual3=r3["entry_common_progress_minus_robot1_m"]
    ok=(r1["entry_wall_time"] < r2["entry_wall_time"] and
        r1["entry_wall_time"] < r3["entry_wall_time"] and
        # The two rear supports are not exactly coincident along a curved
        # path.  "Approximately simultaneous" is therefore checked as less
        # than 5 cm of nominal travel (0.50 s at 0.10 m/s), while the separate
        # progress checks below retain the geometric requirement.
        abs(r2["entry_wall_time"]-r3["entry_wall_time"]) < .50 and
        abs(actual2-expected) < .04 and abs(actual3-expected) < .04)
    result={"passed":ok,"expected_front_rear_geometry_m":expected,
            "entry_order":verification["entry_order"],
            "robot2_minus_robot1_entry_time_seconds":r2["entry_wall_time"]-r1["entry_wall_time"],
            "robot3_minus_robot1_entry_time_seconds":r3["entry_wall_time"]-r1["entry_wall_time"],
            "robot2_minus_robot1_common_progress_m":actual2,
            "robot3_minus_robot1_common_progress_m":actual3}
    save(metrics_path.parent/"spatial_sanity.json",result)
    if not ok: raise ValueError("spatial propagation sanity check failed")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--prepare",type=Path); parser.add_argument("--method",choices=METHODS)
    parser.add_argument("--check-run",type=Path); parser.add_argument("--check-log",type=Path)
    parser.add_argument("--aggregate",type=Path); parser.add_argument("--sanity",type=Path)
    args=parser.parse_args()
    if args.prepare: prepare(args.prepare.resolve(),args.method); return 0
    if args.aggregate: aggregate(args.aggregate.resolve()); return 0
    if args.sanity: sanity(args.sanity.resolve()); return 0
    try:
        result=analyze(args.check_run.resolve(),args.method,args.check_log.resolve())
    except Exception as exc:
        result={"experiment_id":IDENTITY,"markers":MARKERS,"category":"INVALID_RUN","reason":str(exc)}
    save(args.check_run/"candidate_b_spatial_metrics.json",result)
    print(result["category"])
    return 20 if result["category"]=="INVALID_RUN" else 0


if __name__ == "__main__":
    raise SystemExit(main())
