#!/usr/bin/env python3
"""Analyze and plot direct physical Candidate B gradient15 runs.

The physical comparison deliberately reuses the frozen fake implementation's
error signals, phase metrics and 5001-point spatial-domain RMSE definition.
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "candidate_b_gradient15_fake",
    HERE / "analyze_candidate_B_spatial_gradient15_fake.py")
GRADIENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(GRADIENT)
BASE = GRADIENT.BASE
V2 = BASE.V2
V4 = BASE.V4

IDENTITY = "candidate_B_spatial_gradient15_physical_validation"
PROFILE_ID = "candidate_B_v2_spatial_gradient15_rho0p85_av0p020_aw0p2625"
PROFILE = dict(BASE.PROFILE)
METHODS = BASE.METHODS
METRIC_FILE = "candidate_b_spatial_gradient15_physical_metrics.json"
COMPARISON_FILE = "candidate_b_spatial_gradient15_physical_comparison.json"
SUMMARY_FILE = "candidate_b_spatial_gradient15_physical_summary.json"
REPORT_FILE = "candidate-B-spatial-gradient15-physical-results.md"
SUMMARY_CSV = "candidate_b_spatial_gradient15_physical_summary.csv"

LOWER_BETTER = [
    "support_error_rmse", "support_error_max",
    "rigid_fit_error_rmse", "rigid_fit_error_max",
    "pairwise_side_error_rmse", "pairwise_side_error_max",
    "equivalent_load_position_error_rmse",
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
    "wheel_margin_below_0p7_duration_seconds",
]
for _robot in (1, 2, 3):
    LOWER_BETTER.extend(
        "robot{}_{}_error_rmse".format(_robot, kind)
        for kind in ("progress", "lateral", "heading", "velocity"))
HIGHER_BETTER = ["wheel_margin_minimum", "wheel_margin_mean",
                 "wheel_margin_p05"]


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def classify_physical_evidence(runtime_valid, authorization_source):
    provenance_complete = (
        authorization_source == "runtime_operator_overlay")
    if runtime_valid and provenance_complete:
        return "VALID_COMPLETED", provenance_complete
    if runtime_valid:
        return "VALID_COMPLETED_NONFORMAL_PROVENANCE", provenance_complete
    return "INVALID_RUN", provenance_complete


def physical_algorithm_params(params):
    """Return the serial algorithm namespace captured by rosparam dump."""
    algorithm = params.get("formal_algorithm")
    if not isinstance(algorithm, dict):
        raise ValueError(
            "physical rosparams are missing the formal_algorithm namespace")
    return algorithm


def nominal_common_inner_upper(algorithm_params):
    """Recover the frozen no-risk common upper for historical serial bags."""
    agents = algorithm_params["formal_upper"]["agents"]
    nominal = agents["nominal_upper"]
    margin = agents["inner_margin"]
    if len(nominal) != 3 or len(margin) != 3:
        raise ValueError("physical upper configuration must contain three agents")
    return min(float(upper) - float(inner)
               for upper, inner in zip(nominal, margin))


def selected_rows(run, primary_only=False):
    rows, seen = [], set()
    with (run / "converted/aligned_samples.csv").open(newline="") as stream:
        for source in csv.DictReader(stream):
            if (not V2.flag(source, "algorithm_valid") or
                    not V2.flag(source,
                                "candidate_b_spatial_state_available")):
                continue
            wall = V2.num(source, "candidate_b_spatial_wall_time")
            if not math.isfinite(wall) or wall in seen:
                continue
            seen.add(wall)
            row = dict(source)
            row["wall"] = wall
            row["errors"] = V4.error_signals(row)
            row["robot_errors"] = {
                robot: BASE.robot_errors(row, robot) for robot in (1, 2, 3)}
            rows.append(row)
    rows.sort(key=lambda row: row["wall"])
    if not rows:
        raise ValueError("no valid Candidate B spatial samples")
    if not primary_only:
        return rows
    verification = BASE.disturbance_verification(rows)
    entry = min(verification["Robot{}".format(robot)]["entry_wall_time"]
                for robot in (1, 2, 3))
    exit_time = max(
        verification["Robot{}".format(robot)]["exit_wall_time"]
        for robot in (1, 2, 3))
    return [row for row in rows if entry <= row["wall"] <= exit_time]


def analyze_physical_run(run, method):
    manifest = yaml.safe_load((run / "manifest.yaml").read_text())
    params = yaml.safe_load((run / "rosparams.yaml").read_text())
    algorithm_params = physical_algorithm_params(params)
    runtime = algorithm_params["formal_fake_runtime"]
    if manifest["experiment_id"] != IDENTITY or runtime["experiment_id"] != IDENTITY:
        raise ValueError("wrong physical Candidate B identity")
    if manifest.get("method_id") != METHODS[method]:
        raise ValueError("physical method metadata does not match request")
    if runtime["candidate_b_spatial_composite"] != PROFILE:
        raise ValueError("physical Candidate B profile differs from gradient15 freeze")
    physical = runtime.get("candidate_b_spatial_physical", {})
    if (physical.get("software_qualification_passed") is not True or
            physical.get("enabled") is not True or
            physical.get("hardware_execution_authorized") is not True):
        raise ValueError(
            "physical Candidate B qualification or authorization was not active")
    if (runtime.get("candidate_b", {}).get("enabled", False) or
            runtime.get("classic_additive_candidate_A", {}).get(
                "enabled", False)):
        raise ValueError("another disturbance profile was enabled")
    for robot in (1, 2, 3):
        transport = params["agv{}".format(robot)]["chassis_controller"][
            "transport_type"]
        if transport != "serial":
            raise ValueError("physical analysis requires serial transport")
    metadata = manifest.get("candidate_b_spatial", {})
    if (metadata.get("profile_id") != PROFILE_ID or
            metadata.get("physical_authorized") is not True):
        raise ValueError("Candidate B physical provenance is incomplete")
    authorization_source = metadata.get("authorization_source", "")
    if (algorithm_params.get("formal_upper", {}).get(
            "hardware_execution_authorized") is not True or
            algorithm_params.get("formal_lower", {}).get(
                "hardware_execution_authorized") is not True):
        raise ValueError("method hardware authorization was not recorded")
    payload = manifest.get("payload", {})
    if (payload.get("mode") != "unloaded" or
            payload.get("pose_source") != "equivalent_load"):
        raise ValueError("this entry requires unloaded equivalent payload semantics")

    rows = selected_rows(run)
    verification = BASE.disturbance_verification(rows)
    entry = min(verification["Robot{}".format(robot)]["entry_wall_time"]
                for robot in (1, 2, 3))
    exit_time = max(
        verification["Robot{}".format(robot)]["exit_wall_time"]
        for robot in (1, 2, 3))
    for row in rows:
        row["relative_wall_time"] = row["wall"] - entry
    primary = [row for row in rows if entry <= row["wall"] <= exit_time]
    metrics = BASE.phase_metrics(primary)
    validation = json.loads((run / "validation.json").read_text())
    summary = json.loads((run / "summary_metrics.json").read_text())
    complete = bool(validation.get("task_completion", {}).get("complete"))
    algorithm = validation.get("algorithm", {})
    localization = validation.get("localization", {})
    algorithm_invalid = float(algorithm.get("invalid_fraction", 0.0))
    localization_invalid = float(localization.get("invalid_fraction", 0.0))
    scales = PROFILE["severity_scale"]
    profile_ok = all(
        abs(verification["Robot{}".format(robot)]["minimum_rho"] -
            (1.0 - .15 * scales[robot - 1])) < 2e-3 and
        verification["Robot{}".format(robot)]["d_v_peak"] <=
            .020 * scales[robot - 1] + 1e-7 and
        verification["Robot{}".format(robot)]["d_omega_peak"] <=
            .2625 * scales[robot - 1] + 1e-7
        for robot in (1, 2, 3))
    runtime_log = run / "physical_runtime.log"
    runtime_log_present = runtime_log.is_file()
    first_valid_stamp = float(algorithm.get(
        "first_valid_stamp", -math.inf))
    completion_stamp = float(algorithm.get(
        "completion_stamp", math.inf))
    hard_failure = (BASE.hard_failure_in_valid_window(
        runtime_log, first_valid_stamp, completion_stamp,
        ("emergency abort", "emergency-stop abort", "abort latched"))
        if runtime_log_present else False)
    git_dirty = bool(manifest.get("git_dirty", False))
    git_dirty_reason = manifest.get(
        "git_dirty_reason",
        "unspecified_worktree_changes" if git_dirty else "clean")
    runtime_valid = (validation.get("valid") is True and complete and
                     profile_ok and algorithm_invalid == 0.0 and
                     localization_invalid == 0.0 and
                     runtime_log_present and not hard_failure)
    category, provenance_complete = classify_physical_evidence(
        runtime_valid, authorization_source)
    result = {
        "experiment_id": IDENTITY,
        "profile_id": PROFILE_ID,
        "formal_evidence": category == "VALID_COMPLETED",
        "physical_authorization_recorded": True,
        "software_qualification_recorded": True,
        "payload_semantics": "unloaded_equivalent_payload",
        "method": METHODS[method],
        "category": category,
        "complete": complete,
        "task_time_seconds": summary.get("task", {}).get("completion_time"),
        "metrics": metrics,
        "disturbance_verification": verification,
        "M1_mechanism": (BASE.mechanism_metrics(
            primary, nominal_common_inner_upper(algorithm_params))
                           if method == "M1" else None),
        "profile_verified": profile_ok,
        "algorithm_invalid_fraction": algorithm_invalid,
        "localization_invalid_fraction": localization_invalid,
        "emergency_threshold_events": sum(
            V2.flag(row, "agv{}_candidate_b_spatial_post_disturbance_over_0p180".format(robot))
            for row in primary for robot in (1, 2, 3)),
        "fail_zero_or_abort_detected": hard_failure,
        "runtime_log_present": runtime_log_present,
        "provenance": {
            "git_sha": manifest.get("git_sha", ""),
            "git_dirty": git_dirty,
            "git_dirty_reason": git_dirty_reason,
            "git_status": manifest.get("git_status", []),
            "authorization_source": authorization_source,
            "tracked_frozen_config_and_runtime_overlay_archived":
                provenance_complete,
        },
        "run_path": str(run),
    }
    save(run / METRIC_FILE, result)
    return result


def improvement(m1, comparator, higher=False):
    if abs(comparator) <= 1e-12:
        return None
    numerator = m1 - comparator if higher else comparator - m1
    return 100.0 * numerator / abs(comparator)


def comparison_metrics(data):
    result = {}
    for comparator in ("M1b", "M2b"):
        table = {}
        for key in LOWER_BETTER + HIGHER_BETTER:
            m1 = data["M1"]["metrics"][key]
            other = data[comparator]["metrics"][key]
            table[key] = {
                "M1": m1, comparator: other,
                "direction": ("higher_is_better" if key in HIGHER_BETTER
                              else "lower_is_better"),
                "improvement_percent": improvement(
                    m1, other, key in HIGHER_BETTER),
            }
        result["M1_vs_{}".format(comparator)] = table
    return result


def _error_series(rows, position=True):
    if position:
        return [math.hypot(
            V2.num(row, "equivalent_load_pose_x") -
            V2.num(row, "load_x_reference"),
            V2.num(row, "equivalent_load_pose_y") -
            V2.num(row, "load_y_reference")) for row in rows]
    return [abs(V2.wrap(
        V2.num(row, "equivalent_load_pose_yaw") -
        V2.num(row, "load_yaw_reference"))) for row in rows]


def make_plots(output, rows_by_method, spatial, data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir = output / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    colors = {"M1": "#1f77b4", "M1b": "#d62728", "M2b": "#2ca02c"}
    robot_colors = {1: "#1f77b4", 2: "#d62728", 3: "#2ca02c"}

    def finish(fig, name):
        fig.savefig(plot_dir / (name + ".png"), dpi=300,
                    bbox_inches="tight")
        fig.savefig(plot_dir / (name + ".pdf"), bbox_inches="tight")
        plt.close(fig)

    # Figure 1: M1 robot trajectories and the actual path segment that defines
    # the spatial disturbance zone (not an invented Cartesian rectangle).
    fig, ax = plt.subplots(figsize=(8, 6))
    rows = rows_by_method["M1"]
    for robot in (1, 2, 3):
        ax.plot([V2.num(row, "agv{}_robot_pose_x".format(robot)) for row in rows],
                [V2.num(row, "agv{}_robot_pose_y".format(robot)) for row in rows],
                color=robot_colors[robot], label="Robot{}".format(robot))
    ref = [(V2.num(row, "load_x_reference"),
            V2.num(row, "load_y_reference"),
            V2.num(row, "load_s_reference")) for row in rows]
    ax.plot([p[0] for p in ref], [p[1] for p in ref], "k--", lw=1,
            label="load reference")
    zone = [p for p in ref if 2.0 <= p[2] <= 2.8]
    ax.plot([p[0] for p in zone], [p[1] for p in zone], color="#f0a202",
            lw=6, alpha=.45, label="gradient15 zone")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x / m"); ax.set_ylabel("y / m")
    ax.grid(alpha=.25); ax.legend(ncol=2)
    finish(fig, "figure1_trajectories_and_spatial_zone")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for method, method_rows in rows_by_method.items():
        t0 = method_rows[0]["wall"]
        axes[0].plot([row["wall"] - t0 for row in method_rows],
                     [row["errors"]["support"] for row in method_rows],
                     color=colors[method], label=method)
    axes[0].set_xlabel("time / s"); axes[0].set_ylabel("support error / m")
    vals = [spatial["methods"][method]["support_error_spatial_rmse"]
            for method in ("M1", "M1b", "M2b")]
    axes[1].bar(("M1", "M1b", "M2b"), vals,
                color=[colors[m] for m in ("M1", "M1b", "M2b")])
    axes[1].set_ylabel("spatial RMSE / m")
    for ax in axes: ax.grid(alpha=.25); ax.legend() if ax is axes[0] else None
    finish(fig, "figure2_support_error")

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for method, method_rows in rows_by_method.items():
        t0 = method_rows[0]["wall"]
        tt = [row["wall"] - t0 for row in method_rows]
        axes[0, 0].plot(tt, [row["errors"]["rigid_fit"] for row in method_rows],
                        color=colors[method], label=method)
        axes[1, 0].plot(tt, [row["errors"]["pairwise_side"] for row in method_rows],
                        color=colors[method], label=method)
    methods = ("M1", "M1b", "M2b")
    axes[0, 1].bar(methods, [spatial["methods"][m]["rigid_fit_error_spatial_rmse"] for m in methods],
                    color=[colors[m] for m in methods])
    axes[1, 1].bar(methods, [spatial["methods"][m]["pairwise_side_error_spatial_rmse"] for m in methods],
                    color=[colors[m] for m in methods])
    axes[0, 0].set_ylabel("rigid-fit / m")
    axes[1, 0].set_ylabel("pairwise / m"); axes[1, 0].set_xlabel("time / s")
    axes[0, 1].set_ylabel("rigid spatial RMSE / m")
    axes[1, 1].set_ylabel("pairwise spatial RMSE / m")
    for ax in axes.flat: ax.grid(alpha=.25)
    axes[0, 0].legend()
    finish(fig, "figure3_rigid_fit_and_pairwise")

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=False)
    for method, method_rows in rows_by_method.items():
        t0 = method_rows[0]["wall"]
        tt = [row["wall"] - t0 for row in method_rows]
        axes[0].plot(tt, _error_series(method_rows, True),
                     color=colors[method], label=method)
        axes[1].plot(tt, _error_series(method_rows, False),
                     color=colors[method], label=method)
    axes[0].set_ylabel("equivalent-load position error / m")
    axes[1].set_ylabel("equivalent-load yaw error / rad")
    axes[1].set_xlabel("time / s")
    for ax in axes: ax.grid(alpha=.25); ax.legend()
    finish(fig, "figure4_equivalent_load_errors")

    rows = rows_by_method["M1"]
    verification = data["M1"]["disturbance_verification"]
    zone_entry = min(verification["Robot{}".format(robot)]["entry_wall_time"]
                     for robot in (1, 2, 3))
    zone_exit = max(verification["Robot{}".format(robot)]["exit_wall_time"]
                    for robot in (1, 2, 3))
    rows = [row for row in rows if zone_entry <= row["wall"] <= zone_exit]
    t0 = rows[0]["wall"]
    tt = [row["wall"] - t0 for row in rows]
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    risk_line = axes[0, 0].plot(
        tt, [max(V2.num(row, "agv{}_risk_factor".format(r))
                 for r in (1, 2, 3)) for row in rows],
        label="risk factor")
    contraction_axis = axes[0, 0].twinx()
    contraction_line = contraction_axis.plot(
        tt, [V2.num(row, "risk_contraction") for row in rows],
        color="#f0a202", label="contraction")
    contraction_axis.set_ylabel("contraction / (m/s)", color="#a66b00")
    contraction_axis.tick_params(axis="y", colors="#a66b00")
    axes[0, 1].plot(tt, [V2.num(row, "candidate_common_velocity") for row in rows],
                    "--", label="candidate reference")
    axes[0, 1].plot(tt, [V2.num(row, "common_velocity_reference") for row in rows],
                    label="effective reference")
    for robot in (1, 2, 3):
        prefix = "agv{}_candidate_b_spatial_".format(robot)
        axes[1, 0].plot(tt, [max(abs(V2.num(row, prefix + "left_native")),
                                      abs(V2.num(row, prefix + "right_native")))
                                   for row in rows],
                        color=robot_colors[robot], label="Robot{} native".format(robot))
        axes[1, 0].plot(tt, [max(abs(V2.num(row, prefix + "actual_left")),
                                      abs(V2.num(row, prefix + "actual_right")))
                                   for row in rows],
                        color=robot_colors[robot], linestyle=":",
                        label="Robot{} actual".format(robot))
        axes[1, 1].plot(tt, [max(0.0, min(1.0,
            (V2.num(row, prefix + "capability_limit") - max(
                abs(V2.num(row, prefix + "left_native")),
                abs(V2.num(row, prefix + "right_native")))) / .010))
            for row in rows], color=robot_colors[robot], label="Robot{}".format(robot))
    axes[1, 0].axhline(.160, color="k", linestyle="--", label="0.160 limit")
    axes[1, 0].axhline(.180, color="k", linestyle=":", label="0.180 emergency")
    axes[0, 0].set_ylabel("risk factor")
    axes[0, 1].set_ylabel("reference / (m/s)")
    axes[1, 0].set_ylabel("wheel speed / (m/s)")
    axes[1, 1].set_ylabel("normalized wheel margin")
    axes[1, 0].set_xlabel("time / s"); axes[1, 1].set_xlabel("time / s")
    for ax in axes.flat: ax.grid(alpha=.25); ax.legend(fontsize=7, ncol=2)
    axes[0, 0].legend(risk_line + contraction_line,
                      [line.get_label() for line in
                       risk_line + contraction_line], fontsize=7)
    finish(fig, "figure5_mechanism_and_execution_zoom")

    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.axis("off")
    columns = ["Method", "Support spatial", "Rigid spatial", "Pair spatial",
               "Load pos RMS", "Load yaw RMS", "Native peak", "Limiter s",
               "Margin mean", "Task s"]
    cells = []
    for method in methods:
        metric = data[method]["metrics"]
        spatial_metric = spatial["methods"][method]
        cells.append([
            method,
            "{:.5f}".format(spatial_metric["support_error_spatial_rmse"]),
            "{:.5f}".format(spatial_metric["rigid_fit_error_spatial_rmse"]),
            "{:.5f}".format(spatial_metric["pairwise_side_error_spatial_rmse"]),
            "{:.5f}".format(metric["equivalent_load_position_error_rmse"]),
            "{:.5f}".format(metric["equivalent_load_yaw_error_rmse"]),
            "{:.4f}".format(metric["native_controller_raw_peak_mps"]),
            "{:.3f}".format(metric["physical_limiter_duration_seconds"]),
            "{:.3f}".format(metric["wheel_margin_mean"]),
            "{:.3f}".format(data[method]["task_time_seconds"]),
        ])
    table = ax.table(cellText=cells, colLabels=columns, loc="center",
                     cellLoc="center")
    table.auto_set_font_size(False); table.set_fontsize(8); table.scale(1, 1.6)
    finish(fig, "figure6_summary_table")


def aggregate(output, paths, source_kind):
    output.mkdir(parents=True, exist_ok=True)
    data, rows, spatial_rows = {}, {}, {}
    for method in ("M1", "M1b", "M2b"):
        run = paths[method].resolve()
        if source_kind == "physical":
            metric_path = run / METRIC_FILE
            data[method] = (json.loads(metric_path.read_text())
                            if metric_path.exists()
                            else analyze_physical_run(run, method))
        else:
            data[method] = json.loads(
                (run / "candidate_b_spatial_metrics.json").read_text())
        rows[method] = selected_rows(run, primary_only=False)
        spatial_rows[method] = selected_rows(run, primary_only=True)
    valid = all(value["category"] == "VALID_COMPLETED"
                for value in data.values())
    spatial = BASE.spatial_domain_metrics(spatial_rows, sample_count=5001)
    comparisons = comparison_metrics(data)
    result = {
        "experiment_id": (IDENTITY if source_kind == "physical"
                          else GRADIENT.BASE.IDENTITY),
        "source_kind": source_kind,
        "profile_id": PROFILE_ID,
        "profile": PROFILE,
        "all_runs_valid": valid,
        "methods": data,
        "comparisons": comparisons,
        "spatial_domain_rmse": spatial,
    }
    save(output / COMPARISON_FILE, result)
    save(output / SUMMARY_FILE, {
        "source_kind": source_kind, "all_runs_valid": valid,
        "profile_id": PROFILE_ID,
        "common_spatial_interval": {
            "coordinate": "load_s_reference",
            "start_m": spatial["common_interval_start_m"],
            "end_m": spatial["common_interval_end_m"],
            "grid_samples": spatial["grid_samples"],
            "integration": spatial["integration"],
        },
        "methods": {method: data[method]["metrics"] for method in data},
        "spatial_metrics": spatial,
    })

    fieldnames = ["method", "category", "task_time_seconds"] + \
        LOWER_BETTER + HIGHER_BETTER + [
            "support_error_spatial_rmse", "rigid_fit_error_spatial_rmse",
            "pairwise_side_error_spatial_rmse"]
    with (output / SUMMARY_CSV).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for method in ("M1", "M1b", "M2b"):
            row = {"method": method, "category": data[method]["category"],
                   "task_time_seconds": data[method]["task_time_seconds"]}
            row.update({key: data[method]["metrics"][key]
                        for key in LOWER_BETTER + HIGHER_BETTER})
            row.update(spatial["methods"][method])
            writer.writerow(row)

    make_plots(output, rows, spatial, data)
    lines = [
        "# Candidate B spatial gradient15 {} results".format(source_kind), "",
        "Profile: `{}`".format(PROFILE_ID), "",
        "Payload semantics: unloaded equivalent payload (not measured payload).", "",
        "Common spatial interval: `{:.9g}..{:.9g} m`, coordinate "
        "`load_s_reference`, 5001 samples, trapezoidal spatial integration.".format(
            spatial["common_interval_start_m"],
            spatial["common_interval_end_m"]), "",
        "| Method | Support spatial RMSE / m | Rigid-fit spatial RMSE / m | Pairwise spatial RMSE / m | Load position RMS / m | Load yaw RMS / rad | Native wheel peak / (m/s) | Limiter / s | Margin mean | Task / s |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for method in ("M1", "M1b", "M2b"):
        metric = data[method]["metrics"]
        sm = spatial["methods"][method]
        lines.append(
            "| {} | {:.8g} | {:.8g} | {:.8g} | {:.8g} | {:.8g} | {:.8g} | {:.8g} | {:.8g} | {:.8g} |".format(
                method, sm["support_error_spatial_rmse"],
                sm["rigid_fit_error_spatial_rmse"],
                sm["pairwise_side_error_spatial_rmse"],
                metric["equivalent_load_position_error_rmse"],
                metric["equivalent_load_yaw_error_rmse"],
                metric["native_controller_raw_peak_mps"],
                metric["physical_limiter_duration_seconds"],
                metric["wheel_margin_mean"], data[method]["task_time_seconds"]))
    lines += ["", "## Spatial-domain improvements", "",
              "| Comparison | Support | Rigid-fit | Pairwise |",
              "| --- | ---: | ---: | ---: |"]
    for comparator in ("M1b", "M2b"):
        values = spatial["comparisons"]["M1_vs_{}".format(comparator)]
        lines.append("| M1 vs {} | {:.3f}% | {:.3f}% | {:.3f}% |".format(
            comparator,
            values["support_error_spatial_rmse"]["improvement_percent"],
            values["rigid_fit_error_spatial_rmse"]["improvement_percent"],
            values["pairwise_side_error_spatial_rmse"]["improvement_percent"]))
    lines += ["", "All source values remain in the JSON/CSV outputs. "
              "No samples are removed for plotting."]
    (output / REPORT_FILE).write_text("\n".join(lines) + "\n")
    return 0 if valid else 20


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--analyze-run", type=Path)
    parser.add_argument("--method", choices=METHODS)
    parser.add_argument("--compare", type=Path)
    parser.add_argument("--m1", type=Path)
    parser.add_argument("--m1b", type=Path)
    parser.add_argument("--m2b", type=Path)
    parser.add_argument("--source-kind", choices=("physical", "fake"),
                        default="physical")
    args = parser.parse_args()
    if args.analyze_run:
        if not args.method:
            parser.error("--analyze-run requires --method")
        result = analyze_physical_run(args.analyze_run.resolve(), args.method)
        print(result["category"])
        return 0 if result["category"] == "VALID_COMPLETED" else 20
    if args.compare:
        if not all((args.m1, args.m1b, args.m2b)):
            parser.error("--compare requires --m1, --m1b and --m2b")
        return aggregate(args.compare.resolve(), {
            "M1": args.m1, "M1b": args.m1b, "M2b": args.m2b},
            args.source_kind)
    parser.error("select --analyze-run or --compare")


if __name__ == "__main__":
    raise SystemExit(main())
