"""Logical paper tables and deterministic draft figures for one run."""

import math
from pathlib import Path

from multi_agv_analysis.io_utils import finite_float, read_csv, write_csv


GROUPS = {
    "state.csv": (
        "stamp", "localization_valid", "evaluation_active", "load_",
        "agv1_robot_pose_", "agv2_robot_pose_", "agv3_robot_pose_",
        "agv1_support_pose_", "agv2_support_pose_", "agv3_support_pose_",
        "agv1_support_reference_", "agv2_support_reference_",
        "agv3_support_reference_", "agv1_s_", "agv2_s_", "agv3_s_"),
    "upper_layer.csv": (
        "stamp", "evaluation_active", "common_velocity_",
        "public_reference_velocity", "common_boundary_",
        "mapped_path_capability_available", "agv1_mapped_path_",
        "agv2_mapped_path_", "agv3_mapped_path_",
        "agv1_boundary_", "agv2_boundary_", "agv3_boundary_",
        "agv1_robust_margin", "agv2_robust_margin", "agv3_robust_margin",
        "agv1_risk_", "agv2_risk_", "agv3_risk_", "agv1_z",
        "agv2_z", "agv3_z", "agv1_upsilon", "agv2_upsilon",
        "agv3_upsilon", "agv1_phi", "agv2_phi", "agv3_phi"),
    "wheel_chain.csv": (
        "stamp", "evaluation_active", "agv1_wheel_", "agv2_wheel_",
        "agv3_wheel_"),
    "lower_layer.csv": (
        "stamp", "evaluation_active", "load_s_reference",
        "load_velocity_reference", "agv1_position_error",
        "agv2_position_error", "agv3_position_error",
        "agv1_velocity_error", "agv2_velocity_error",
        "agv3_velocity_error", "agv1_psi", "agv2_psi", "agv3_psi",
        "agv1_gamma", "agv2_gamma", "agv3_gamma",
        "agv1_gamma_inverse_upsilon", "agv2_gamma_inverse_upsilon",
        "agv3_gamma_inverse_upsilon", "agv1_composite_error",
        "agv2_composite_error", "agv3_composite_error",
        "agv1_channel_input_", "agv2_channel_input_",
        "agv3_channel_input_", "agv1_theta_hat_",
        "agv2_theta_hat_", "agv3_theta_hat_",
        "agv1_disturbance_estimate", "agv2_disturbance_estimate",
        "agv3_disturbance_estimate"),
    "m2b_internal.csv": ("stamp", "evaluation_active", "m2b_"),
}


def _selected_fields(rows, prefixes):
    fields = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen and any(
                    key == prefix or key.startswith(prefix)
                    for prefix in prefixes):
                fields.append(key)
                seen.add(key)
    return fields


def export_views(converted_dir, output_dir):
    rows = read_csv(Path(converted_dir) / "aligned_samples.csv")
    if not rows:
        raise RuntimeError("aligned_samples.csv contains no samples")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    controller_path = Path(converted_dir) / "controller_state.csv"
    controller_rows = read_csv(controller_path) if controller_path.exists() else []
    method_id = (
        str(controller_rows[0].get("method_id", ""))
        if controller_rows else "")
    written = []
    for filename, prefixes in GROUPS.items():
        fields = _selected_fields(rows, prefixes)
        payload_fields = [field for field in fields
                          if field not in ("stamp", "evaluation_active")]
        if not payload_fields:
            if (filename == "m2b_internal.csv" and
                    method_id != "M2b_M2b"):
                # M2b internals do not exist for M1/M2a methods.  Their
                # absence is method-correct and must not invalidate an
                # otherwise complete paper run.
                continue
            raise RuntimeError("no fields available for {}".format(filename))
        view = [{field: row.get(field, "") for field in fields}
                for row in rows]
        write_csv(output_dir / filename, view, fields)
        written.append(filename)
    capability_rows = read_csv(Path(converted_dir) / "capability_report.csv")
    write_csv(output_dir / "capability.csv", capability_rows)
    written.append("capability.csv")
    return written


def _series(rows, field):
    return [finite_float(row.get(field)) for row in rows]


def _finite_xy(x_values, y_values):
    return [(x, y) for x, y in zip(x_values, y_values)
            if math.isfinite(x) and math.isfinite(y)]


def _save(fig, output, name):
    fig.tight_layout()
    fig.savefig(output / (name + ".png"), dpi=320)
    fig.savefig(output / (name + ".pdf"))


def plot_run(aligned_csv, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = read_csv(aligned_csv)
    if not rows:
        raise RuntimeError("aligned input contains no samples")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    stamps = _series(rows, "stamp")
    finite_stamps = [value for value in stamps if math.isfinite(value)]
    if not finite_stamps:
        raise RuntimeError("aligned input has no finite timestamps")
    origin = finite_stamps[0]
    time = [value - origin for value in stamps]
    colors = ("#1f77b4", "#d62728", "#2ca02c")

    fig, ax = plt.subplots(figsize=(7.0, 5.4))
    reference = _finite_xy(_series(rows, "load_x_reference"),
                           _series(rows, "load_y_reference"))
    if reference:
        ax.plot(*zip(*reference), "k--", label="reference")
    for robot, color in zip(range(1, 4), colors):
        values = _finite_xy(
            _series(rows, "agv{}_support_pose_x".format(robot)),
            _series(rows, "agv{}_support_pose_y".format(robot)))
        if values:
            ax.plot(*zip(*values), color=color, label="AGV{}".format(robot))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend()
    _save(fig, output, "figure2_trajectory")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.5), sharex=True)
    for robot, color in zip(range(1, 4), colors):
        physical = []
        for row in rows:
            left = finite_float(row.get(
                "agv{}_wheel_left_reported_limit".format(robot)))
            right = finite_float(row.get(
                "agv{}_wheel_right_reported_limit".format(robot)))
            physical.append(min(left, right) if math.isfinite(left + right)
                            else math.nan)
        axes[0].plot(time, physical, color=color,
                     label="AGV{} physical wheel".format(robot))
    axes[0].set_ylabel("wheel limit [m/s]")
    axes[0].legend(loc="upper right")
    for robot, color in zip(range(1, 4), colors):
        axes[1].plot(time, _series(
            rows, "agv{}_mapped_path_velocity_upper".format(robot)),
            color=color, alpha=0.8,
            label="AGV{} mapped path".format(robot))
        axes[1].plot(time, _series(
            rows, "agv{}_boundary_upper".format(robot)),
            color=color, ls=":", alpha=0.65,
            label="AGV{} dynamic boundary".format(robot))
    axes[1].plot(time, _series(rows, "common_boundary_upper"), "k--",
                 lw=1.0, label="common boundary")
    axes[1].plot(time, _series(rows, "public_reference_velocity"), "k",
                 lw=1.5, label="public reference")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("path speed [m/s]")
    axes[1].legend(loc="upper right", ncol=2, fontsize=7)
    _save(fig, output, "figure3_capability_boundary_reference")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.0), sharex=True)
    for robot, ax, color in zip(range(1, 4), axes, colors):
        ax.plot(time, _series(rows, "agv{}_s_dot_actual".format(robot)),
                color=color, label="actual")
        ax.plot(time, _series(rows, "agv{}_boundary_lower".format(robot)),
                "k--", lw=0.8, label="lower")
        ax.plot(time, _series(rows, "agv{}_boundary_upper".format(robot)),
                "k-.", lw=0.8, label="upper")
        ax.set_ylabel("AGV{} [m/s]".format(robot))
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    _save(fig, output, "figure4_channel_constraints")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True)
    for ax, side in zip(axes, ("left", "right")):
        prefix = "agv2_wheel_{}".format(side)
        ax.plot(time, _series(rows, prefix + "_pre_limit"),
                label="pre-limit")
        ax.plot(time, _series(rows, prefix + "_fleet_scaled"),
                label="fleet-scaled")
        ax.plot(time, _series(rows, prefix + "_applied"), label="applied")
        ax.plot(time, _series(rows, prefix + "_actual"), label="actual")
        ax.plot(time, _series(rows, prefix + "_reported_limit"), "k--",
                label="limit")
        ax.set_ylabel(side + " [m/s]")
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    _save(fig, output, "figure5_robot2_wheel_chain")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    load_error = []
    for row in rows:
        dx = (finite_float(row.get("load_pose_x")) -
              finite_float(row.get("load_x_reference")))
        dy = (finite_float(row.get("load_pose_y")) -
              finite_float(row.get("load_y_reference")))
        load_error.append(math.hypot(dx, dy) if math.isfinite(dx + dy)
                          else math.nan)
    ax.plot(time, load_error, "k", label="load/equivalent load")
    for robot, color in zip(range(1, 4), colors):
        errors = []
        for row in rows:
            dx = (finite_float(row.get(
                "agv{}_support_pose_x".format(robot))) -
                finite_float(row.get(
                    "agv{}_support_reference_x".format(robot))))
            dy = (finite_float(row.get(
                "agv{}_support_pose_y".format(robot))) -
                finite_float(row.get(
                    "agv{}_support_reference_y".format(robot))))
            errors.append(math.hypot(dx, dy) if math.isfinite(dx + dy)
                          else math.nan)
        ax.plot(time, errors, color=color, label="AGV{} support".format(robot))
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.legend()
    _save(fig, output, "figure6_cooperative_errors")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True)
    s_ref = _series(rows, "load_s_reference")
    v_ref = _series(rows, "load_velocity_reference")
    for robot, color in zip(range(1, 4), colors):
        s_error = [a - b for a, b in zip(
            _series(rows, "agv{}_s_actual".format(robot)), s_ref)]
        v_error = [a - b for a, b in zip(
            _series(rows, "agv{}_s_dot_actual".format(robot)), v_ref)]
        axes[0].plot(time, s_error, color=color, label="AGV{}".format(robot))
        axes[1].plot(time, v_error, color=color, label="AGV{}".format(robot))
    axes[0].set_ylabel("progress error [m]")
    axes[0].legend()
    axes[1].set_ylabel("speed error [m/s]")
    axes[1].set_xlabel("time [s]")
    _save(fig, output, "figure7_progress_recovery")
    plt.close(fig)
    return 6
