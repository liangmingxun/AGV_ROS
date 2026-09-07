"""Logical paper tables and deterministic draft figures for one run."""

import math
from pathlib import Path

from multi_agv_analysis.io_utils import (
    atomic_dump_json, finite_float, load_yaml, read_csv, write_csv)
from multi_agv_analysis.metrics import _rigid_fit_residual
from multi_agv_analysis.publication_plots import _pyplot


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
    fig.savefig(output / (name + ".png"), dpi=320, bbox_inches="tight")
    fig.savefig(output / (name + ".pdf"), bbox_inches="tight")


def _command_rows(rows):
    """Drop terminal fail-zero rows whose support references are cleared."""
    selected = []
    for row in rows:
        reference = [
            (finite_float(row.get(
                "agv{}_support_reference_x".format(robot))),
             finite_float(row.get(
                "agv{}_support_reference_y".format(robot))))
            for robot in range(1, 4)]
        distances = [math.hypot(
            reference[first][0] - reference[second][0],
            reference[first][1] - reference[second][1])
            for first, second in ((0, 1), (0, 2), (1, 2))]
        if (all(math.isfinite(value) and value > 1.0e-6
                for value in distances) and
                str(row.get("localization_valid", "true")).lower()
                in ("1", "true", "yes")):
            selected.append(row)
    return selected or rows


def _nice_wide_limits(values, absolute):
    """Return padded, rounded limits with roughly five to seven ticks."""
    lower = min(values)
    upper = max(values)
    if absolute:
        padded_lower = 0.0
        padded_upper = max(upper, 0.0) * 1.20
        if padded_upper <= 0.0:
            padded_upper = 1.0
    else:
        span = upper - lower
        if span <= 1.0e-12:
            span = max(abs(upper), 1.0) * 0.1
        padded_lower = lower - 0.20 * span
        padded_upper = upper + 0.20 * span

    raw_step = (padded_upper - padded_lower) / 5.0
    exponent = math.floor(math.log10(raw_step))
    candidates = []
    for shift in (-1, 0, 1):
        scale = 10.0 ** (exponent + shift)
        candidates.extend(value * scale for value in (1.0, 2.0, 2.5, 3.0,
                                                        4.0, 5.0, 10.0))
    choices = []
    for step in sorted(set(candidates)):
        rounded_lower = (0.0 if absolute else
                         math.floor(padded_lower / step) * step)
        rounded_upper = math.ceil(padded_upper / step) * step
        intervals = int(round((rounded_upper - rounded_lower) / step))
        if 4 <= intervals <= 6:
            choices.append((rounded_upper - rounded_lower, step,
                            rounded_lower, rounded_upper))
    if choices:
        _, step, lower, upper = min(choices)
        return lower, upper, step
    return padded_lower, padded_upper, None


def _set_y_axis(axis, series, key, unit, metadata, overrides,
                absolute=False, wide=False):
    from matplotlib.ticker import MaxNLocator, MultipleLocator

    values = [value for group in series for value in group
              if math.isfinite(value)]
    if not values:
        return
    automatic_step = None
    if wide:
        lower, upper, automatic_step = _nice_wide_limits(values, absolute)
    else:
        lower = min(values)
        upper = max(values)
        if absolute:
            lower = 0.0
            upper = max(upper, 0.0)
            upper = upper * 1.08 if upper > 0.0 else 1.0
        else:
            span = upper - lower
            if span <= 1.0e-12:
                span = max(abs(upper), 1.0) * 0.1
            lower -= 0.08 * span
            upper += 0.08 * span
    override = (overrides or {}).get(key, {})
    lower = float(override.get("y_min", lower))
    upper = float(override.get("y_max", upper))
    if not math.isfinite(lower + upper) or lower >= upper:
        raise ValueError("invalid y-axis override for {}".format(key))
    axis.set_ylim(lower, upper)
    step = override.get("y_tick_step", automatic_step)
    if step is not None:
        step = float(step)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("invalid y_tick_step for {}".format(key))
        axis.yaxis.set_major_locator(MultipleLocator(step))
    else:
        axis.yaxis.set_major_locator(MaxNLocator(nbins=6, min_n_ticks=5))
    metadata[key] = {
        "unit": unit,
        "y_min": lower,
        "y_max": upper,
        "absolute_error_from_zero": bool(absolute),
    }


def _record_ticks(figure, axes, metadata):
    figure.canvas.draw()
    for key, axis in axes.items():
        if key not in metadata:
            continue
        lower, upper = axis.get_ylim()
        ticks = [float(value) for value in axis.get_yticks()
                 if lower - 1.0e-12 <= value <= upper + 1.0e-12]
        metadata[key]["ticks"] = ticks


def _legend(axis, columns=3):
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        # Keep legends outside the plotting rectangle so neither data nor the
        # Chinese title is covered. ``columns`` is retained for API stability.
        axis.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0),
                    ncol=1, fontsize=8,
                    frameon=False)


def _formation_errors(rows):
    result = []
    for row in rows:
        actual = [(finite_float(row.get(
            "agv{}_support_pose_x".format(robot))),
                   finite_float(row.get(
            "agv{}_support_pose_y".format(robot))))
                  for robot in range(1, 4)]
        reference = [(finite_float(row.get(
            "agv{}_support_reference_x".format(robot))),
                      finite_float(row.get(
            "agv{}_support_reference_y".format(robot))))
                     for robot in range(1, 4)]
        result.append(_rigid_fit_residual(actual, reference))
    return result


def _emergency_threshold_from_snapshot(aligned_csv):
    aligned = Path(aligned_csv).resolve()
    run_dir = aligned.parent.parent if aligned.parent.name == "converted" else None
    config_dir = run_dir / "config" if run_dir else None
    if not config_dir or not config_dir.is_dir():
        return None
    for path in sorted(config_dir.glob("*.yaml")):
        config = load_yaml(path)
        runtime = config.get("formal_fake_runtime", {})
        value = finite_float(runtime.get("emergency_abort_limit"))
        if math.isfinite(value) and value > 0.0:
            return value
    return None


def plot_run(aligned_csv, output_dir, axis_overrides=None):
    plt = _pyplot()

    rows = read_csv(aligned_csv)
    if not rows:
        raise RuntimeError("aligned input contains no samples")
    rows = _command_rows(rows)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    stamps = _series(rows, "stamp")
    finite_stamps = [value for value in stamps if math.isfinite(value)]
    if not finite_stamps:
        raise RuntimeError("aligned input has no finite timestamps")
    origin = finite_stamps[0]
    time = [value - origin for value in stamps]
    colors = ("#1f77b4", "#d62728", "#2ca02c")
    robot_styles = ("-", "--", "-.")
    emergency_threshold = _emergency_threshold_from_snapshot(aligned_csv)
    metadata = {}

    fig, ax = plt.subplots(figsize=(7.0, 5.4))
    reference = _finite_xy(_series(rows, "load_x_reference"),
                           _series(rows, "load_y_reference"))
    if reference:
        ax.plot(*zip(*reference), "k--", label="等效载荷参考路径")
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        values = _finite_xy(
            _series(rows, "agv{}_support_pose_x".format(robot)),
            _series(rows, "agv{}_support_pose_y".format(robot)))
        if values:
            ax.plot(*zip(*values), color=color, ls=style,
                    label="Robot{}支撑点".format(robot))
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("S路径与三车支撑点轨迹")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    _legend(ax, 2)
    fig.canvas.draw()
    trajectory_lower, trajectory_upper = ax.get_ylim()
    metadata["figure2.trajectory_y"] = {
        "unit": "m",
        "y_min": trajectory_lower,
        "y_max": trajectory_upper,
        "ticks": [float(value) for value in ax.get_yticks()
                  if trajectory_lower <= value <= trajectory_upper],
        "axis_equal": True,
    }
    _save(fig, output, "figure2_trajectory")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.5), sharex=True)
    physical_series = []
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        physical = []
        for row in rows:
            left = finite_float(row.get(
                "agv{}_wheel_left_reported_limit".format(robot)))
            right = finite_float(row.get(
                "agv{}_wheel_right_reported_limit".format(robot)))
            physical.append(min(left, right) if math.isfinite(left + right)
                            else math.nan)
        axes[0].plot(time, physical, color=color, ls=style,
                     label="Robot{}物理轮速上限".format(robot))
        physical_series.append(physical)
    axes[0].set_title("底盘物理轮速能力（轮速域）")
    axes[0].set_ylabel("轮速 / (m/s)")
    _legend(axes[0])
    _set_y_axis(axes[0], physical_series, "figure3.wheel_capability",
                "m/s", metadata, axis_overrides, absolute=True)
    path_series = []
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        mapped = _series(
            rows, "agv{}_mapped_path_velocity_upper".format(robot))
        boundary = _series(rows, "agv{}_boundary_upper".format(robot))
        axes[1].plot(time, mapped,
            color=color, ls=style, alpha=0.8,
            label="Robot{}路径映射能力".format(robot))
        axes[1].plot(time, boundary,
            color=color, ls=":", alpha=0.65,
            label="Robot{} M1动态上界".format(robot))
        path_series.extend((mapped, boundary))
    common_boundary = _series(rows, "common_boundary_upper")
    public_reference = _series(rows, "public_reference_velocity")
    axes[1].plot(time, common_boundary, "k--", lw=1.0,
                 label="M1公共动态边界")
    axes[1].plot(time, public_reference, "k", lw=1.5,
                 label="公共参考速度")
    path_series.extend((common_boundary, public_reference))
    axes[1].set_title("路径域能力与M1动态边界（非R1固定状态约束域）")
    axes[1].set_xlabel("时间 / s")
    axes[1].set_ylabel("路径速度 / (m/s)")
    _legend(axes[1], 4)
    _set_y_axis(axes[1], path_series, "figure3.path_capability",
                "m/s", metadata, axis_overrides, absolute=True)
    _record_ticks(fig, {
        "figure3.wheel_capability": axes[0],
        "figure3.path_capability": axes[1]}, metadata)
    _save(fig, output, "figure3_capability_boundary_reference")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.0), sharex=True)
    channel_series = []
    for robot, ax, color in zip(range(1, 4), axes, colors):
        actual = _series(rows, "agv{}_s_dot_actual".format(robot))
        lower = _series(rows, "agv{}_boundary_lower".format(robot))
        upper = _series(rows, "agv{}_boundary_upper".format(robot))
        ax.plot(time, actual, color=color, label="实际通道速度")
        ax.plot(time, lower, "k--", lw=0.8, label="M1动态下界")
        ax.plot(time, upper, "k-.", lw=0.8, label="M1动态上界")
        ax.set_ylabel("Robot{} / (m/s)".format(robot))
        _legend(ax)
        channel_series.extend((actual, lower, upper))
    for robot, ax in zip(range(1, 4), axes):
        _set_y_axis(ax, channel_series, "figure4.robot{}".format(robot),
                    "m/s", metadata, axis_overrides)
    axes[0].set_title("三车通道速度及M1动态上下界")
    axes[-1].set_xlabel("时间 / s")
    _record_ticks(fig, {
        "figure4.robot{}".format(robot): axes[robot - 1]
        for robot in range(1, 4)}, metadata)
    _save(fig, output, "figure4_channel_constraints")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True)
    wheel_axes_series = []
    for ax, side, side_cn in zip(axes, ("left", "right"), ("左轮", "右轮")):
        prefix = "agv2_wheel_{}".format(side)
        pre_limit = _series(rows, prefix + "_pre_limit")
        fleet_scaled = _series(rows, prefix + "_fleet_scaled")
        applied = _series(rows, prefix + "_applied")
        actual = _series(rows, prefix + "_actual")
        limit = _series(rows, prefix + "_reported_limit")
        ax.plot(time, pre_limit, color="#9467bd", label="限幅前需求")
        if any(math.isfinite(a + b) and abs(a - b) > 1.0e-9
               for a, b in zip(pre_limit, fleet_scaled)):
            ax.plot(time, fleet_scaled, color="#8c564b", ls=":",
                    label="车队缩放后需求")
        ax.plot(time, applied, color="#ff7f0e", ls="--", label="执行命令")
        ax.plot(time, actual, color=colors[1], label="实际轮速")
        ax.plot(time, limit, "k--", lw=0.9, label="物理上限")
        negative_limit = [-value if math.isfinite(value) else math.nan
                          for value in limit]
        ax.plot(time, negative_limit, "k--", lw=0.9)
        if emergency_threshold is not None:
            emergency = [emergency_threshold] * len(time)
            negative_emergency = [-emergency_threshold] * len(time)
            ax.plot(time, emergency, color="#555555", ls=":", lw=0.9,
                    label="pre-limit紧急阈值")
            ax.plot(time, negative_emergency, color="#555555", ls=":",
                    lw=0.9)
            wheel_axes_series.extend((emergency, negative_emergency))
        ax.set_ylabel("{} / (m/s)".format(side_cn))
        _legend(ax, 4)
        wheel_axes_series.extend(
            (pre_limit, applied, actual, limit, negative_limit))
    for ax, side in zip(axes, ("left", "right")):
        _set_y_axis(ax, wheel_axes_series, "figure5.{}".format(side),
                    "m/s", metadata, axis_overrides)
    axes[0].set_title(
        "Robot2轮速执行链（物理限幅与pre-limit紧急阈值语义分离）")
    axes[-1].set_xlabel("时间 / s")
    _record_ticks(fig, {
        "figure5.left": axes[0], "figure5.right": axes[1]}, metadata)
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
    ax.plot(time, load_error, "k", label="虚拟等效载荷位置误差")
    error_series = [load_error]
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
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
        ax.plot(time, errors, color=color, ls=style,
                label="Robot{}支撑点误差".format(robot))
        error_series.append(errors)
    formation_error = _formation_errors(rows)
    ax.plot(time, formation_error, color="#7f3c8d", ls="--",
            label="构型残差")
    error_series.append(formation_error)
    ax.set_title("等效载荷、支撑点及构型误差")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("绝对误差 / m")
    _legend(ax, 3)
    _set_y_axis(ax, error_series, "figure6.position_error", "m",
                metadata, axis_overrides, absolute=True, wide=True)
    _record_ticks(fig, {"figure6.position_error": ax}, metadata)
    _save(fig, output, "figure6_cooperative_errors")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.0), sharex=True)
    s_ref = _series(rows, "load_s_reference")
    v_ref = _series(rows, "load_velocity_reference")
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        s_error = [a - b for a, b in zip(
            _series(rows, "agv{}_s_actual".format(robot)), s_ref)]
        v_error = [a - b for a, b in zip(
            _series(rows, "agv{}_s_dot_actual".format(robot)), v_ref)]
        axes[0].plot(time, s_error, color=color, ls=style,
                     label="Robot{}".format(robot))
        axes[1].plot(time, v_error, color=color, ls=style,
                     label="Robot{}".format(robot))
    axes[0].set_title("路径进度误差与速度误差")
    axes[0].set_ylabel("进度误差 / m")
    _legend(axes[0])
    axes[1].set_ylabel("速度误差 / (m/s)")
    axes[1].set_xlabel("时间 / s")
    _legend(axes[1])
    progress_series = [line.get_ydata() for line in axes[0].lines]
    velocity_series = [line.get_ydata() for line in axes[1].lines]
    _set_y_axis(axes[0], progress_series, "figure7.progress_error", "m",
                metadata, axis_overrides, wide=True)
    _set_y_axis(axes[1], velocity_series, "figure7.velocity_error", "m/s",
                metadata, axis_overrides, wide=True)
    _record_ticks(fig, {
        "figure7.progress_error": axes[0],
        "figure7.velocity_error": axes[1]}, metadata)
    _save(fig, output, "figure7_progress_recovery")
    plt.close(fig)
    atomic_dump_json(output / "plot_metadata.json", {
        "schema_version": 1,
        "source": str(Path(aligned_csv).resolve()),
        "figures": 6,
        "emergency_pre_limit_threshold_mps": emergency_threshold,
        "axes": metadata,
    })
    return 6
