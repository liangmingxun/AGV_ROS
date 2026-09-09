"""Logical paper tables and deterministic draft figures for one run."""

import math
import statistics
from pathlib import Path

from multi_agv_analysis.io_utils import (
    atomic_dump_json, finite_float, load_yaml, read_csv, write_csv)
from multi_agv_analysis.metrics import _rigid_fit_residual
from multi_agv_analysis.publication_plots import _pyplot, _save_pdf


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


def _run_context(aligned_csv):
    """Infer method/path labels from the recorded run, never from filenames."""
    aligned = Path(aligned_csv).resolve()
    converted = aligned.parent
    run_dir = converted.parent if converted.name == "converted" else None
    method_id = "M1_R1"
    controller = converted / "controller_state.csv"
    if controller.is_file():
        controller_rows = read_csv(controller)
        recorded = next((str(row.get("method_id", "")).strip()
                         for row in controller_rows
                         if str(row.get("method_id", "")).strip()), "")
        if recorded:
            method_id = recorded
    methods = {
        "M1_R1": (
            "M1+R1", "M1动态参考", "R1",
            "M1局部动态上界", "M1公共动态上界"),
        "M2a_R1": (
            "M2a+R1", "M2a固定边界参考", "R1",
            "M2a局部固定上界", "M2a公共固定上界"),
        "M2b_M2b": (
            "M2b", "M2b固定约束参考", "M2b",
            "M2b局部约束上界", "M2b公共约束上界"),
    }
    (method_label, upper_label, lower_label,
     local_upper_label, common_upper_label) = methods.get(
        method_id, (
            method_id, "参考", "执行层", "局部参考上界", "公共参考上界"))

    path_label = "S形路径"
    path_model = "unknown"
    r1_velocity_lower_bound = -0.15
    r1_velocity_upper_bound = 0.58
    if run_dir and (run_dir / "config").is_dir():
        for snapshot in sorted((run_dir / "config").glob("*.yaml")):
            config = load_yaml(snapshot)
            runtime = config.get("formal_fake_runtime", {})
            lower = runtime.get("lower", {})
            recorded_lower = finite_float(lower.get("velocity_lower_bound"))
            recorded_upper = finite_float(lower.get("velocity_upper_bound"))
            if math.isfinite(recorded_lower):
                r1_velocity_lower_bound = recorded_lower
            if math.isfinite(recorded_upper):
                r1_velocity_upper_bound = recorded_upper
            path = config.get("path_s_curve", {})
            model = str(path.get("model", ""))
            if not model:
                continue
            path_model = model
            if model.startswith("circle_"):
                radius = finite_float(path.get("circle_radius"))
                direction = finite_float(path.get("circle_direction"))
                turn = "顺时针" if direction < 0.0 else "逆时针"
                path_label = "R={:.1f} m{}圆形路径".format(radius, turn)
                if model == "circle_smooth_entry_exit":
                    path_label += "（平滑进出）"
            elif "sine" in model:
                path_label = "S形路径"
            else:
                path_label = model
            break
    return {
        "method_id": method_id,
        "method_label": method_label,
        "upper_label": upper_label,
        "lower_label": lower_label,
        "local_upper_label": local_upper_label,
        "common_upper_label": common_upper_label,
        "path_model": path_model,
        "path_label": path_label,
        "r1_velocity_lower_bound": r1_velocity_lower_bound,
        "r1_velocity_upper_bound": r1_velocity_upper_bound,
    }


def _display_smooth(time, values, window_seconds=0.10):
    """Robust centred display trend; source samples and metrics stay intact."""
    finite_time = [value for value in time if math.isfinite(value)]
    intervals = [right - left for left, right in zip(
        finite_time[:-1], finite_time[1:]) if right > left]
    if not intervals or window_seconds <= 0.0:
        return list(values)
    dt = statistics.median(intervals)
    radius = max(1, int(round(window_seconds / dt / 2.0)))
    median_radius = max(1, radius // 2)
    medians = []
    for index in range(len(values)):
        local = [value for value in values[
            max(0, index - median_radius):index + median_radius + 1]
                 if math.isfinite(value)]
        medians.append(statistics.median(local) if local else math.nan)
    trend = []
    for index in range(len(medians)):
        local = [value for value in medians[
            max(0, index - radius):index + radius + 1]
                 if math.isfinite(value)]
        trend.append(sum(local) / len(local) if local else math.nan)
    return trend


def _display_indices(time, maximum_rate_hz=25.0):
    """Return plot-only indices; acquisition and metric samples are untouched."""
    if len(time) <= 2 or maximum_rate_hz <= 0.0:
        return list(range(len(time)))
    finite_time = [value for value in time if math.isfinite(value)]
    intervals = [right - left for left, right in zip(
        finite_time[:-1], finite_time[1:]) if right > left]
    if not intervals:
        return list(range(len(time)))
    source_rate = 1.0 / statistics.median(intervals)
    stride = max(1, int(round(source_rate / maximum_rate_hz)))
    indices = list(range(0, len(time), stride))
    if indices[-1] != len(time) - 1:
        indices.append(len(time) - 1)
    return indices


def _plot_measured(axis, time, values, color, label, style="-",
                   window_seconds=0.10, show_raw=False, linewidth=1.7,
                   maximum_plot_rate_hz=25.0):
    if show_raw:
        axis.plot(time, values, color=color, ls=style, lw=0.55,
                  alpha=0.16, label="_nolegend_")
    trend = _display_smooth(time, values, window_seconds)
    indices = _display_indices(time, maximum_plot_rate_hz)
    plot_time = [time[index] for index in indices]
    plot_trend = [trend[index] for index in indices]
    axis.plot(plot_time, plot_trend, color=color, ls=style, lw=linewidth,
              alpha=0.96, label=label)
    return trend


def _same_series(series, tolerance=1.0e-9):
    if not series:
        return False
    reference = series[0]
    for candidate in series[1:]:
        for left, right in zip(reference, candidate):
            if math.isfinite(left) and math.isfinite(right):
                if abs(left - right) > tolerance:
                    return False
            elif math.isfinite(left) != math.isfinite(right):
                return False
    return True


def _save(fig, output, name):
    fig.tight_layout()
    fig.savefig(output / (name + ".png"), dpi=320, bbox_inches="tight")
    _save_pdf(fig, output / (name + ".pdf"))


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
    context = _run_context(aligned_csv)
    display = (axis_overrides or {}).get("display", {})
    smoothing_window = float(display.get("smoothing_window_seconds", 0.10))
    wheel_feedback_smoothing_window = float(display.get(
        "wheel_feedback_smoothing_window_seconds", 0.80))
    show_raw = bool(display.get("show_raw_samples", False))
    maximum_plot_rate = float(display.get("maximum_plot_rate_hz", 25.0))
    if not math.isfinite(smoothing_window) or smoothing_window < 0.0:
        raise ValueError("invalid display smoothing window")
    if (not math.isfinite(wheel_feedback_smoothing_window) or
            wheel_feedback_smoothing_window < 0.0):
        raise ValueError("invalid wheel feedback display smoothing window")
    if not math.isfinite(maximum_plot_rate) or maximum_plot_rate <= 0.0:
        raise ValueError("invalid maximum display plot rate")
    metadata = {}

    fig, ax = plt.subplots(figsize=(7.0, 5.4))
    reference = _finite_xy(_series(rows, "load_x_reference"),
                           _series(rows, "load_y_reference"))
    if reference:
        ax.plot(*zip(*reference), "k--", label="等效载荷参考路径")
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        raw_x = _series(rows, "agv{}_support_pose_x".format(robot))
        raw_y = _series(rows, "agv{}_support_pose_y".format(robot))
        points = [(index, x, y) for index, (x, y) in enumerate(
            zip(raw_x, raw_y)) if math.isfinite(x) and math.isfinite(y)]
        if points:
            point_time = [time[index] for index, _, _ in points]
            x_values = [x for _, x, _ in points]
            y_values = [y for _, _, y in points]
            if show_raw:
                ax.plot(x_values, y_values, color=color, ls=style,
                        lw=0.5, alpha=0.14, label="_nolegend_")
            trajectory_indices = _display_indices(
                point_time, maximum_plot_rate)
            smooth_x = _display_smooth(
                point_time, x_values, smoothing_window)
            smooth_y = _display_smooth(
                point_time, y_values, smoothing_window)
            ax.plot([smooth_x[index] for index in trajectory_indices],
                    [smooth_y[index] for index in trajectory_indices],
                    color=color, ls=style, lw=1.7,
                    label="Robot{}支撑点".format(robot))
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("{}：{}与三车支撑点轨迹".format(
        context["method_label"], context["path_label"]))
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
    physical_to_plot = []
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        physical = []
        for row in rows:
            left = finite_float(row.get(
                "agv{}_wheel_left_reported_limit".format(robot)))
            right = finite_float(row.get(
                "agv{}_wheel_right_reported_limit".format(robot)))
            physical.append(min(left, right) if math.isfinite(left + right)
                            else math.nan)
        physical_series.append(physical)
        physical_to_plot.append((robot, color, style, physical))
    if _same_series(physical_series):
        axes[0].plot(time, physical_series[0], color="#333333", lw=1.6,
                     label="三车物理轮速上限")
    else:
        for robot, color, style, physical in physical_to_plot:
            axes[0].plot(time, physical, color=color, ls=style,
                         label="Robot{}物理轮速上限".format(robot))
    axes[0].set_title("三车底盘物理轮速能力（轮速域）")
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
            color=color, ls="-.", alpha=0.75,
            label="Robot{} {}".format(
                robot, context["local_upper_label"]))
        path_series.extend((mapped, boundary))
    common_boundary = _series(rows, "common_boundary_upper")
    public_reference = _series(rows, "public_reference_velocity")
    axes[1].plot(time, common_boundary, color="#7A3E9D", ls="--", lw=1.3,
                 label=context["common_upper_label"])
    axes[1].plot(time, public_reference, "k", lw=1.5,
                 label="公共参考速度")
    path_series.extend((common_boundary, public_reference))
    axes[1].set_title("{}：路径域能力与{}边界".format(
        context["method_label"], context["upper_label"]))
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
    public_reference = _series(rows, "public_reference_velocity")
    common_upper = _series(rows, "common_boundary_upper")
    for robot, ax, color in zip(range(1, 4), axes, colors):
        actual = _series(rows, "agv{}_s_dot_actual".format(robot))
        lower = _series(rows, "agv{}_boundary_lower".format(robot))
        upper = _series(rows, "agv{}_boundary_upper".format(robot))
        actual_trend = _plot_measured(
            ax, time, actual, color,
            "融合实测路径速度",
            window_seconds=smoothing_window, show_raw=show_raw,
            maximum_plot_rate_hz=maximum_plot_rate)
        ax.plot(time, public_reference, color="#555555", ls=":", lw=1.15,
                label="公共参考速度")
        ax.plot(time, lower, "k--", lw=0.8,
                label="M1动态参考下界")
        ax.plot(time, upper, color="#D9A900", ls="-.", lw=1.35,
                label="Robot{} M1局部动态上界".format(robot))
        ax.plot(time, common_upper, color="#7A3E9D", ls="--", lw=1.2,
                label="M1公共动态上界")
        ax.set_ylabel("Robot{} / (m/s)".format(robot))
        _legend(ax, 3)
        channel_series.extend(
            (actual, actual_trend, public_reference, lower, upper,
             common_upper))
    for robot, ax in zip(range(1, 4), axes):
        _set_y_axis(ax, channel_series, "figure4.robot{}".format(robot),
                    "m/s", metadata, axis_overrides)
    axes[-1].set_xlabel("时间 / s")
    axes[0].set_title("{}：局部/公共边界、公共参考与实测路径速度".format(
        context["method_label"]))
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
        _plot_measured(
            ax, time, pre_limit, "#9467bd", "限幅前需求",
            window_seconds=smoothing_window, show_raw=show_raw,
            maximum_plot_rate_hz=maximum_plot_rate)
        if any(math.isfinite(a + b) and abs(a - b) > 1.0e-9
               for a, b in zip(pre_limit, fleet_scaled)):
            ax.plot(time, fleet_scaled, color="#8c564b", ls=":",
                    label="车队缩放后需求")
        _plot_measured(
            ax, time, applied, "#ff7f0e", "执行命令", style="--",
            window_seconds=smoothing_window, show_raw=show_raw,
            maximum_plot_rate_hz=maximum_plot_rate)
        _plot_measured(
            ax, time, actual, colors[1], "实际轮速",
            window_seconds=wheel_feedback_smoothing_window,
            show_raw=show_raw,
            maximum_plot_rate_hz=maximum_plot_rate)
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
        if side == "left":
            _legend(ax, 4)
        wheel_axes_series.extend(
            (pre_limit, applied, actual, limit, negative_limit))
    for ax, side in zip(axes, ("left", "right")):
        _set_y_axis(ax, wheel_axes_series, "figure5.{}".format(side),
                    "m/s", metadata, axis_overrides)
    axes[0].set_title(
        "{}：Robot2轮速需求、执行与反馈".format(context["method_label"]))
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
    _plot_measured(
        ax, time, load_error, "#222222", "虚拟等效载荷位置误差",
        window_seconds=smoothing_window, show_raw=show_raw,
        maximum_plot_rate_hz=maximum_plot_rate)
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
        _plot_measured(
            ax, time, errors, color,
            "Robot{}支撑点误差".format(robot), style=style,
            window_seconds=smoothing_window, show_raw=show_raw,
            maximum_plot_rate_hz=maximum_plot_rate)
        error_series.append(errors)
    formation_error = _formation_errors(rows)
    _plot_measured(
        ax, time, formation_error, "#7f3c8d", "构型残差", style="--",
        window_seconds=smoothing_window, show_raw=show_raw,
        maximum_plot_rate_hz=maximum_plot_rate)
    error_series.append(formation_error)
    ax.set_title("{}：等效载荷、支撑点及构型误差".format(
        context["method_label"]))
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
    progress_series = []
    velocity_series = []
    for robot, color, style in zip(range(1, 4), colors, robot_styles):
        s_error = [a - b for a, b in zip(
            _series(rows, "agv{}_s_actual".format(robot)), s_ref)]
        v_error = [a - b for a, b in zip(
            _series(rows, "agv{}_s_dot_actual".format(robot)), v_ref)]
        _plot_measured(
            axes[0], time, s_error, color, "Robot{}".format(robot),
            style=style, window_seconds=smoothing_window,
            show_raw=show_raw, maximum_plot_rate_hz=maximum_plot_rate)
        _plot_measured(
            axes[1], time, v_error, color, "Robot{}".format(robot),
            style=style, window_seconds=smoothing_window,
            show_raw=show_raw, maximum_plot_rate_hz=maximum_plot_rate)
        progress_series.append(s_error)
        velocity_series.append(v_error)
    axes[0].set_title("{}：路径进度误差与速度误差".format(
        context["method_label"]))
    axes[0].set_ylabel("进度误差 / m")
    _legend(axes[0])
    axes[1].set_ylabel("速度误差 / (m/s)")
    axes[1].set_xlabel("时间 / s")
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
        "schema_version": 2,
        "source": str(Path(aligned_csv).resolve()),
        "figures": 6,
        "emergency_pre_limit_threshold_mps": emergency_threshold,
        "run_context": context,
        "display_processing": {
            "raw_samples_preserved": True,
            "metrics_use_raw_samples": True,
            "outliers_removed": False,
            "show_raw_samples": show_raw,
            "robust_centered_trend_window_seconds": smoothing_window,
            "wheel_feedback_trend_window_seconds": (
                wheel_feedback_smoothing_window),
            "maximum_trend_plot_rate_hz": maximum_plot_rate,
        },
        "axes": metadata,
    })
    return 6
