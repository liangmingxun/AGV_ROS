"""Compact Chinese paper figures from Robot1/ROS aligned experiment data."""

import math
from pathlib import Path

from .io_utils import finite_float, load_yaml, read_csv, write_csv


ROBOT_COLORS = ("#1f77b4", "#d62728", "#2ca02c")
METHOD_STYLES = {
    "M1": "-", "M2a": "--", "M2b": "-.",
    "R1": "-", "R2": "--", "R3": "-.",
}


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    preferred = (
        "Noto Sans CJK SC", "SimHei", "Microsoft YaHei",
        "WenQuanYi Zen Hei")
    installed = {font.name for font in font_manager.fontManager.ttflist}
    selected = next((name for name in preferred if name in installed), None)
    if selected is None:
        # Matplotlib's stale user cache may omit fonts that fontconfig can
        # already see. Register common system CJK fonts directly.
        candidates = (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        )
        for candidate in candidates:
            if Path(candidate).is_file():
                properties = font_manager.FontProperties(fname=candidate)
                # FontManager.addfont() is unavailable in Matplotlib 3.1.2.
                # Register through its legacy public font list in that case.
                if hasattr(font_manager.fontManager, "addfont"):
                    font_manager.fontManager.addfont(candidate)
                else:
                    font = font_manager.ft2font.FT2Font(candidate)
                    font_manager.fontManager.ttflist.append(
                        font_manager.ttfFontProperty(font))
                selected = properties.get_name()
                break
    selected = selected or "DejaVu Sans"
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": [selected, "DejaVu Sans"],
        "axes.unicode_minus": False,
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.titleweight": "semibold",
        "axes.labelsize": 10.5,
        "axes.linewidth": 0.8,
        "axes.axisbelow": True,
        "axes.grid": True,
        "grid.alpha": 0.18,
        "grid.linewidth": 0.7,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "legend.frameon": False,
        "savefig.facecolor": "white",
        "figure.facecolor": "white",
        "lines.linewidth": 1.45,
    })
    return plt


def _save_pdf(figure, path):
    """Write PDF, falling back around Matplotlib 3.1.x TTC embedding bugs."""
    try:
        figure.savefig(path, bbox_inches="tight")
        return
    except (RuntimeError, ValueError) as error:
        if "TrueType font is missing table" not in str(error):
            raise
    from matplotlib.backends.backend_cairo import FigureCanvasCairo
    original_canvas = figure.canvas
    try:
        FigureCanvasCairo(figure).print_figure(
            str(path), format="pdf", bbox_inches="tight")
    finally:
        figure.set_canvas(original_canvas)


def _load_rows(source):
    source = Path(source)
    candidates = (
        source / "converted" / "aligned_samples.csv",
        source / "aligned_samples.csv") if source.is_dir() else (source,)
    path = next((item for item in candidates if item.is_file()), None)
    if path is None:
        raise RuntimeError("找不到 aligned_samples.csv: {}".format(source))
    rows = read_csv(path)
    if not rows:
        raise RuntimeError("aligned_samples.csv 没有数据: {}".format(path))
    active = [row for row in rows
              if str(row.get("evaluation_active", "true")).lower()
              in ("1", "true", "yes")]
    rows = active or rows
    stamps = [finite_float(row.get("stamp")) for row in rows]
    origin = next((value for value in stamps if math.isfinite(value)), None)
    if origin is None:
        raise RuntimeError("aligned_samples.csv 没有有效时间戳")
    return rows, [value - origin for value in stamps]


def _series(rows, field):
    return [finite_float(row.get(field)) for row in rows]


def _finite_xy(x_values, y_values):
    return [(x, y) for x, y in zip(x_values, y_values)
            if math.isfinite(x) and math.isfinite(y)]


def _difference_norm(rows, actual_x, actual_y, reference_x, reference_y):
    result = []
    for row in rows:
        dx = finite_float(row.get(actual_x)) - finite_float(row.get(reference_x))
        dy = finite_float(row.get(actual_y)) - finite_float(row.get(reference_y))
        result.append(math.hypot(dx, dy) if math.isfinite(dx + dy)
                      else math.nan)
    return result


def _support_errors(rows):
    return [_difference_norm(
        rows, "agv{}_support_pose_x".format(robot),
        "agv{}_support_pose_y".format(robot),
        "agv{}_support_reference_x".format(robot),
        "agv{}_support_reference_y".format(robot))
        for robot in range(1, 4)]


def _formation_error(rows):
    result = []
    for row in rows:
        actual = [(finite_float(row.get("agv{}_support_pose_x".format(robot))),
                   finite_float(row.get("agv{}_support_pose_y".format(robot))))
                  for robot in range(1, 4)]
        reference = [
            (finite_float(row.get(
                "agv{}_support_reference_x".format(robot))),
             finite_float(row.get(
                "agv{}_support_reference_y".format(robot))))
            for robot in range(1, 4)]
        values = [coordinate for point in actual + reference
                  for coordinate in point]
        if not all(math.isfinite(value) for value in values):
            result.append(math.nan)
            continue
        actual_center = tuple(
            sum(point[axis] for point in actual) / 3.0
            for axis in range(2))
        reference_center = tuple(
            sum(point[axis] for point in reference) / 3.0
            for axis in range(2))
        dot = 0.0
        cross = 0.0
        for measured, expected in zip(actual, reference):
            ax = measured[0] - actual_center[0]
            ay = measured[1] - actual_center[1]
            rx = expected[0] - reference_center[0]
            ry = expected[1] - reference_center[1]
            dot += rx * ax + ry * ay
            cross += rx * ay - ry * ax
        yaw = math.atan2(cross, dot)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        squared = []
        for measured, expected in zip(actual, reference):
            rx = expected[0] - reference_center[0]
            ry = expected[1] - reference_center[1]
            predicted = (
                actual_center[0] + cosine * rx - sine * ry,
                actual_center[1] + sine * rx + cosine * ry)
            squared.append(
                (measured[0] - predicted[0]) ** 2 +
                (measured[1] - predicted[1]) ** 2)
        result.append(math.sqrt(sum(squared) / len(squared)))
    return result


def _wheel_envelope(rows, robot, suffix):
    result = []
    for row in rows:
        values = [finite_float(row.get("agv{}_wheel_{}_{}".format(
            robot, side, suffix))) for side in ("left", "right")]
        result.append(max(abs(value) for value in values)
                      if all(math.isfinite(value) for value in values)
                      else math.nan)
    return result


def _robot2_capability(rows):
    result = []
    for row in rows:
        values = [finite_float(row.get(
            "agv2_wheel_{}_reported_limit".format(side)))
            for side in ("left", "right")]
        result.append(min(values) if all(math.isfinite(v) for v in values)
                      else math.nan)
    return result


def _automatic_derating_span(rows, time):
    capability = _robot2_capability(rows)
    finite = [value for value in capability if math.isfinite(value)]
    if not finite:
        return None
    nominal = max(finite)
    indices = [index for index, value in enumerate(capability)
               if math.isfinite(value) and value < 0.95 * nominal]
    return (time[indices[0]], time[indices[-1]]) if indices else None


def _mark_event(axes, span, label):
    if span is None:
        return
    axes = axes if isinstance(axes, (list, tuple)) else [axes]
    for axis in axes:
        axis.axvspan(span[0], span[1], color="#f2c14e", alpha=0.18,
                     label=label)


def _legend(axis, columns=3):
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(loc="upper center", bbox_to_anchor=(0.5, 1.20),
                    ncol=min(columns, len(handles)), fontsize=8,
                    frameon=False)


def _save(figure, directory, name):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(directory / (name + ".png"), dpi=320,
                   bbox_inches="tight")
    _save_pdf(figure, directory / (name + ".pdf"))


def _plot_boundary_method(axis, rows, time, method, event=None):
    style = METHOD_STYLES[method]
    axis.plot(time, _series(rows, "agv2_mapped_path_velocity_upper"),
              color=ROBOT_COLORS[1], ls=style, label="{} Robot2能力".format(method))
    axis.plot(time, _series(rows, "common_boundary_upper"),
              color="#7f3c8d", ls=style, label="{} 边界".format(method))
    axis.plot(time, _series(rows, "public_reference_velocity"),
              color="#111111", ls=style, label="{} 参考速度".format(method))
    _mark_event(axis, event, "能力退化区间")
    axis.set_ylabel("路径速度 / (m/s)")
    axis.set_title(method)
    _legend(axis)


def _plot_robot2_wheel(axis, rows, time, title, event=None):
    axis.plot(time, _wheel_envelope(rows, 2, "pre_limit"),
              color=ROBOT_COLORS[1], label="需求")
    axis.plot(time, _wheel_envelope(rows, 2, "applied"),
              color="#ff7f0e", ls="--", label="执行命令")
    axis.plot(time, _wheel_envelope(rows, 2, "actual"),
              color="#9467bd", ls="-.", label="实际轮速")
    axis.plot(time, _robot2_capability(rows), color="#222222", ls=":",
              label="轮速限制")
    _mark_event(axis, event, "能力退化区间")
    axis.set_ylabel("|轮速| / (m/s)")
    axis.set_title(title)
    _legend(axis, 4)


def plot_experiment1(source, output_dir):
    """Write the four principal M1+R1 physical-experiment figures."""
    plt = _pyplot()
    rows, time = _load_rows(source)
    output = Path(output_dir)

    fig, axis = plt.subplots(figsize=(7.2, 5.4))
    for xfield, yfield, style, label, color in (
            ("load_x_reference", "load_y_reference", "--", "载荷参考S路径", "#111111"),
            ("load_pose_x", "load_pose_y", "-", "实际等效载荷", "#555555")):
        values = _finite_xy(_series(rows, xfield), _series(rows, yfield))
        if values:
            axis.plot(*zip(*values), ls=style, color=color, label=label)
    for robot, color in zip(range(1, 4), ROBOT_COLORS):
        values = _finite_xy(_series(rows, "agv{}_support_pose_x".format(robot)),
                            _series(rows, "agv{}_support_pose_y".format(robot)))
        if values:
            axis.plot(*zip(*values), color=color,
                      label="Robot{} support".format(robot))
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x / m"); axis.set_ylabel("y / m")
    _legend(axis, 3)
    _save(fig, output, "experiment1_trajectory"); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.4), sharex=True)
    for robot, color in zip(range(1, 4), ROBOT_COLORS):
        axes[0].plot(time, _series(rows, "agv{}_mapped_path_velocity_upper".format(robot)),
                     color=color, label="Robot{}映射能力".format(robot))
        axes[1].plot(time, _series(rows, "agv{}_boundary_upper".format(robot)),
                     color=color, ls=":", label="Robot{}动态边界".format(robot))
    axes[1].plot(time, _series(rows, "common_boundary_upper"), "k--", label="公共边界")
    axes[1].plot(time, _series(rows, "public_reference_velocity"), "k-", label="公共参考速度")
    axes[0].set_ylabel("映射能力 / (m/s)"); axes[1].set_ylabel("路径速度 / (m/s)")
    axes[1].set_xlabel("时间 / s"); _legend(axes[0]); _legend(axes[1])
    _save(fig, output, "experiment1_boundary_velocity"); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.2), sharex=True)
    load_error = _difference_norm(rows, "load_pose_x", "load_pose_y",
                                  "load_x_reference", "load_y_reference")
    axes[0].plot(time, load_error, color="#111111", label="载荷跟踪误差")
    for robot, color, values in zip(range(1, 4), ROBOT_COLORS, _support_errors(rows)):
        axes[0].plot(time, values, color=color, label="Robot{} support误差".format(robot))
    axes[1].plot(time, _formation_error(rows), color="#7f3c8d", label="构型误差")
    axes[0].set_ylabel("位置误差 / m"); axes[1].set_ylabel("构型误差 / m")
    axes[1].set_xlabel("时间 / s"); _legend(axes[0], 4); _legend(axes[1])
    _save(fig, output, "experiment1_tracking_error"); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8.2, 7.4), sharex=True)
    for robot, color, axis in zip(range(1, 4), ROBOT_COLORS, axes):
        axis.plot(time, _wheel_envelope(rows, robot, "pre_limit"), color=color, label="需求")
        axis.plot(time, _wheel_envelope(rows, robot, "applied"), color=color, ls="--", label="执行命令")
        axis.plot(time, _wheel_envelope(rows, robot, "actual"), color=color, ls="-.", label="实际轮速")
        limits = []
        for row in rows:
            pair = [finite_float(row.get("agv{}_wheel_{}_reported_limit".format(robot, side))) for side in ("left", "right")]
            limits.append(min(pair) if all(math.isfinite(v) for v in pair) else math.nan)
        axis.plot(time, limits, color="#222222", ls=":", label="轮速限制")
        axis.set_ylabel("Robot{}\n|轮速| / (m/s)".format(robot)); _legend(axis, 4)
    axes[-1].set_xlabel("时间 / s")
    _save(fig, output, "experiment1_wheel_execution"); plt.close(fig)
    return 4


def _comparison_inputs(sources, event):
    loaded = {}
    for method, source in sources.items():
        rows, time = _load_rows(source)
        loaded[method] = (rows, time)
    if event is None:
        spans = [_automatic_derating_span(rows, time)
                 for rows, time in loaded.values()]
        event = next((span for span in spans if span is not None), None)
    return loaded, event


def _shared_time_limit(loaded):
    maxima = [max((value for value in time if math.isfinite(value)),
                  default=0.0) for _, time in loaded.values()]
    return min(maxima)


def _local_trajectory(plt, loaded, output, name, window):
    fig, axis = plt.subplots(figsize=(7.2, 5.2))
    all_points = []
    for method, (rows, time) in loaded.items():
        indices = range(len(rows)) if window is None else [
            i for i, value in enumerate(time) if window[0] <= value <= window[1]]
        x = _series(rows, "load_pose_x"); y = _series(rows, "load_pose_y")
        points = [(x[i], y[i]) for i in indices
                  if math.isfinite(x[i]) and math.isfinite(y[i])]
        if points:
            all_points.extend(points)
            axis.plot(*zip(*points), ls=METHOD_STYLES[method],
                      label="{} 实际载荷".format(method))
    first_rows, first_time = next(iter(loaded.values()))
    reference_indices = range(len(first_rows)) if window is None else [
        i for i, value in enumerate(first_time)
        if window[0] <= value <= window[1]]
    reference_x = _series(first_rows, "load_x_reference")
    reference_y = _series(first_rows, "load_y_reference")
    reference = [(reference_x[i], reference_y[i]) for i in reference_indices
                 if math.isfinite(reference_x[i]) and
                 math.isfinite(reference_y[i])]
    if reference:
        axis.plot(*zip(*reference), color="#999999", ls=":", label="参考路径")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x / m"); axis.set_ylabel("y / m"); _legend(axis)
    _save(fig, output, name); plt.close(fig)


def plot_experiment2a(m1_source, m2a_source, output_dir,
                      event=None, local_window=None):
    plt = _pyplot(); output = Path(output_dir)
    loaded, event = _comparison_inputs(
        {"M1": m1_source, "M2a": m2a_source}, event)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.5), sharex=False)
    for axis, method in zip(axes, ("M1", "M2a")):
        rows, time = loaded[method]
        _plot_boundary_method(axis, rows, time, method, event)
        axis.set_xlabel("时间 / s")
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    _save(fig, output, "experiment2a_boundary_comparison"); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.5), sharex=False)
    for axis, method in zip(axes, ("M1", "M2a")):
        rows, time = loaded[method]
        _plot_robot2_wheel(axis, rows, time, method, event)
        axis.set_xlabel("时间 / s")
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    _save(fig, output, "experiment2a_robot2_wheel"); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.2), sharex=False)
    for method, (rows, time) in loaded.items():
        style = METHOD_STYLES[method]
        axes[0].plot(time, _formation_error(rows), ls=style, label=method)
        supports = _support_errors(rows)
        maximum = [max(values) if all(math.isfinite(v) for v in values) else math.nan
                   for values in zip(*supports)]
        axes[1].plot(time, maximum, ls=style, label=method)
    for axis in axes:
        _mark_event(axis, event, "能力退化区间"); _legend(axis)
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    axes[0].set_ylabel("构型误差 / m"); axes[1].set_ylabel("最大support误差 / m")
    axes[1].set_xlabel("时间 / s")
    _save(fig, output, "experiment2a_formation_error"); plt.close(fig)
    _local_trajectory(plt, loaded, output, "experiment2a_local_trajectory",
                      local_window or event)
    return 4


def plot_experiment2b(m1_source, m2b_source, output_dir,
                      event=None, local_window=None):
    plt = _pyplot(); output = Path(output_dir)
    loaded, event = _comparison_inputs(
        {"M1": m1_source, "M2b": m2b_source}, event)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.5), sharex=False)
    for axis, method in zip(axes, ("M1", "M2b")):
        rows, time = loaded[method]; _plot_boundary_method(axis, rows, time, method, event)
        axis.set_xlabel("时间 / s")
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    _save(fig, output, "experiment2b_boundary_comparison"); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.2), sharex=False)
    for method, (rows, time) in loaded.items():
        style = METHOD_STYLES[method]
        axes[0].plot(time, _wheel_envelope(rows, 2, "pre_limit"), ls=style,
                     label="{} 轮速需求".format(method))
        axes[0].plot(time, _wheel_envelope(rows, 2, "actual"), ls=style,
                     color="#9467bd", alpha=0.75,
                     label="{} 实际轮速".format(method))
        axes[0].plot(time, _robot2_capability(rows), color="#777777", ls=style,
                     alpha=0.7, label="{} 轮速限制".format(method))
        axes[1].plot(time, _formation_error(rows), ls=style, label=method)
    for axis in axes:
        _mark_event(axis, event, "能力退化区间"); _legend(axis)
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    axes[0].set_ylabel("Robot2 |轮速| / (m/s)"); axes[1].set_ylabel("构型误差 / m")
    axes[1].set_xlabel("时间 / s")
    _save(fig, output, "experiment2b_execution_error"); plt.close(fig)
    _local_trajectory(plt, loaded, output, "experiment2b_local_trajectory",
                      local_window or event)
    return 3


def plot_experiment3(r1_source, r2_source, r3_source, output_dir,
                     disturbance=None):
    plt = _pyplot(); output = Path(output_dir)
    loaded = {method: _load_rows(source) for method, source in (
        ("R1", r1_source), ("R2", r2_source), ("R3", r3_source))}
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.2), sharex=False)
    for method, (rows, time) in loaded.items():
        style = METHOD_STYLES[method]
        load_error = _difference_norm(rows, "load_pose_x", "load_pose_y",
                                      "load_x_reference", "load_y_reference")
        composite = []
        for row in rows:
            values = [abs(finite_float(row.get("agv{}_composite_error".format(robot)))) for robot in range(1, 4)]
            composite.append(max(values) if all(math.isfinite(v) for v in values) else math.nan)
        axes[0].plot(time, load_error, ls=style, label=method)
        axes[1].plot(time, composite, ls=style, label=method)
    for axis in axes:
        _mark_event(axis, disturbance, "扰动区间"); _legend(axis)
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    axes[0].set_ylabel("载荷位置误差 / m"); axes[1].set_ylabel("最大复合误差")
    axes[1].set_xlabel("时间 / s")
    _save(fig, output, "experiment3_tracking_error"); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.2), sharex=False)
    for method, (rows, time) in loaded.items():
        style = METHOD_STYLES[method]
        disturbance_values = []
        control_values = []
        for row in rows:
            d = [abs(finite_float(row.get("agv{}_disturbance_estimate".format(robot)))) for robot in range(1, 4)]
            u = [abs(finite_float(row.get("agv{}_channel_input_raw".format(robot)))) for robot in range(1, 4)]
            disturbance_values.append(max(d) if all(math.isfinite(v) for v in d) else math.nan)
            control_values.append(max(u) if all(math.isfinite(v) for v in u) else math.nan)
        axes[0].plot(time, disturbance_values, ls=style, label=method)
        axes[1].plot(time, control_values, ls=style, label=method)
    for axis in axes:
        _mark_event(axis, disturbance, "扰动区间"); _legend(axis)
        axis.set_xlim(0.0, _shared_time_limit(loaded))
    axes[0].set_ylabel("扰动估计绝对值"); axes[1].set_ylabel("控制输入绝对值")
    axes[1].set_xlabel("时间 / s")
    _save(fig, output, "experiment3_disturbance_response"); plt.close(fig)
    return 2


def write_statistics(entries, output_path, experiment):
    """Write a compact CSV; callers may include M4/R4 without plotting them."""
    rows = []
    for method, metrics_path in entries:
        metrics = load_yaml(metrics_path)
        geometry = metrics.get("geometry", {})
        support = geometry.get("support", {})
        support_max = max((finite_float(value.get("position_max"))
                           for value in support.values()), default=math.nan)
        recovery = metrics.get("recovery", {})
        recovery_time = (
            recovery.get("common_velocity_after_capability_delay")
            if experiment == "experiment2a"
            else recovery.get("disturbance_recovery_time"))
        row = {
            "experiment": experiment,
            "method": method,
            "task_completion_time_s": metrics.get("task", {}).get("completion_time"),
            "load_position_rmse_m": geometry.get("load_position_rmse"),
            "formation_error_max_m": geometry.get("rigid_fit_residual_max"),
            "support_error_max_m": support_max,
            "peak_wheel_demand_mps": metrics.get("wheel", {}).get("maximum_pre_limit_demand"),
            "wheel_limited_time_s": metrics.get("wheel", {}).get("limited_time"),
            "recovery_time_s": recovery_time,
            "peak_error": metrics.get("path", {}).get("progress_max_absolute"),
            "error_rmse": metrics.get("path", {}).get("progress_rmse"),
            "steady_state_error": metrics.get("path", {}).get(
                "steady_state_error"),
            "peak_control_input": metrics.get("internal", {}).get("channel_input_max_absolute"),
        }
        rows.append(row)
    write_csv(output_path, rows)
    return len(rows)
