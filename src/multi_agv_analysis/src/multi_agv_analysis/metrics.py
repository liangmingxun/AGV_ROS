"""Causal paper metrics over the aligned, unfiltered experiment samples."""

import math

from .io_utils import bool_value, finite_float


WHEELS = tuple(
    (robot, side) for robot in range(1, 4) for side in ("left", "right"))


def _finite(values):
    return [value for value in values if math.isfinite(value)]


def _rmse(values):
    values = _finite(values)
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def _maximum_absolute(values):
    values = _finite(values)
    return max((abs(value) for value in values), default=None)


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _rigid_fit_residual(actual, reference):
    if len(actual) != 3 or len(reference) != 3:
        return math.nan
    values = [coordinate for point in actual + reference
              for coordinate in point]
    if not all(math.isfinite(value) for value in values):
        return math.nan
    actual_center = (
        sum(point[0] for point in actual) / 3.0,
        sum(point[1] for point in actual) / 3.0)
    reference_center = (
        sum(point[0] for point in reference) / 3.0,
        sum(point[1] for point in reference) / 3.0)
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
    return math.sqrt(sum(squared) / len(squared))


def _first_time(rows, predicate, not_before=-math.inf):
    for row in rows:
        stamp = finite_float(row.get("stamp"))
        if stamp >= not_before and predicate(row):
            return stamp
    return None


def _mapped_path_capability(row, robot):
    """Return only a genuinely recorded CapabilityMapper path bound.

    New runs expose it explicitly. Historical M2b debug also stored the
    mapper input under ``reported_capability``. Historical M1/M2a debug did
    not store it, so no dynamic-boundary fallback is permitted.
    """
    value = finite_float(row.get(
        "agv{}_mapped_path_velocity_upper".format(robot)))
    if math.isfinite(value):
        return value
    value = finite_float(row.get(
        "m2b_agv{}_reported_capability".format(robot)))
    if math.isfinite(value):
        return value
    return math.nan


def compute_metrics(rows, sample_period, command_epsilon=1e-6,
                    nominal_agv2_capability=None,
                    nominal_common_velocity=None,
                    evaluation_target=None):
    """Compute frozen Task-17 metrics without acausal filtering.

    Each input row is one aligned causal sample.  A duration indicator counts
    a sample once when *any* wheel satisfies the condition, matching the
    discrete definitions in section 19.1. Missing/non-finite values are
    excluded and reported through ``sample_counts``.
    """
    all_rows = list(rows)
    if not all_rows:
        raise ValueError("metrics require at least one aligned sample")
    if any(not math.isfinite(finite_float(row.get("stamp")))
           for row in all_rows):
        raise ValueError("every aligned sample must have a finite stamp")
    all_rows.sort(key=lambda row: finite_float(row.get("stamp")))
    rows = [
        row for row in all_rows
        if bool_value(row.get("evaluation_active", True))]
    if not rows:
        raise ValueError("metrics require at least one evaluation sample")
    if not math.isfinite(sample_period) or sample_period <= 0.0:
        raise ValueError("sample_period must be positive and finite")
    if not math.isfinite(command_epsilon) or command_epsilon < 0.0:
        raise ValueError("command_epsilon must be non-negative and finite")
    if (nominal_common_velocity is not None and
            (not math.isfinite(float(nominal_common_velocity)) or
             float(nominal_common_velocity) <= 0.0)):
        raise ValueError(
            "nominal_common_velocity must be positive and finite")
    if (evaluation_target is not None and
            (not math.isfinite(float(evaluation_target)) or
             float(evaluation_target) < 0.0)):
        raise ValueError("evaluation_target must be non-negative and finite")

    demand_samples = 0
    limited_samples = 0
    maximum_ratio = 0.0
    maximum_exceedance = 0.0
    ratios_seen = 0
    fleet_scales = []
    fleet_scaling_samples = 0
    wheel_errors = []
    path_errors = []
    path_velocity_errors = {robot: [] for robot in range(1, 4)}
    path_progress_errors = {robot: [] for robot in range(1, 4)}
    support_errors = {robot: [] for robot in range(1, 4)}
    load_position_errors = []
    load_yaw_errors = []
    rigid_residuals = []
    link_margins = []
    demand_margins = []
    limiter_samples = {"speed": 0, "acceleration": 0, "deceleration": 0}
    internal = {
        "psi": [], "composite_error": [], "theta_hat": [],
        "disturbance_estimate": [], "m2b_delta_z": [],
        "m2b_delta_w": []}

    for row in rows:
        fleet_scale = finite_float(row.get("fleet_scale"))
        if math.isfinite(fleet_scale):
            fleet_scales.append(fleet_scale)
            fleet_scaling_samples += int(fleet_scale < 1.0 - command_epsilon)
        demanded = False
        limited = False
        limiter_active = {name: False for name in limiter_samples}
        for robot, side in WHEELS:
            prefix = "agv{}_wheel_{}".format(robot, side)
            pre_limit = finite_float(row.get(prefix + "_pre_limit"))
            fleet_scaled = finite_float(row.get(prefix + "_fleet_scaled"))
            applied = finite_float(row.get(prefix + "_applied"))
            actual = finite_float(row.get(prefix + "_actual"))
            limit = finite_float(row.get(prefix + "_reported_limit"))
            if (math.isfinite(pre_limit) and math.isfinite(limit) and
                    limit > 0.0):
                ratio = abs(pre_limit) / limit
                maximum_ratio = max(maximum_ratio, ratio)
                maximum_exceedance = max(
                    maximum_exceedance, max(0.0, abs(pre_limit) - limit))
                ratios_seen += 1
                demanded = demanded or abs(pre_limit) > limit
            if math.isfinite(pre_limit) and math.isfinite(applied):
                limited = limited or abs(
                    applied - pre_limit) > command_epsilon
            if math.isfinite(actual) and math.isfinite(applied):
                wheel_errors.append(actual - applied)
            limiter_active["speed"] |= bool_value(
                row.get(prefix + "_speed_limit_active", False))
            limiter_active["acceleration"] |= bool_value(
                row.get(prefix + "_accel_limit_active", False))
            limiter_active["deceleration"] |= bool_value(
                row.get(prefix + "_decel_limit_active", False))
        demand_samples += int(demanded)
        limited_samples += int(limited)
        for name, active in limiter_active.items():
            limiter_samples[name] += int(active)

        actual_progress = finite_float(row.get("load_s_actual"))
        reference_progress = finite_float(row.get("load_s_reference"))
        if math.isfinite(actual_progress) and math.isfinite(reference_progress):
            path_errors.append(actual_progress - reference_progress)
        actual_supports = []
        reference_supports = []
        for robot in range(1, 4):
            actual_s = finite_float(row.get("agv{}_s_actual".format(robot)))
            actual_v = finite_float(
                row.get("agv{}_s_dot_actual".format(robot)))
            reference_s = reference_progress
            reference_v = finite_float(row.get("load_velocity_reference"))
            if math.isfinite(actual_s) and math.isfinite(reference_s):
                path_progress_errors[robot].append(actual_s - reference_s)
            if math.isfinite(actual_v) and math.isfinite(reference_v):
                path_velocity_errors[robot].append(actual_v - reference_v)
            actual = (
                finite_float(row.get(
                    "agv{}_support_pose_x".format(robot))),
                finite_float(row.get(
                    "agv{}_support_pose_y".format(robot))))
            reference = (
                finite_float(row.get(
                    "agv{}_support_reference_x".format(robot))),
                finite_float(row.get(
                    "agv{}_support_reference_y".format(robot))))
            actual_supports.append(actual)
            reference_supports.append(reference)
            if all(math.isfinite(value) for value in actual + reference):
                support_errors[robot].append(math.hypot(
                    actual[0] - reference[0], actual[1] - reference[1]))
            for name in ("psi", "composite_error", "disturbance_estimate"):
                value = finite_float(row.get(
                    "agv{}_{}".format(robot, name)))
                if math.isfinite(value):
                    internal[name].append(value)
            for index in (1, 2):
                value = finite_float(row.get(
                    "agv{}_theta_hat_{}".format(robot, index)))
                if math.isfinite(value):
                    internal["theta_hat"].append(value)
        rigid = _rigid_fit_residual(actual_supports, reference_supports)
        if math.isfinite(rigid):
            rigid_residuals.append(rigid)
        load_actual = (
            finite_float(row.get("load_pose_x")),
            finite_float(row.get("load_pose_y")))
        load_reference = (
            finite_float(row.get("load_x_reference")),
            finite_float(row.get("load_y_reference")))
        if all(math.isfinite(value)
               for value in load_actual + load_reference):
            load_position_errors.append(math.hypot(
                load_actual[0] - load_reference[0],
                load_actual[1] - load_reference[1]))
        load_yaw = finite_float(row.get("load_pose_yaw"))
        load_yaw_reference = finite_float(row.get("load_yaw_reference"))
        if math.isfinite(load_yaw) and math.isfinite(load_yaw_reference):
            load_yaw_errors.append(
                _wrap_angle(load_yaw - load_yaw_reference))
        mapped = [
            _mapped_path_capability(row, robot)
            for robot in range(1, 4)]
        common_upper = finite_float(row.get("common_boundary_upper"))
        if all(math.isfinite(value) for value in mapped) and math.isfinite(
                common_upper):
            link_margins.append(min(mapped) - common_upper)
            demands = [
                finite_float(row.get(
                    "m2b_agv{}_upsilon".format(robot)))
                for robot in range(1, 4)]
            if not all(math.isfinite(value) for value in demands):
                common = finite_float(row.get("common_velocity_reference"))
                demands = [common] * 3
            if all(math.isfinite(value) for value in demands):
                demand_margins.append(min(
                    mapped[index] - abs(demands[index])
                    for index in range(3)))
        for name in ("m2b_delta_z", "m2b_delta_w"):
            value = finite_float(row.get(name))
            if math.isfinite(value):
                internal[name].append(value)

    capability_recovery_time = None
    common_velocity_recovery_time = None
    recovery_delay = None
    capability_recovery_stamp = None
    velocity_recovery_stamp = None
    derating_observed_stamp = None
    first_stamp = finite_float(rows[0].get("stamp"))

    if (nominal_agv2_capability is not None and
            float(nominal_agv2_capability) > 0.0):
        nominal_agv2_capability = float(nominal_agv2_capability)
        threshold = 0.95 * nominal_agv2_capability
        derating_observed_stamp = _first_time(
            rows,
            lambda row: _mapped_path_capability(row, 2) < threshold)
        if derating_observed_stamp is not None:
            capability_recovery_stamp = _first_time(
                rows,
                lambda row: _mapped_path_capability(row, 2) >= threshold,
                not_before=derating_observed_stamp)
            if capability_recovery_stamp is not None:
                capability_recovery_time = (
                    capability_recovery_stamp - first_stamp)

    if (capability_recovery_stamp is not None and
            nominal_common_velocity is not None):
        velocity_threshold = 0.95 * float(nominal_common_velocity)
        velocity_recovery_stamp = _first_time(
            rows,
            lambda row: finite_float(
                row.get("common_velocity_reference")) >= velocity_threshold,
            not_before=capability_recovery_stamp)
        if velocity_recovery_stamp is not None:
            common_velocity_recovery_time = (
                velocity_recovery_stamp - first_stamp)
            recovery_delay = (
                velocity_recovery_stamp - capability_recovery_stamp)

    task_time = None
    task_start_stamp = None
    task_completion_stamp = None
    if evaluation_target is not None:
        target = float(evaluation_target)
        task_start_stamp = finite_float(all_rows[0].get("stamp"))
        task_completion_stamp = _first_time(
            all_rows,
            lambda row: finite_float(row.get("load_s_actual")) >= target)
        if task_completion_stamp is not None:
            task_time = task_completion_stamp - task_start_stamp

    invalid_samples = sum(
        not bool_value(row.get("localization_valid", True)) for row in rows)
    experiment_state_samples = sum(
        bool_value(row.get("experiment_state_available", False))
        for row in all_rows)
    if experiment_state_samples == len(all_rows):
        evaluation_source = "experiment_state"
    elif experiment_state_samples:
        evaluation_source = "partial_experiment_state"
    else:
        evaluation_source = "all_samples_fallback"
    active_indices = [
        index for index, row in enumerate(all_rows)
        if bool_value(row.get("evaluation_active", False))]
    bounded_window = bool(active_indices)
    if bounded_window:
        bounded_window = (
            active_indices[0] > 0 and
            active_indices[-1] < len(all_rows) - 1 and
            active_indices == list(range(
                active_indices[0], active_indices[-1] + 1)))
    return {
        "schema_version": 2,
        "causal_unfiltered": True,
        "sample_period": sample_period,
        "sample_counts": {
            "aligned": len(rows),
            "recorded": len(all_rows),
            "wheel_errors": len(wheel_errors),
            "path_errors": len(path_errors),
            "invalid_localization": invalid_samples,
            "experiment_state": experiment_state_samples,
        },
        "evaluation_window": {
            "source": evaluation_source,
            "bounded_and_contiguous": bounded_window,
            "formal_statistics_ready":
                evaluation_source == "experiment_state" and bounded_window,
        },
        "wheel": {
            "demand_exceedance_samples": demand_samples,
            "demand_exceedance_time": demand_samples * sample_period,
            "limited_samples": limited_samples,
            "limited_time": limited_samples * sample_period,
            "maximum_demand_ratio": (
                maximum_ratio if ratios_seen else None),
            "maximum_demand_exceedance": (
                maximum_exceedance if ratios_seen else None),
            "demand_source": "wheel_pre_limit",
            "minimum_fleet_scale": (
                min(fleet_scales) if fleet_scales else None),
            "fleet_scaling_duration": (
                fleet_scaling_samples * sample_period),
            "fleet_scaling_ratio": (
                float(fleet_scaling_samples) / len(fleet_scales)
                if fleet_scales else None),
            "tracking_rmse": _rmse(wheel_errors),
            "tracking_max_absolute": _maximum_absolute(wheel_errors),
            "speed_limited_time":
                limiter_samples["speed"] * sample_period,
            "acceleration_limited_time":
                limiter_samples["acceleration"] * sample_period,
            "deceleration_limited_time":
                limiter_samples["deceleration"] * sample_period,
        },
        "path": {
            "progress_rmse": _rmse(path_errors),
            "progress_max_absolute": _maximum_absolute(path_errors),
            "per_robot": {
                "agv{}".format(robot): {
                    "progress_rmse": _rmse(path_progress_errors[robot]),
                    "progress_max_absolute":
                        _maximum_absolute(path_progress_errors[robot]),
                    "velocity_rmse": _rmse(path_velocity_errors[robot]),
                    "velocity_max_absolute":
                        _maximum_absolute(path_velocity_errors[robot]),
                } for robot in range(1, 4)},
        },
        "geometry": {
            "support": {
                "agv{}".format(robot): {
                    "position_rmse": _rmse(support_errors[robot]),
                    "position_max": _maximum_absolute(
                        support_errors[robot]),
                } for robot in range(1, 4)},
            "load_position_rmse": _rmse(load_position_errors),
            "load_position_max": _maximum_absolute(load_position_errors),
            "load_yaw_rmse": _rmse(load_yaw_errors),
            "load_yaw_max_absolute": _maximum_absolute(load_yaw_errors),
            "rigid_fit_residual_rmse": _rmse(rigid_residuals),
            "rigid_fit_residual_max": _maximum_absolute(rigid_residuals),
        },
        "capability": {
            "link_margin_minimum": (
                min(link_margins) if link_margins else None),
            "demand_margin_minimum": (
                min(demand_margins) if demand_margins else None),
            "mapped_path_capability_available": bool(link_margins),
        },
        "internal": {
            "psi_max_absolute": _maximum_absolute(internal["psi"]),
            "composite_error_max_absolute":
                _maximum_absolute(internal["composite_error"]),
            "theta_hat_max_absolute":
                _maximum_absolute(internal["theta_hat"]),
            "disturbance_estimate_max":
                max(internal["disturbance_estimate"], default=None),
            "m2b_delta_z_max":
                max(internal["m2b_delta_z"], default=None),
            "m2b_delta_w_max":
                max(internal["m2b_delta_w"], default=None),
        },
        "recovery": {
            "capability_source":
                "explicit_mapper_output_or_historical_m2b_mapper_input",
            "derating_observed_stamp": derating_observed_stamp,
            "capability_95_stamp": capability_recovery_stamp,
            "capability_95_time": capability_recovery_time,
            "common_velocity_95_stamp": velocity_recovery_stamp,
            "common_velocity_95_time": common_velocity_recovery_time,
            "common_velocity_after_capability_delay": recovery_delay,
        },
        "task": {
            "start_stamp": task_start_stamp,
            "completion_stamp": task_completion_stamp,
            "completion_time": task_time,
            "evaluation_target": evaluation_target,
            "completion_scope": "all_recorded_samples",
        },
    }
