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


def _first_time(rows, predicate, not_before=-math.inf):
    for row in rows:
        stamp = finite_float(row.get("stamp"))
        if stamp >= not_before and predicate(row):
            return stamp
    return None


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
    ratios_seen = 0
    wheel_errors = []
    path_errors = []

    for row in rows:
        demanded = False
        limited = False
        for robot, side in WHEELS:
            prefix = "agv{}_wheel_{}".format(robot, side)
            raw = finite_float(row.get(prefix + "_raw"))
            applied = finite_float(row.get(prefix + "_applied"))
            actual = finite_float(row.get(prefix + "_actual"))
            limit = finite_float(row.get(prefix + "_reported_limit"))
            if math.isfinite(raw) and math.isfinite(limit) and limit > 0.0:
                ratio = abs(raw) / limit
                maximum_ratio = max(maximum_ratio, ratio)
                ratios_seen += 1
                demanded = demanded or abs(raw) > limit
            if math.isfinite(raw) and math.isfinite(applied):
                limited = limited or abs(applied - raw) > command_epsilon
            if math.isfinite(actual) and math.isfinite(applied):
                wheel_errors.append(actual - applied)
        demand_samples += int(demanded)
        limited_samples += int(limited)

        actual_progress = finite_float(row.get("load_s_actual"))
        reference_progress = finite_float(row.get("load_s_reference"))
        if math.isfinite(actual_progress) and math.isfinite(reference_progress):
            path_errors.append(actual_progress - reference_progress)

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
            lambda row: finite_float(
                row.get("agv2_mapped_velocity_upper")) < threshold)
        if derating_observed_stamp is not None:
            capability_recovery_stamp = _first_time(
                rows,
                lambda row: finite_float(
                    row.get("agv2_mapped_velocity_upper")) >= threshold,
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
    task_completion_stamp = None
    if evaluation_target is not None:
        target = float(evaluation_target)
        task_completion_stamp = _first_time(
            rows,
            lambda row: finite_float(row.get("load_s_actual")) >= target)
        if task_completion_stamp is not None:
            task_time = task_completion_stamp - first_stamp

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
    return {
        "schema_version": 1,
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
            "formal_statistics_ready":
                evaluation_source == "experiment_state",
        },
        "wheel": {
            "demand_exceedance_samples": demand_samples,
            "demand_exceedance_time": demand_samples * sample_period,
            "limited_samples": limited_samples,
            "limited_time": limited_samples * sample_period,
            "maximum_demand_ratio": (
                maximum_ratio if ratios_seen else None),
            "tracking_rmse": _rmse(wheel_errors),
            "tracking_max_absolute": _maximum_absolute(wheel_errors),
        },
        "path": {
            "progress_rmse": _rmse(path_errors),
            "progress_max_absolute": _maximum_absolute(path_errors),
        },
        "recovery": {
            "capability_source": "agv2_mapped_velocity_upper",
            "derating_observed_stamp": derating_observed_stamp,
            "capability_95_stamp": capability_recovery_stamp,
            "capability_95_time": capability_recovery_time,
            "common_velocity_95_stamp": velocity_recovery_stamp,
            "common_velocity_95_time": common_velocity_recovery_time,
            "common_velocity_after_capability_delay": recovery_delay,
        },
        "task": {
            "completion_stamp": task_completion_stamp,
            "completion_time": task_time,
            "evaluation_target": evaluation_target,
        },
    }
