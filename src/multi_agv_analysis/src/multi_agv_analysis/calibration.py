"""Parameter-free SE(2) calibration solvers and fail-closed evidence checks."""

import math
from collections import defaultdict

import numpy as np

from .io_utils import finite_float


ENTITIES = ("agv1", "agv2", "agv3", "load")


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (
        a[0] + ca * b[0] - sa * b[1],
        a[1] + sa * b[0] + ca * b[1],
        wrap_angle(a[2] + b[2]),
    )


def inverse(value):
    c, s = math.cos(value[2]), math.sin(value[2])
    return (
        -c * value[0] - s * value[1],
        s * value[0] - c * value[1],
        wrap_angle(-value[2]),
    )


def _required_limit(policy, name):
    value = finite_float(policy.get(name))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(
            "calibration policy must provide positive {}".format(name))
    return value


def _required_count(policy, name, minimum):
    value = policy.get(name)
    if value is None or isinstance(value, bool):
        raise ValueError(
            "calibration policy must provide integer {}".format(name))
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise ValueError(
            "calibration policy must provide integer {}".format(name))
    if parsed != value or parsed < minimum:
        raise ValueError(
            "{} must be an integer of at least {}".format(name, minimum))
    return parsed


def _pose(row, prefix):
    values = tuple(finite_float(row.get(prefix + suffix))
                   for suffix in ("_x", "_y", "_yaw"))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite {} pose".format(prefix))
    return values


def _circular_mean(values):
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) <= 1.0e-12:
        raise ValueError("yaw samples have no unique circular mean")
    return math.atan2(sine, cosine)


def solve_tag_to_target(rows, policy):
    """Estimate fixed tag->target transforms from paired world poses."""
    required_entities = policy.get("required_entities", list(ENTITIES))
    if (not isinstance(required_entities, list) or
            not required_entities or
            len(set(required_entities)) != len(required_entities) or
            any(entity not in ENTITIES for entity in required_entities)):
        raise ValueError(
            "required_entities must be a non-empty unique subset of {}".format(
                ENTITIES))
    minimum_samples = _required_count(
        policy, "minimum_samples_per_entity", 2)
    maximum_translation = _required_limit(
        policy, "maximum_translation_residual")
    maximum_yaw = _required_limit(policy, "maximum_yaw_residual")
    grouped = defaultdict(list)
    for row in rows:
        entity = str(row.get("entity", "")).strip()
        if entity not in ENTITIES:
            raise ValueError("unknown calibration entity: {}".format(entity))
        tag = _pose(row, "tag")
        target = _pose(row, "target")
        grouped[entity].append(compose(inverse(tag), target))

    missing = [entity for entity in required_entities
               if len(grouped[entity]) < minimum_samples]
    if missing:
        raise ValueError(
            "insufficient tag-to-target samples: {}".format(missing))
    estimates = {}
    passed = True
    for entity in required_entities:
        samples = grouped[entity]
        estimate = (
            sum(value[0] for value in samples) / len(samples),
            sum(value[1] for value in samples) / len(samples),
            _circular_mean([value[2] for value in samples]),
        )
        translation = [
            math.hypot(value[0] - estimate[0], value[1] - estimate[1])
            for value in samples]
        yaw = [abs(wrap_angle(value[2] - estimate[2]))
               for value in samples]
        entity_passed = (
            max(translation) <= maximum_translation and
            max(yaw) <= maximum_yaw)
        passed = passed and entity_passed
        estimates[entity] = {
            "tag_to_target_x": estimate[0],
            "tag_to_target_y": estimate[1],
            "tag_to_target_yaw": estimate[2],
            "samples": len(samples),
            "translation_residual_rmse": math.sqrt(
                sum(value * value for value in translation) / len(samples)),
            "translation_residual_max": max(translation),
            "yaw_residual_rmse": math.sqrt(
                sum(value * value for value in yaw) / len(samples)),
            "yaw_residual_max": max(yaw),
            "passed": entity_passed,
        }
    return {
        "passed": passed,
        "required_entities": list(required_entities),
        "unresolved_entities": [
            entity for entity in ENTITIES if entity not in required_entities],
        "entities": estimates,
    }


def solve_world_alignment(rows, policy):
    """Solve one rigid 2-D source->world transform from point pairs."""
    minimum_samples = _required_count(
        policy, "minimum_world_samples", 3)
    maximum_residual = _required_limit(
        policy, "maximum_world_point_residual")
    source = []
    target = []
    for row in rows:
        sx = finite_float(row.get("source_x"))
        sy = finite_float(row.get("source_y"))
        wx = finite_float(row.get("world_x"))
        wy = finite_float(row.get("world_y"))
        if not all(math.isfinite(value) for value in (sx, sy, wx, wy)):
            raise ValueError("world alignment contains a non-finite point")
        source.append((sx, sy))
        target.append((wx, wy))
    if len(source) < minimum_samples:
        raise ValueError("insufficient world-alignment samples")
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    predicted = (rotation @ source.T).T + translation
    residual = np.linalg.norm(predicted - target, axis=1)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return {
        "passed": bool(float(residual.max()) <= maximum_residual),
        "source_to_world": {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "yaw": yaw,
        },
        "samples": len(source),
        "point_residual_rmse": float(
            math.sqrt(float(np.mean(residual * residual)))),
        "point_residual_max": float(residual.max()),
    }


def calibration_candidate(tag_result, world_result, source_hashes):
    passed = bool(tag_result.get("passed") and world_result.get("passed"))
    return {
        "schema_version": 1,
        "status": (
            "candidate_passed_numeric_policy"
            if passed else "candidate_rejected_numeric_policy"),
        "calibration_authorized": False,
        "extrinsics_frozen": False,
        "requires_manual_review": True,
        "source_hashes": source_hashes,
        "world_alignment": world_result,
        "entities": tag_result.get("entities", {}),
        "unresolved_entities": tag_result.get(
            "unresolved_entities", []),
        "numeric_policy_passed": passed,
        "warning": (
            "Candidate only. Copy reviewed values into a separately approved "
            "localization config; this file cannot authorize hardware."),
    }
