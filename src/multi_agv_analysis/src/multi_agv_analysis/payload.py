"""Central payload data-source and paper-label resolution."""

import math
from pathlib import Path

from .io_utils import bool_value, finite_float, load_yaml, sha256_file


STAGE_D_FREEZE_ID = "wheel_speed_scale_stage_d_frozen_v1"
STAGE_D_FREEZE_CONFIG = "wheel_speed_scale_stage_d_freeze_v1.yaml"


def _mode(metadata):
    payload = metadata.get("payload", {})
    if isinstance(payload, dict) and payload.get("mode"):
        return str(payload["mode"]).strip().lower(), True
    for key in ("payload_mode", "payload_state"):
        if metadata.get(key):
            return str(metadata[key]).strip().lower(), True
    return "unloaded", False


def _has_pose(rows, prefix, require_yaw=True):
    axes = ("x", "y", "yaw") if require_yaw else ("x", "y")
    fields = tuple(prefix + suffix for suffix in axes)
    return any(all(math.isfinite(finite_float(row.get(field)))
                   for field in fields) for row in rows)


def verify_stage_d_snapshot(metadata, run_dir):
    """Verify archived evidence, never the current checkout or file names alone."""
    if run_dir is None:
        raise RuntimeError("publication mode requires the recorded run directory")
    root = Path(run_dir).resolve()
    records = metadata.get("config_hashes", [])

    def archived(name):
        matches = [item for item in records if isinstance(item, dict) and
                   Path(str(item.get("archived_path", ""))).name.endswith(name)]
        if len(matches) != 1:
            raise RuntimeError("publication mode requires one archived {}".format(name))
        item = matches[0]
        path = (root / item["archived_path"]).resolve()
        if root not in path.parents or not path.is_file():
            raise RuntimeError("missing or unsafe archived config: {}".format(name))
        if not item.get("sha256") or sha256_file(path) != item["sha256"]:
            raise RuntimeError("archived config hash mismatch: {}".format(name))
        return load_yaml(path)

    freeze = archived(STAGE_D_FREEZE_CONFIG)
    if (freeze.get("freeze_id") != STAGE_D_FREEZE_ID or
            freeze.get("status") != "frozen" or
            freeze.get("acceptance", {}).get("all_three_robots_passed") is not True):
        raise RuntimeError("Stage-D freeze is not an accepted frozen configuration")
    runtime = archived("rosparams.yaml")
    payload = metadata.get("payload", {})
    declared_transform = payload.get("world_to_reference") if isinstance(payload, dict) else None
    if declared_transform and declared_transform != runtime.get("experiment_recorder", {}).get(
            "measured_payload_world_to_reference"):
        raise RuntimeError("measured payload transform differs from archived runtime")
    for robot in ("agv1", "agv2", "agv3"):
        expected = freeze.get("robots", {}).get(robot, {})
        config = archived(robot + "_chassis.yaml")
        active = runtime.get(robot, {}).get("chassis_controller", {})
        if expected.get("revalidation_status") != "passed":
            raise RuntimeError("Stage-D revalidation missing for {}".format(robot))
        for key in ("wheel_command_scale_left", "wheel_command_scale_right",
                    "wheel_feedback_scale_left", "wheel_feedback_scale_right"):
            values = [finite_float(source.get(key)) for source in (expected, config, active)]
            if not all(math.isfinite(value) and 0.5 <= value <= 1.5 for value in values) or any(
                    not math.isclose(value, values[0], rel_tol=0.0, abs_tol=1e-9)
                    for value in values[1:]):
                raise RuntimeError("Stage-D scale mismatch: {} {}".format(robot, key))
    return True


def resolve_payload_context(metadata, rows, publication_mode=False, run_dir=None):
    """Resolve one traceable source and its load-only display vocabulary.

    Historical runs without payload metadata remain readable in diagnostic
    mode as unloaded runs.  Publication mode requires explicit payload and
    Stage-D freeze metadata; it never disguises an equivalent-load fallback
    as a measured physical payload.
    """
    metadata = metadata or {}
    rows = list(rows)
    mode, explicit_mode = _mode(metadata)
    if mode not in ("unloaded", "loaded"):
        raise RuntimeError("unsupported payload mode: {}".format(mode))
    if publication_mode and not explicit_mode:
        raise RuntimeError(
            "publication mode requires explicit payload_mode metadata")

    freeze_id = str(metadata.get("wheel_speed_scale_freeze_id", "")).strip()
    freeze_snapshot_present = any(
        str(item.get("archived_path", item.get("path", ""))).endswith(
            STAGE_D_FREEZE_CONFIG)
        for item in metadata.get("config_hashes", [])
        if isinstance(item, dict))
    if publication_mode and freeze_id != STAGE_D_FREEZE_ID:
        raise RuntimeError(
            "publication mode requires wheel_speed_scale_freeze_id={}".format(
                STAGE_D_FREEZE_ID))
    if publication_mode and not freeze_snapshot_present:
        raise RuntimeError(
            "publication mode requires archived Stage-D freeze config")
    freeze_verified = verify_stage_d_snapshot(metadata, run_dir) if publication_mode else False

    equivalent_available = _has_pose(
        rows, "equivalent_load_pose_", require_yaw=False)
    measured_available = _has_pose(
        [row for row in rows if bool_value(row.get("measured_load_pose_valid", False))],
        "measured_load_pose_")
    legacy_available = _has_pose(rows, "load_pose_", require_yaw=False)
    diagnostic_fallback = False
    if mode == "loaded" and measured_available:
        active = [row for row in rows if bool_value(row.get("evaluation_active", True))
                  and bool_value(row.get("localization_valid", True))]
        if publication_mode and any(not bool_value(row.get("measured_load_pose_valid", False)) or
                not _has_pose([row], "measured_load_pose_") for row in active):
            raise RuntimeError("loaded publication has missing or unaligned measured payload samples")
        source = "measured_payload"
        prefix = "measured_load_pose_"
        label = "实际载荷"
    elif mode == "loaded" and publication_mode:
        raise RuntimeError(
            "loaded publication mode requires independent measured payload pose")
    else:
        if not equivalent_available and not legacy_available:
            raise RuntimeError("equivalent load pose is unavailable")
        source = "equivalent"
        prefix = ("equivalent_load_pose_" if equivalent_available
                  else "load_pose_")
        label = "虚拟等效载荷"
        diagnostic_fallback = mode == "loaded"

    return {
        "payload_mode": mode,
        "payload_mode_explicit": explicit_mode,
        "payload_actual_source": source,
        "payload_pose_prefix": prefix,
        "payload_label": label,
        "payload_trajectory_label": label + "轨迹",
        "payload_reference_label": label + "参考轨迹",
        "payload_error_label": label + "位置误差",
        "load_metric_source": source,
        "measured_payload_available": measured_available,
        "equivalent_load_available": equivalent_available,
        "diagnostic_equivalent_fallback": diagnostic_fallback,
        "wheel_speed_scale_freeze_id": freeze_id,
        "wheel_speed_scale_freeze_snapshot_present":
            freeze_snapshot_present,
        "wheel_speed_scale_freeze_verified": freeze_verified,
    }


def payload_pose(row, context):
    prefix = context["payload_pose_prefix"]
    if prefix == "measured_load_pose_" and not bool_value(row.get("measured_load_pose_valid", False)):
        return (math.nan, math.nan, math.nan)
    return tuple(finite_float(row.get(prefix + axis))
                 for axis in ("x", "y", "yaw"))


def materialize_paper_load_fields(rows, context):
    """Return copied rows with a single resolver-owned paper load view."""
    result = []
    for source in rows:
        row = dict(source)
        x, y, yaw = payload_pose(row, context)
        row["paper_load_actual_x"] = x
        row["paper_load_actual_y"] = y
        row["paper_load_actual_yaw"] = yaw
        result.append(row)
    return result
