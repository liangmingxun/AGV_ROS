"""Central payload data-source and paper-label resolution."""

import math

from .io_utils import finite_float


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


def resolve_payload_context(metadata, rows, publication_mode=False):
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

    equivalent_available = _has_pose(
        rows, "equivalent_load_pose_", require_yaw=False)
    measured_available = _has_pose(rows, "measured_load_pose_")
    legacy_available = _has_pose(rows, "load_pose_", require_yaw=False)
    diagnostic_fallback = False
    if mode == "loaded" and measured_available:
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
    }


def payload_pose(row, context):
    prefix = context["payload_pose_prefix"]
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
