# -*- coding: utf-8 -*-
"""Pure protocol-body validation for the Task 15 UDP bridge.

This module intentionally has no ROS imports so protocol and safety checks can
be unit-tested on any Python 3 host.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Tuple

EXPECTED_ENTITY_IDS = frozenset(("agv1", "agv2", "agv3", "load"))


@dataclass(frozen=True)
class EntitySpec:
    entity_id: str
    aruco_id: int
    plane_id: str
    marker_height_m: float


@dataclass(frozen=True)
class PlaneCalibrationLimits:
    height_m: float
    maximum_rmse_m: float
    maximum_point_error_m: float
    minimum_inliers: int
    minimum_inlier_ratio: float
    maximum_validation_rmse_m: float
    maximum_validation_point_error_m: float
    minimum_validation_points: int
    minimum_fit_points: int


@dataclass(frozen=True)
class GroundReferenceSpec:
    mode: str
    marker_ids: Tuple[int, int, int, int]
    origin_id: int
    x_axis_id: int
    y_side_id: int
    marker_length_m: float


@dataclass(frozen=True)
class TimestampPolicy:
    mode: str
    maximum_source_processing_sec: float
    maximum_transport_delay_sec: float
    maximum_future_offset_sec: float


@dataclass(frozen=True)
class Workspace:
    enabled: bool
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float


@dataclass(frozen=True)
class ValidatedDetection:
    entity_id: str
    aruco_id: int
    plane_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    quality: float
    marker_area_px: float
    pixel_x: float
    pixel_y: float


@dataclass(frozen=True)
class ValidatedPacket:
    session_id: str
    seq: int
    capture_time_unix_ns: int
    send_time_unix_ns: int
    frame_number: int
    processing_ms: float
    ground_reference: Dict[str, object]
    calibration: Dict[str, float]
    detections: List[ValidatedDetection]
    source_processing_sec: float
    transport_delay_sec: float
    future_offset_sec: float


def _strict_int(
    value,
    name: str,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if minimum is not None and value < minimum:
        raise ValueError(f"{name} is below its minimum")
    if maximum is not None and value > maximum:
        raise ValueError(f"{name} exceeds its maximum")
    return value


def _finite(value, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} is non-finite")
    return result


def _validate_calibration(
    calibration_obj,
    entity_specs: Mapping[str, EntitySpec],
    calibration_limits: Mapping[str, PlaneCalibrationLimits],
) -> Dict[str, float]:
    if not isinstance(calibration_obj, Mapping):
        raise ValueError("calibration must be a mapping")
    result: Dict[str, float] = {}
    plane_ids = sorted({spec.plane_id for spec in entity_specs.values()})
    if set(calibration_limits) != set(plane_ids):
        raise ValueError("calibration limit configuration does not match entity planes")
    for plane_id in plane_ids:
        required = (
            f"{plane_id}_rmse_m",
            f"{plane_id}_max_error_m",
            f"{plane_id}_validation_rmse_m",
            f"{plane_id}_validation_max_error_m",
            f"{plane_id}_inlier_count",
            f"{plane_id}_total_count",
            f"{plane_id}_validation_count",
            f"{plane_id}_height_m",
        )
        for key in required:
            if key not in calibration_obj:
                raise ValueError(f"calibration is missing {key}")
            value = _finite(calibration_obj[key], f"calibration.{key}")
            if key.endswith("_count"):
                if value < 0 or not value.is_integer():
                    raise ValueError(f"calibration.{key} must be a non-negative integer")
            elif key.endswith("_height_m"):
                pass
            elif value < 0.0:
                raise ValueError(f"calibration.{key} must be non-negative")
            result[key] = value

        configured_heights = {
            spec.marker_height_m
            for spec in entity_specs.values()
            if spec.plane_id == plane_id
        }
        if len(configured_heights) != 1:
            raise ValueError(f"configured entities disagree on height for plane {plane_id}")
        configured_height = next(iter(configured_heights))
        packet_height = result[f"{plane_id}_height_m"]
        if abs(packet_height - configured_height) > 1e-6:
            raise ValueError(f"calibration height mismatch for plane {plane_id}")
        limits = calibration_limits[plane_id]
        if abs(limits.height_m - configured_height) > 1e-12:
            raise ValueError(f"local calibration height mismatch for plane {plane_id}")

        rmse = result[f"{plane_id}_rmse_m"]
        maximum_error = result[f"{plane_id}_max_error_m"]
        validation_rmse = result[f"{plane_id}_validation_rmse_m"]
        validation_maximum_error = result[f"{plane_id}_validation_max_error_m"]
        inlier_count = int(result[f"{plane_id}_inlier_count"])
        total_count = int(result[f"{plane_id}_total_count"])
        validation_count = int(result[f"{plane_id}_validation_count"])
        if total_count < limits.minimum_fit_points or inlier_count > total_count:
            raise ValueError(f"invalid calibration fit counts for plane {plane_id}")
        if inlier_count < limits.minimum_inliers:
            raise ValueError(f"calibration inlier count below limit for plane {plane_id}")
        if inlier_count / total_count < limits.minimum_inlier_ratio:
            raise ValueError(f"calibration inlier ratio below limit for plane {plane_id}")
        if validation_count < limits.minimum_validation_points:
            raise ValueError(f"calibration validation count below limit for plane {plane_id}")
        if rmse > limits.maximum_rmse_m:
            raise ValueError(f"calibration RMSE exceeds limit for plane {plane_id}")
        if maximum_error > limits.maximum_point_error_m:
            raise ValueError(f"calibration max error exceeds limit for plane {plane_id}")
        if validation_rmse > limits.maximum_validation_rmse_m:
            raise ValueError(f"calibration validation RMSE exceeds limit for plane {plane_id}")
        if validation_maximum_error > limits.maximum_validation_point_error_m:
            raise ValueError(
                f"calibration validation max error exceeds limit for plane {plane_id}"
            )
    return result


def _validate_ground_reference(
    ground_obj,
    spec: GroundReferenceSpec,
) -> Dict[str, object]:
    if not isinstance(ground_obj, Mapping):
        raise ValueError("ground_reference must be a mapping")
    if ground_obj.get("mode") != spec.mode:
        raise ValueError("ground_reference mode mismatch")
    marker_ids_obj = ground_obj.get("marker_ids")
    if not isinstance(marker_ids_obj, list):
        raise ValueError("ground_reference.marker_ids must be an array")
    marker_ids = tuple(
        _strict_int(value, "ground_reference.marker_ids", 0)
        for value in marker_ids_obj
    )
    if marker_ids != spec.marker_ids:
        raise ValueError("ground_reference marker ID/order mismatch")
    for field, expected in (
        ("origin_id", spec.origin_id),
        ("x_axis_id", spec.x_axis_id),
        ("y_side_id", spec.y_side_id),
    ):
        if _strict_int(ground_obj.get(field), f"ground_reference.{field}", 0) != expected:
            raise ValueError(f"ground_reference {field} mismatch")
    marker_length_m = _finite(
        ground_obj.get("marker_length_m"),
        "ground_reference.marker_length_m",
    )
    if abs(marker_length_m - spec.marker_length_m) > 1.0e-9:
        raise ValueError("ground_reference marker length mismatch")
    sample_count = _strict_int(
        ground_obj.get("sample_count"),
        "ground_reference.sample_count",
        1,
    )
    calibrated_at_unix_ns = _strict_int(
        ground_obj.get("calibrated_at_unix_ns"),
        "ground_reference.calibrated_at_unix_ns",
        1,
        (1 << 63) - 1,
    )
    return {
        "mode": spec.mode,
        "marker_ids": list(marker_ids),
        "origin_id": spec.origin_id,
        "x_axis_id": spec.x_axis_id,
        "y_side_id": spec.y_side_id,
        "marker_length_m": marker_length_m,
        "sample_count": sample_count,
        "calibrated_at_unix_ns": calibrated_at_unix_ns,
    }


def validate_body(
    body: Mapping,
    *,
    expected_source_ip: str,
    frame_id: str,
    entity_specs: Mapping[str, EntitySpec],
    ground_reference_spec: GroundReferenceSpec,
    calibration_limits: Mapping[str, PlaneCalibrationLimits],
    workspace: Workspace,
    timestamp_policy: TimestampPolicy,
    receipt_time_unix_ns: int,
) -> ValidatedPacket:
    if not isinstance(body, Mapping):
        raise ValueError("packet body must be a mapping")

    source_ip = body.get("source_ip")
    if not isinstance(source_ip, str) or source_ip != expected_source_ip:
        raise ValueError("declared source_ip mismatch")
    packet_frame = body.get("frame_id")
    if not isinstance(packet_frame, str) or packet_frame != frame_id:
        raise ValueError("frame_id mismatch")
    if body.get("camera_alive") is not True:
        raise ValueError("camera_alive is not true")

    session = body.get("session_id")
    if not isinstance(session, str) or not session or len(session) > 128:
        raise ValueError("invalid session_id")
    maximum_int64 = (1 << 63) - 1
    seq = _strict_int(body.get("seq"), "seq", 0, maximum_int64)
    capture_ns = _strict_int(
        body.get("capture_time_unix_ns"),
        "capture_time_unix_ns",
        1,
        maximum_int64,
    )
    send_ns = _strict_int(
        body.get("send_time_unix_ns"),
        "send_time_unix_ns",
        1,
        maximum_int64,
    )
    if send_ns < capture_ns:
        raise ValueError("send_time_unix_ns precedes capture_time_unix_ns")
    frame_number = _strict_int(
        body.get("frame_number"), "frame_number", 0, maximum_int64
    )
    processing_ms = _finite(body.get("processing_ms"), "processing_ms")
    if processing_ms < 0.0:
        raise ValueError("processing_ms must be non-negative")

    source_processing_sec = (send_ns - capture_ns) * 1e-9
    if source_processing_sec > timestamp_policy.maximum_source_processing_sec:
        raise ValueError("source processing time exceeds configured limit")
    if processing_ms * 1e-3 > timestamp_policy.maximum_source_processing_sec:
        raise ValueError("declared processing_ms exceeds configured limit")

    transport_delay_sec = (receipt_time_unix_ns - send_ns) * 1e-9
    future_offset_sec = (capture_ns - receipt_time_unix_ns) * 1e-9
    if timestamp_policy.mode == "capture_synced":
        if transport_delay_sec > timestamp_policy.maximum_transport_delay_sec:
            raise ValueError("transport delay exceeds configured limit")
        if future_offset_sec > timestamp_policy.maximum_future_offset_sec:
            raise ValueError("capture timestamp is too far in the future")
        send_future_sec = (send_ns - receipt_time_unix_ns) * 1e-9
        if send_future_sec > timestamp_policy.maximum_future_offset_sec:
            raise ValueError("send timestamp is too far in the future")
    elif timestamp_policy.mode != "receipt":
        raise ValueError("timestamp mode must be receipt or capture_synced")

    if set(entity_specs) != EXPECTED_ENTITY_IDS:
        raise ValueError("entity configuration must contain agv1/agv2/agv3/load")
    ground_reference = _validate_ground_reference(
        body.get("ground_reference"), ground_reference_spec
    )
    calibration = _validate_calibration(
        body.get("calibration"), entity_specs, calibration_limits
    )

    detections_obj = body.get("detections")
    if not isinstance(detections_obj, list) or len(detections_obj) > 4:
        raise ValueError("detections must be an array of length 0..4")
    detections: List[ValidatedDetection] = []
    seen_entities = set()
    seen_aruco = set()
    for index, item in enumerate(detections_obj):
        if not isinstance(item, Mapping):
            raise ValueError(f"detections[{index}] must be a mapping")
        entity_id = item.get("entity_id")
        if not isinstance(entity_id, str) or entity_id not in entity_specs:
            raise ValueError(f"unexpected entity_id: {entity_id}")
        aruco_id = _strict_int(item.get("aruco_id"), f"{entity_id}.aruco_id", 0)
        plane_id = item.get("plane_id")
        if not isinstance(plane_id, str) or not plane_id:
            raise ValueError(f"{entity_id}.plane_id is invalid")
        spec = entity_specs[entity_id]
        if aruco_id != spec.aruco_id:
            raise ValueError(f"entity/ArUco mapping mismatch for {entity_id}")
        if plane_id != spec.plane_id:
            raise ValueError(f"entity/plane mapping mismatch for {entity_id}")
        if entity_id in seen_entities or aruco_id in seen_aruco:
            raise ValueError("duplicate entity or ArUco detection in one packet")
        seen_entities.add(entity_id)
        seen_aruco.add(aruco_id)

        x = _finite(item.get("x_m"), f"{entity_id}.x_m")
        y = _finite(item.get("y_m"), f"{entity_id}.y_m")
        yaw = _finite(item.get("yaw_rad"), f"{entity_id}.yaw_rad")
        if yaw < -math.pi - 1e-6 or yaw > math.pi + 1e-6:
            raise ValueError(f"{entity_id}.yaw_rad is outside [-pi, pi]")
        quality = _finite(item.get("quality"), f"{entity_id}.quality")
        if quality < 0.0 or quality > 1.0:
            raise ValueError(f"{entity_id}.quality is outside [0,1]")
        area = _finite(item.get("marker_area_px", 0.0), f"{entity_id}.marker_area_px")
        pixel_x = _finite(item.get("pixel_x", 0.0), f"{entity_id}.pixel_x")
        pixel_y = _finite(item.get("pixel_y", 0.0), f"{entity_id}.pixel_y")
        if area < 0.0:
            raise ValueError(f"{entity_id}.marker_area_px must be non-negative")
        if workspace.enabled and not (
            workspace.x_min_m <= x <= workspace.x_max_m
            and workspace.y_min_m <= y <= workspace.y_max_m
        ):
            raise ValueError(f"{entity_id} pose is outside configured workspace")
        detections.append(
            ValidatedDetection(
                entity_id=entity_id,
                aruco_id=aruco_id,
                plane_id=plane_id,
                x_m=x,
                y_m=y,
                yaw_rad=yaw,
                quality=quality,
                marker_area_px=area,
                pixel_x=pixel_x,
                pixel_y=pixel_y,
            )
        )

    return ValidatedPacket(
        session_id=session,
        seq=seq,
        capture_time_unix_ns=capture_ns,
        send_time_unix_ns=send_ns,
        frame_number=frame_number,
        processing_ms=processing_ms,
        ground_reference=ground_reference,
        calibration=calibration,
        detections=detections,
        source_processing_sec=source_processing_sec,
        transport_delay_sec=transport_delay_sec,
        future_offset_sec=future_offset_sec,
    )
