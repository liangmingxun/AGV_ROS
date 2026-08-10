import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros1_ws_src" / "multi_agv_vision_bridge" / "src"
sys.path.insert(0, str(PKG))

from multi_agv_vision_bridge.validation import (  # noqa: E402
    EntitySpec,
    GroundReferenceSpec,
    PlaneCalibrationLimits,
    TimestampPolicy,
    Workspace,
    validate_body,
)

SPECS = {
    "agv1": EntitySpec("agv1", 1, "agv_marker_plane", 0.225),
    "agv2": EntitySpec("agv2", 2, "agv_marker_plane", 0.225),
    "agv3": EntitySpec("agv3", 3, "agv_marker_plane", 0.225),
    "load": EntitySpec("load", 0, "load_marker_plane", 0.0),
}
GROUND_REFERENCE = GroundReferenceSpec(
    "ground_aruco_dynamic", (8, 11, 15, 30), 8, 11, 30, 0.150
)
CALIBRATION_LIMITS = {
    "agv_marker_plane": PlaneCalibrationLimits(
        0.225, 0.025, 0.050, 6, 0.80, 0.030, 0.060, 4, 8
    ),
    "load_marker_plane": PlaneCalibrationLimits(
        0.0, 0.025, 0.050, 6, 0.80, 0.030, 0.060, 4, 8
    ),
}
WORKSPACE = Workspace(False, -1.0, 2.0, -1.0, 2.0)
POLICY_RECEIPT = TimestampPolicy("receipt", 0.1, 0.05, 0.02)
POLICY_SYNCED = TimestampPolicy("capture_synced", 0.1, 0.05, 0.02)


def calibration():
    result = {}
    for plane, height in (("agv_marker_plane", 0.225), ("load_marker_plane", 0.0)):
        result.update(
            {
                f"{plane}_rmse_m": 0.004,
                f"{plane}_max_error_m": 0.009,
                f"{plane}_validation_rmse_m": 0.005,
                f"{plane}_validation_max_error_m": 0.011,
                f"{plane}_inlier_count": 8,
                f"{plane}_total_count": 8,
                f"{plane}_validation_count": 4,
                f"{plane}_height_m": height,
            }
        )
    return result


def packet(receipt_ns=None):
    receipt_ns = receipt_ns or time.time_ns()
    capture = receipt_ns - 2_000_000
    send = receipt_ns - 1_000_000
    detections = []
    for index, spec in enumerate(SPECS.values(), start=1):
        detections.append(
            {
                "entity_id": spec.entity_id,
                "aruco_id": spec.aruco_id,
                "x_m": 0.1 * index,
                "y_m": 0.2 * index,
                "yaw_rad": 0.01 * index,
                "quality": 0.9,
                "marker_area_px": 2000.0,
                "pixel_x": 100.0,
                "pixel_y": 200.0,
                "plane_id": spec.plane_id,
            }
        )
    return receipt_ns, {
        "session_id": "abc",
        "seq": 1,
        "capture_time_unix_ns": capture,
        "send_time_unix_ns": send,
        "source_ip": "192.168.6.100",
        "frame_id": "world",
        "camera_alive": True,
        "frame_number": 10,
        "processing_ms": 1.0,
        "ground_reference": {
            "mode": "ground_aruco_dynamic",
            "marker_ids": [8, 11, 15, 30],
            "origin_id": 8,
            "x_axis_id": 11,
            "y_side_id": 30,
            "marker_length_m": 0.150,
            "sample_count": 15,
            "calibrated_at_unix_ns": capture,
        },
        "calibration": calibration(),
        "detections": detections,
    }


def validate(body, receipt, policy=POLICY_RECEIPT):
    return validate_body(
        body,
        expected_source_ip="192.168.6.100",
        frame_id="world",
        entity_specs=SPECS,
        ground_reference_spec=GROUND_REFERENCE,
        calibration_limits=CALIBRATION_LIMITS,
        workspace=WORKSPACE,
        timestamp_policy=policy,
        receipt_time_unix_ns=receipt,
    )


def expect_reject(body, receipt, text, policy=POLICY_RECEIPT):
    try:
        validate(body, receipt, policy)
    except ValueError as exc:
        assert text in str(exc)
    else:
        raise AssertionError(f"expected rejection containing {text}")


def test_four_entities_and_empty_packet():
    receipt, body = packet()
    assert len(validate(body, receipt).detections) == 4
    body["detections"] = []
    assert validate(body, receipt).detections == []


def test_low_confidence_is_preserved():
    receipt, body = packet()
    body["detections"][0]["quality"] = 0.01
    validated = validate(body, receipt)
    assert validated.detections[0].quality == 0.01


def test_independent_load_omission():
    receipt, body = packet()
    body["detections"] = [d for d in body["detections"] if d["entity_id"] != "load"]
    validated = validate(body, receipt)
    assert {d.entity_id for d in validated.detections} == {"agv1", "agv2", "agv3"}


def test_duplicate_entity_rejected():
    receipt, body = packet()
    body["detections"][-1] = dict(body["detections"][0])
    expect_reject(body, receipt, "duplicate")


def test_wrong_aruco_and_plane_rejected():
    receipt, body = packet()
    body["detections"][0]["aruco_id"] = 99
    expect_reject(body, receipt, "ArUco")
    receipt, body = packet()
    body["detections"][0]["plane_id"] = "wrong"
    expect_reject(body, receipt, "plane")


def test_bad_session_and_time_rejected():
    receipt, body = packet()
    body["session_id"] = ""
    expect_reject(body, receipt, "session")
    receipt, body = packet()
    body["send_time_unix_ns"] = body["capture_time_unix_ns"] - 1
    expect_reject(body, receipt, "precedes")


def test_capture_synced_latency_checks():
    receipt, body = packet()
    body["send_time_unix_ns"] = receipt - 100_000_000
    body["capture_time_unix_ns"] = body["send_time_unix_ns"] - 1_000_000
    expect_reject(body, receipt, "transport delay", POLICY_SYNCED)
    receipt, body = packet()
    body["capture_time_unix_ns"] = receipt + 100_000_000
    body["send_time_unix_ns"] = receipt + 101_000_000
    expect_reject(body, receipt, "future", POLICY_SYNCED)


def test_height_mismatch_rejected():
    receipt, body = packet()
    body["calibration"]["load_marker_plane_height_m"] = 0.5
    expect_reject(body, receipt, "height mismatch")


def test_ground_reference_identity_mismatch_rejected():
    receipt, body = packet()
    body["ground_reference"]["marker_ids"] = [8, 11, 15, 31]
    expect_reject(body, receipt, "marker ID/order")

    receipt, body = packet()
    body["ground_reference"]["marker_length_m"] = 0.080
    expect_reject(body, receipt, "marker length")


def test_calibration_quality_limits_rejected():
    receipt, body = packet()
    body["calibration"]["agv_marker_plane_validation_rmse_m"] = 0.031
    expect_reject(body, receipt, "validation RMSE")

    receipt, body = packet()
    body["calibration"]["load_marker_plane_inlier_count"] = 5
    expect_reject(body, receipt, "inlier count")


def test_integer_overflow_rejected():
    receipt, body = packet()
    body["seq"] = 1 << 63
    expect_reject(body, receipt, "maximum")
