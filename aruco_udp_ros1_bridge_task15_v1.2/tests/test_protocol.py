import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sender = load_module("sender_protocol", ROOT / "windows_sender" / "protocol.py")
receiver = load_module(
    "receiver_protocol",
    ROOT / "ros1_ws_src" / "multi_agv_vision_bridge" / "src" / "multi_agv_vision_bridge" / "protocol.py",
)


def body():
    return {
        "protocol": sender.PROTOCOL_NAME,
        "version": sender.PROTOCOL_VERSION,
        "session_id": "session-test",
        "seq": 4,
        "capture_time_unix_ns": 1_000_000_000,
        "send_time_unix_ns": 1_001_000_000,
        "source_ip": "192.168.6.100",
        "frame_id": "world",
        "camera_alive": True,
        "frame_number": 7,
        "processing_ms": 1.0,
        "ground_reference": {
            "mode": "ground_aruco_dynamic",
            "marker_ids": [8, 11, 15, 30],
            "origin_id": 8,
            "x_axis_id": 11,
            "y_side_id": 30,
            "marker_length_m": 0.150,
            "sample_count": 15,
            "calibrated_at_unix_ns": 999_000_000,
        },
        "calibration": {
            "agv_marker_plane_rmse_m": 0.004,
            "agv_marker_plane_max_error_m": 0.009,
            "agv_marker_plane_validation_rmse_m": 0.005,
            "agv_marker_plane_validation_max_error_m": 0.011,
            "agv_marker_plane_inlier_count": 8,
            "agv_marker_plane_total_count": 8,
            "agv_marker_plane_validation_count": 4,
            "agv_marker_plane_height_m": 0.225,
            "load_marker_plane_rmse_m": 0.005,
            "load_marker_plane_max_error_m": 0.011,
            "load_marker_plane_validation_rmse_m": 0.006,
            "load_marker_plane_validation_max_error_m": 0.012,
            "load_marker_plane_inlier_count": 8,
            "load_marker_plane_total_count": 8,
            "load_marker_plane_validation_count": 4,
            "load_marker_plane_height_m": 0.0,
        },
        "detections": [
            {
                "entity_id": entity,
                "aruco_id": aruco,
                "x_m": 0.1 * index,
                "y_m": 0.2 * index,
                "yaw_rad": 0.01 * index,
                "quality": 0.9,
                "marker_area_px": 2000.0,
                "pixel_x": 100.0,
                "pixel_y": 200.0,
                "plane_id": "load_marker_plane" if entity == "load" else "agv_marker_plane",
            }
            for index, (entity, aruco) in enumerate(
                (("agv1", 1), ("agv2", 2), ("agv3", 3), ("load", 0)), start=1
            )
        ],
    }


def test_cross_codec_round_trip():
    value = body()
    assert receiver.decode_packet(sender.encode_packet(value)) == value
    assert sender.decode_packet(receiver.encode_packet(value)) == value
    assert sender.canonical_json_bytes(value) == receiver.canonical_json_bytes(value)


def test_crc_damage_rejected():
    packet = sender.encode_packet(body())
    envelope = json.loads(packet.decode("utf-8"))
    envelope["crc32"] = "00000000"
    damaged = sender.canonical_json_bytes(envelope)
    try:
        receiver.decode_packet(damaged)
    except ValueError as exc:
        assert "CRC32" in str(exc)
    else:
        raise AssertionError("bad CRC must be rejected")


def test_v2_rejected():
    value = body()
    value["version"] = 2
    packet = sender.encode_packet(value)
    try:
        receiver.decode_packet(packet)
    except ValueError as exc:
        assert "version" in str(exc)
    else:
        raise AssertionError("v2 must be rejected")


def test_packet_limit():
    value = body()
    value["padding"] = "x" * sender.MAX_PACKET_BYTES
    try:
        sender.encode_packet(value)
    except ValueError as exc:
        assert "too large" in str(exc)
    else:
        raise AssertionError("oversized packet must be rejected")
