# -*- coding: utf-8 -*-
"""Send synthetic UDP v3 packets for Task 15 network/interface testing."""
from __future__ import annotations

import argparse
import json
import socket
import time
import uuid
from pathlib import Path
from typing import Dict, List

import yaml

from protocol import PROTOCOL_NAME, PROTOCOL_VERSION, canonical_json_bytes, encode_packet

SCENARIOS = (
    "normal",
    "omit_entity",
    "empty",
    "low_quality",
    "bad_crc",
    "duplicate_entity",
    "duplicate_aruco",
    "wrong_aruco",
    "wrong_plane",
    "out_of_order",
    "stale_time",
    "future_time",
    "bad_time_order",
    "bad_source_declared",
    "empty_session",
    "old_version",
)


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("configuration root must be a mapping")
    return data


def entity_items(config: dict) -> List[dict]:
    items = config.get("entities", [])
    if not isinstance(items, list) or len(items) != 4:
        raise ValueError("config.entities must contain four entities")
    expected_entities = {"agv1", "agv2", "agv3", "load"}
    actual_entities = {item.get("entity_id") for item in items if isinstance(item, dict)}
    aruco_ids = [item.get("aruco_id") for item in items if isinstance(item, dict)]
    if actual_entities != expected_entities or len(aruco_ids) != 4:
        raise ValueError("config.entities must be exactly agv1/agv2/agv3/load")
    if (
        any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in aruco_ids)
        or len(set(aruco_ids)) != 4
    ):
        raise ValueError("config.entities must use four unique non-negative ArUco IDs")
    for item in items:
        expected_plane = (
            "load_marker_plane"
            if item["entity_id"] == "load"
            else "agv_marker_plane"
        )
        if item.get("plane_id") != expected_plane:
            raise ValueError(f"invalid plane_id for {item['entity_id']}")
    return items


def calibration_payload(config: dict) -> Dict[str, float]:
    result: Dict[str, float] = {}
    plane_ids = sorted({str(item["plane_id"]) for item in entity_items(config)})
    planes = config.get("planes", {})
    for plane_id in plane_ids:
        plane = planes.get(plane_id, {}) if isinstance(planes, dict) else {}
        result[f"{plane_id}_rmse_m"] = 0.005
        result[f"{plane_id}_max_error_m"] = 0.010
        result[f"{plane_id}_validation_rmse_m"] = 0.006
        result[f"{plane_id}_validation_max_error_m"] = 0.012
        result[f"{plane_id}_inlier_count"] = 60
        result[f"{plane_id}_total_count"] = 60
        result[f"{plane_id}_validation_count"] = 60
        result[f"{plane_id}_height_m"] = float(plane.get("height_m", 0.0))
    return result


def ground_reference_payload(config: dict, calibrated_at_unix_ns: int) -> dict:
    ground = config.get("external_calibration", {})
    return {
        "mode": str(ground["mode"]),
        "marker_ids": list(ground["marker_ids"]),
        "origin_id": int(ground["origin_id"]),
        "x_axis_id": int(ground["x_axis_id"]),
        "y_side_id": int(ground["y_side_id"]),
        "marker_length_m": float(ground["marker_length_m"]),
        "sample_count": 60,
        "calibrated_at_unix_ns": calibrated_at_unix_ns,
    }


def detections_for(config: dict, seq: int) -> List[dict]:
    result = []
    for index, entity in enumerate(entity_items(config), start=1):
        result.append(
            {
                "entity_id": str(entity["entity_id"]),
                "aruco_id": int(entity["aruco_id"]),
                "x_m": round(0.2 * index + 0.001 * seq, 6),
                "y_m": round(-0.1 + 0.1 * index, 6),
                "yaw_rad": round(0.05 * index, 7),
                "quality": 0.95,
                "marker_area_px": 3000.0 - 100.0 * index,
                "pixel_x": 120.0 * index,
                "pixel_y": 180.0 + 20.0 * index,
                "plane_id": str(entity["plane_id"]),
            }
        )
    return result


def apply_scenario(body: dict, scenario: str, omit_entity: str, iteration: int) -> None:
    detections = body["detections"]
    if scenario == "omit_entity":
        body["detections"] = [d for d in detections if d["entity_id"] != omit_entity]
    elif scenario == "empty":
        body["detections"] = []
    elif scenario == "low_quality":
        detections[0]["quality"] = 0.01
    elif scenario == "duplicate_entity":
        body["detections"][-1] = dict(detections[0])
    elif scenario == "duplicate_aruco":
        duplicate = dict(detections[1])
        duplicate["aruco_id"] = detections[0]["aruco_id"]
        body["detections"] = [detections[0], duplicate] + detections[2:]
    elif scenario == "wrong_aruco":
        detections[0]["aruco_id"] += 100
    elif scenario == "wrong_plane":
        detections[0]["plane_id"] = "wrong_plane"
    elif scenario == "out_of_order" and iteration > 0:
        body["seq"] = 0
    elif scenario == "stale_time":
        body["capture_time_unix_ns"] -= 5_000_000_000
        body["send_time_unix_ns"] -= 5_000_000_000
    elif scenario == "future_time":
        body["capture_time_unix_ns"] += 1_000_000_000
        body["send_time_unix_ns"] += 1_000_000_000
    elif scenario == "bad_time_order":
        body["send_time_unix_ns"] = body["capture_time_unix_ns"] - 1
    elif scenario == "bad_source_declared":
        body["source_ip"] = "192.168.1.250"
    elif scenario == "empty_session":
        body["session_id"] = ""
    elif scenario == "old_version":
        body["version"] = 2


def corrupt_crc(packet: bytes) -> bytes:
    envelope = json.loads(packet.decode("utf-8"))
    old = envelope["crc32"]
    envelope["crc32"] = ("0" if old[0] != "0" else "1") + old[1:]
    return canonical_json_bytes(envelope)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--scenario", choices=SCENARIOS, default="normal")
    parser.add_argument("--omit-entity", choices=("agv1", "agv2", "agv3", "load"), default="load")
    args = parser.parse_args()

    config = load_config(Path(args.config).resolve())
    network = config["network"]
    bind = (str(network["windows_bind_ip"]), int(network["windows_bind_port"]))
    target = (str(network["robot1_ip"]), int(network["robot1_port"]))
    if bind != ("192.168.6.100", 15000):
        raise ValueError("Windows UDP endpoint must be 192.168.6.100:15000")
    if target != ("192.168.6.101", 15001):
        raise ValueError("Robot1 UDP endpoint must be 192.168.6.101:15001")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(bind)
    session = uuid.uuid4().hex
    period = 1.0 / max(args.rate, 0.1)
    try:
        for iteration in range(max(args.count, 1)):
            now_ns = time.time_ns()
            body = {
                "protocol": PROTOCOL_NAME,
                "version": PROTOCOL_VERSION,
                "session_id": session,
                "seq": iteration,
                "capture_time_unix_ns": now_ns,
                "send_time_unix_ns": now_ns + 100_000,
                "source_ip": bind[0],
                "frame_id": "world",
                "camera_alive": True,
                "frame_number": iteration,
                "processing_ms": 0.1,
                "ground_reference": ground_reference_payload(config, now_ns),
                "calibration": calibration_payload(config),
                "detections": detections_for(config, iteration),
            }
            apply_scenario(body, args.scenario, args.omit_entity, iteration)
            packet = encode_packet(body)
            if args.scenario == "bad_crc":
                packet = corrupt_crc(packet)
            sock.sendto(packet, target)
            print(
                f"sent scenario={args.scenario} seq={body['seq']} "
                f"detections={len(body['detections'])} to {target}"
            )
            time.sleep(period)
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
