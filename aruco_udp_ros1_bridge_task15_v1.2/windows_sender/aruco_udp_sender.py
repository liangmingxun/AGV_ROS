# -*- coding: utf-8 -*-
"""Windows MVS/ArUco UDP v3 sender for the Task 15 localization chain.

The sender publishes only raw marker poses world_T_tag.  It deliberately does
not apply tag->base_link/load-center transforms, confidence gating, filtering,
or any control logic.
"""
from __future__ import annotations

import argparse
import csv
import logging
import math
import socket
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from camera_sources import CameraError, MvsCameraSource, OpenCVCameraSource
from ground_reference import (
    GroundReferenceCalibrator,
    GroundReferenceConfig,
    GroundReferenceSolution,
    entity_pose_on_horizontal_plane,
)
from protocol import PROTOCOL_NAME, PROTOCOL_VERSION, encode_packet

LOGGER = logging.getLogger("aruco_udp_sender")
EXPECTED_ENTITY_IDS = frozenset(("agv1", "agv2", "agv3", "load"))


@dataclass(frozen=True)
class EntityConfig:
    entity_id: str
    aruco_id: int
    plane_id: str
    enabled: bool


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"configuration root must be a mapping: {path}")
    return data


def make_detector(aruco_cfg: Mapping):
    dictionary_name = str(aruco_cfg.get("dictionary", "DICT_4X4_50"))
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"unsupported ArUco dictionary: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    refinement_name = str(aruco_cfg.get("corner_refinement", "SUBPIX")).upper()
    refinement_map = {
        "NONE": getattr(cv2.aruco, "CORNER_REFINE_NONE", 0),
        "SUBPIX": getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1),
        "CONTOUR": getattr(cv2.aruco, "CORNER_REFINE_CONTOUR", 2),
        "APRILTAG": getattr(cv2.aruco, "CORNER_REFINE_APRILTAG", 3),
    }
    if refinement_name not in refinement_map:
        raise ValueError(f"unsupported corner_refinement: {refinement_name}")
    if hasattr(parameters, "cornerRefinementMethod"):
        parameters.cornerRefinementMethod = refinement_map[refinement_name]
    if hasattr(parameters, "cornerRefinementWinSize"):
        parameters.cornerRefinementWinSize = int(aruco_cfg.get("corner_refinement_win_size", 5))
    if hasattr(parameters, "cornerRefinementMaxIterations"):
        parameters.cornerRefinementMaxIterations = int(
            aruco_cfg.get("corner_refinement_max_iterations", 30)
        )
    if hasattr(parameters, "cornerRefinementMinAccuracy"):
        parameters.cornerRefinementMinAccuracy = float(
            aruco_cfg.get("corner_refinement_min_accuracy", 0.01)
        )

    detector_params_cfg = aruco_cfg.get("detector_parameters", {})
    if isinstance(detector_params_cfg, Mapping):
        for name, value in detector_params_cfg.items():
            if hasattr(parameters, name):
                current = getattr(parameters, name)
                setattr(parameters, name, int(value) if isinstance(current, int) else float(value))

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return lambda image: detector.detectMarkers(image)
    return lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)


def parse_entities(cfg: Mapping) -> Tuple[Dict[int, EntityConfig], Dict[str, EntityConfig]]:
    items = cfg.get("entities", [])
    if not isinstance(items, list):
        raise ValueError("entities must be a list")
    by_aruco: Dict[int, EntityConfig] = {}
    by_entity: Dict[str, EntityConfig] = {}
    for item in items:
        if not isinstance(item, Mapping):
            raise ValueError("each entity entry must be a mapping")
        entity_id = item.get("entity_id")
        if not isinstance(entity_id, str) or entity_id not in EXPECTED_ENTITY_IDS:
            raise ValueError(f"invalid entity_id: {entity_id}")
        aruco_id = item.get("aruco_id")
        if isinstance(aruco_id, bool) or not isinstance(aruco_id, int) or aruco_id < 0:
            raise ValueError(f"invalid aruco_id for {entity_id}")
        plane_id = item.get("plane_id")
        expected_plane_id = (
            "load_marker_plane" if entity_id == "load" else "agv_marker_plane"
        )
        if (
            not isinstance(plane_id, str)
            or not plane_id
            or plane_id != expected_plane_id
        ):
            raise ValueError(f"invalid plane_id for {entity_id}")
        enabled = item.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ValueError(f"enabled must be boolean for {entity_id}")
        entity = EntityConfig(
            entity_id=entity_id,
            aruco_id=aruco_id,
            plane_id=plane_id,
            enabled=enabled,
        )
        if entity.entity_id in by_entity or entity.aruco_id in by_aruco:
            raise ValueError("duplicate entity_id or aruco_id in configuration")
        by_entity[entity.entity_id] = entity
        by_aruco[entity.aruco_id] = entity
    if set(by_entity) != EXPECTED_ENTITY_IDS:
        raise ValueError("entities must define exactly agv1, agv2, agv3 and load")
    return by_aruco, by_entity


def marker_quality(corners_px: np.ndarray, full_scale_area_px: float) -> Tuple[float, float]:
    corners_px = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    area = abs(float(cv2.contourArea(corners_px.astype(np.float32))))
    sides = np.linalg.norm(np.roll(corners_px, -1, axis=0) - corners_px, axis=1)
    side_mean = float(np.mean(sides))
    squareness = 0.0 if side_mean < 1e-6 else max(0.0, 1.0 - float(np.std(sides)) / side_mean)
    area_score = min(1.0, max(0.0, area / max(float(full_scale_area_px), 1.0)))
    return float(math.sqrt(area_score * squareness)), area


def load_camera_model(
    config: Mapping, width: int, height: int
) -> Tuple[np.ndarray, np.ndarray]:
    calibration = config.get("camera_calibration", {})
    if not isinstance(calibration, Mapping):
        raise ValueError("camera_calibration must be a mapping")
    if str(calibration.get("correction_mode", "point_model")) != "point_model":
        raise ValueError(
            "camera_calibration.correction_mode must be point_model"
        )
    if bool(calibration.get("undistort", False)):
        raise ValueError(
            "full-frame undistort must stay false in point_model mode "
            "to prevent double distortion correction"
        )
    matrix = scaled_camera_matrix(calibration, width, height)
    distortion = np.asarray(
        calibration.get("distortion_coefficients", []), dtype=np.float64
    ).reshape(-1)
    if distortion.size not in (4, 5, 8, 12, 14):
        raise ValueError("unsupported distortion coefficient count")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError("camera calibration contains non-finite values")
    return matrix, distortion


def load_ground_reference_config(config: Mapping) -> GroundReferenceConfig:
    external = config.get("external_calibration", {})
    if not isinstance(external, Mapping):
        raise ValueError("external_calibration must be a mapping")
    if str(external.get("mode", "")) != "ground_aruco_dynamic":
        raise ValueError(
            "external_calibration.mode must be ground_aruco_dynamic"
        )
    marker_ids = external.get("marker_ids", [])
    if (
        not isinstance(marker_ids, list)
        or len(marker_ids) != 4
        or any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in marker_ids
        )
    ):
        raise ValueError("external_calibration.marker_ids must contain four IDs")
    return GroundReferenceConfig(
        marker_ids=tuple(marker_ids),
        origin_id=int(external.get("origin_id")),
        x_axis_id=int(external.get("x_axis_id")),
        y_side_id=int(external.get("y_side_id")),
        marker_length_m=float(external.get("marker_length_m")),
        collection_seconds=float(external.get("collection_seconds", 1.5)),
        minimum_samples=int(external.get("minimum_samples", 15)),
        maximum_samples=int(external.get("maximum_samples", 120)),
        maximum_marker_reprojection_error_px=float(
            external.get("maximum_marker_reprojection_error_px", 1.5)
        ),
        maximum_repeatability_rmse_m=float(
            external.get("maximum_repeatability_rmse_m", 0.015)
        ),
        maximum_repeatability_error_m=float(
            external.get("maximum_repeatability_error_m", 0.030)
        ),
        minimum_axis_baseline_m=float(
            external.get("minimum_axis_baseline_m", 0.20)
        ),
        monitor_maximum_error_m=float(
            external.get("monitor_maximum_error_m", 0.040)
        ),
        monitor_consecutive_failures=int(
            external.get("monitor_consecutive_failures", 5)
        ),
        maximum_reference_absence_sec=float(
            external.get("maximum_reference_absence_sec", 5.0)
        ),
    )


def load_plane_heights(config: Mapping) -> Dict[str, float]:
    planes = config.get("planes", {})
    if not isinstance(planes, Mapping):
        raise ValueError("planes must be a mapping")
    expected = {"agv_marker_plane", "load_marker_plane"}
    if set(planes) != expected:
        raise ValueError("planes must contain agv_marker_plane and load_marker_plane")
    result: Dict[str, float] = {}
    for plane_id in sorted(expected):
        plane = planes[plane_id]
        if not isinstance(plane, Mapping):
            raise ValueError(f"planes.{plane_id} must be a mapping")
        height = float(plane.get("height_m"))
        if not math.isfinite(height) or height < 0.0:
            raise ValueError(f"planes.{plane_id}.height_m is invalid")
        result[plane_id] = height
    return result


def dynamic_calibration_payload(
    solution: GroundReferenceSolution,
    plane_heights: Mapping[str, float],
) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for plane_id, height in sorted(plane_heights.items()):
        result[f"{plane_id}_rmse_m"] = round(solution.fit_rmse_m, 6)
        result[f"{plane_id}_max_error_m"] = round(
            solution.fit_max_error_m, 6
        )
        result[f"{plane_id}_validation_rmse_m"] = round(
            solution.repeatability_rmse_m, 6
        )
        result[f"{plane_id}_validation_max_error_m"] = round(
            solution.repeatability_max_error_m, 6
        )
        result[f"{plane_id}_inlier_count"] = solution.inlier_count
        result[f"{plane_id}_total_count"] = solution.total_count
        result[f"{plane_id}_validation_count"] = solution.validation_count
        result[f"{plane_id}_height_m"] = round(float(height), 6)
    return result


def ground_reference_payload(
    config: GroundReferenceConfig,
    solution: GroundReferenceSolution,
) -> Dict[str, object]:
    return {
        "mode": "ground_aruco_dynamic",
        "marker_ids": list(config.marker_ids),
        "origin_id": config.origin_id,
        "x_axis_id": config.x_axis_id,
        "y_side_id": config.y_side_id,
        "marker_length_m": round(config.marker_length_m, 6),
        "sample_count": solution.sample_count,
        "calibrated_at_unix_ns": solution.calibrated_at_unix_ns,
    }


def detect_dynamic_world_tag_poses(
    image: np.ndarray,
    detect_markers,
    calibrator: GroundReferenceCalibrator,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    plane_heights: Mapping[str, float],
    entity_marker_length_m: float,
    by_aruco: Mapping[int, EntityConfig],
    detection_cfg: Mapping,
) -> Tuple[
    List[dict],
    Sequence[np.ndarray],
    Optional[np.ndarray],
    Optional[GroundReferenceSolution],
]:
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners_list, ids, _rejected = detect_markers(gray)
    solution = calibrator.update(ids, corners_list)
    if ids is None or solution is None:
        return [], corners_list, ids, solution

    min_area = max(0.0, float(detection_cfg.get("min_marker_area_px", 4.0)))
    full_scale_area = float(
        detection_cfg.get("quality_full_scale_area_px", 2500.0)
    )
    reprojection_scale = float(
        detection_cfg.get("quality_reprojection_scale_px", 1.5)
    )
    if not math.isfinite(reprojection_scale) or reprojection_scale <= 0.0:
        raise ValueError("quality_reprojection_scale_px must be positive")
    best_by_entity: Dict[str, dict] = {}
    for corners_raw, marker_id_raw in zip(corners_list, ids.reshape(-1)):
        aruco_id = int(marker_id_raw)
        entity = by_aruco.get(aruco_id)
        if entity is None or not entity.enabled:
            continue
        corners_px = np.asarray(corners_raw, dtype=np.float64).reshape(4, 2)
        shape_quality, area = marker_quality(corners_px, full_scale_area)
        if area < min_area:
            continue
        try:
            x_m, y_m, yaw_rad, reprojection_rms_px = (
                entity_pose_on_horizontal_plane(
                    corners_px,
                    plane_heights[entity.plane_id],
                    entity_marker_length_m,
                    solution,
                    camera_matrix,
                    distortion,
                )
            )
        except (ValueError, cv2.error):
            continue
        reprojection_quality = math.exp(
            -0.5 * (reprojection_rms_px / reprojection_scale) ** 2
        )
        quality = min(
            1.0, max(0.0, shape_quality * reprojection_quality)
        )
        center_px = np.mean(corners_px, axis=0)
        detection = {
            "entity_id": entity.entity_id,
            "aruco_id": entity.aruco_id,
            "x_m": round(x_m, 6),
            "y_m": round(y_m, 6),
            "yaw_rad": round(yaw_rad, 7),
            "quality": round(quality, 4),
            "marker_area_px": round(area, 1),
            "pixel_x": round(float(center_px[0]), 2),
            "pixel_y": round(float(center_px[1]), 2),
            "plane_id": entity.plane_id,
        }
        previous = best_by_entity.get(entity.entity_id)
        if previous is None or (
            detection["quality"], detection["marker_area_px"]
        ) > (previous["quality"], previous["marker_area_px"]):
            best_by_entity[entity.entity_id] = detection
    return (
        [best_by_entity[key] for key in sorted(best_by_entity)],
        corners_list,
        ids,
        solution,
    )


def detect_pixel_only(
    image: np.ndarray,
    detect_markers,
    by_aruco: Mapping[int, EntityConfig],
    detection_cfg: Mapping,
    ground_marker_ids: Sequence[int] = (),
) -> Tuple[List[dict], Sequence[np.ndarray], Optional[np.ndarray]]:
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners_list, ids, _rejected = detect_markers(gray)
    if ids is None:
        return [], corners_list, ids
    full_scale_area = float(detection_cfg.get("quality_full_scale_area_px", 2500.0))
    ground_marker_set = set(int(value) for value in ground_marker_ids)
    results = []
    for corners_raw, marker_id_raw in zip(corners_list, ids.reshape(-1)):
        marker_id = int(marker_id_raw)
        entity = by_aruco.get(marker_id)
        is_ground = marker_id in ground_marker_set
        if entity is None and not is_ground:
            continue
        corners_px = np.asarray(corners_raw, dtype=np.float64).reshape(4, 2)
        quality, area = marker_quality(corners_px, full_scale_area)
        center = np.mean(corners_px, axis=0)
        results.append(
            {
                "entity_id": (
                    entity.entity_id
                    if entity is not None
                    else f"ground_ref_{marker_id}"
                ),
                "aruco_id": marker_id,
                "quality": quality,
                "marker_area_px": area,
                "pixel_x": float(center[0]),
                "pixel_y": float(center[1]),
            }
        )
    return results, corners_list, ids


class CsvPoseLogger:
    def __init__(self, enabled: bool, directory: Path, flush_interval_sec: float) -> None:
        self._file = None
        self._writer = None
        self._last_flush = time.monotonic()
        self._flush_interval_sec = max(float(flush_interval_sec), 0.0)
        if not enabled:
            return
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / time.strftime("task15_raw_tag_%Y%m%d_%H%M%S.csv")
        self._file = path.open("w", newline="", encoding="utf-8-sig", buffering=65536)
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "capture_time_unix_ns", "send_time_unix_ns", "seq", "frame_number",
                "entity_id", "aruco_id", "plane_id", "x_m", "y_m", "yaw_rad",
                "quality", "marker_area_px", "pixel_x", "pixel_y",
                "plane_height_m", "fit_rmse_m", "fit_max_error_m",
                "validation_rmse_m", "validation_max_error_m",
                "fit_inlier_count", "fit_total_count", "validation_count",
            ]
        )
        LOGGER.info("CSV logging: %s", path)

    def write(self, body: Mapping) -> None:
        if self._writer is None or self._file is None:
            return
        for detection in body.get("detections", []):
            plane_id = detection["plane_id"]
            calibration = body["calibration"]
            self._writer.writerow(
                [
                    body["capture_time_unix_ns"], body["send_time_unix_ns"], body["seq"],
                    body.get("frame_number"), detection["entity_id"], detection["aruco_id"],
                    detection["plane_id"], detection["x_m"], detection["y_m"],
                    detection["yaw_rad"], detection["quality"], detection["marker_area_px"],
                    detection["pixel_x"], detection["pixel_y"],
                    calibration[f"{plane_id}_height_m"],
                    calibration[f"{plane_id}_rmse_m"],
                    calibration[f"{plane_id}_max_error_m"],
                    calibration[f"{plane_id}_validation_rmse_m"],
                    calibration[f"{plane_id}_validation_max_error_m"],
                    calibration[f"{plane_id}_inlier_count"],
                    calibration[f"{plane_id}_total_count"],
                    calibration[f"{plane_id}_validation_count"],
                ]
            )
        now = time.monotonic()
        if self._flush_interval_sec == 0.0 or now - self._last_flush >= self._flush_interval_sec:
            self._file.flush()
            self._last_flush = now

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()


def build_camera(config: Mapping, config_dir: Path):
    camera_cfg = config["camera"]
    backend = str(camera_cfg.get("backend", "mvs")).lower()
    if backend == "mvs":
        mvimport = Path(str(camera_cfg.get("mvimport_dir", "MvImport")))
        if not mvimport.is_absolute():
            mvimport = config_dir / mvimport
        return MvsCameraSource(str(mvimport), int(camera_cfg.get("device_index", 0)))
    if backend == "opencv":
        return OpenCVCameraSource(
            int(camera_cfg.get("device_index", 0)), str(camera_cfg.get("video_file", ""))
        )
    raise ValueError(f"unsupported camera backend: {backend}")


def scaled_camera_matrix(calibration: Mapping, width: int, height: int) -> np.ndarray:
    matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64).copy()
    if matrix.shape != (3, 3):
        raise ValueError("camera_matrix must be 3 x 3")
    source_width = int(calibration.get("image_width", width))
    source_height = int(calibration.get("image_height", height))
    if source_width <= 0 or source_height <= 0:
        raise ValueError("calibration image dimensions must be positive")
    matrix[0, 0] *= width / source_width
    matrix[0, 2] *= width / source_width
    matrix[1, 1] *= height / source_height
    matrix[1, 2] *= height / source_height
    return matrix


def preprocess_image(image: np.ndarray, config: Mapping) -> np.ndarray:
    proc = config["processing"]
    width = int(proc["width"])
    height = int(proc["height"])
    if width <= 0 or height <= 0:
        raise ValueError("processing dimensions must be positive")
    if image.shape[1] != width or image.shape[0] != height:
        if bool(proc.get("require_native_resolution", True)):
            raise ValueError(
                "camera frame resolution "
                f"{image.shape[1]}x{image.shape[0]} does not match the "
                f"calibrated {width}x{height} geometry"
            )
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    calibration = config.get("camera_calibration", {})
    if bool(calibration.get("undistort", False)):
        matrix = scaled_camera_matrix(calibration, width, height)
        distortion = np.asarray(calibration["distortion_coefficients"], dtype=np.float64).reshape(-1)
        if distortion.size not in (4, 5, 8, 12, 14):
            raise ValueError("unsupported distortion coefficient count")
        image = cv2.undistort(image, matrix, distortion, None, matrix)
    return image


def draw_world_overlay(image: np.ndarray, detections: Iterable[Mapping], fps: float, target: str) -> None:
    for detection in detections:
        center = (int(round(detection["pixel_x"])), int(round(detection["pixel_y"])))
        cv2.circle(image, center, 5, (0, 255, 0), -1)
        text = (
            f"{detection['entity_id']} tag={detection['aruco_id']} "
            f"x={detection['x_m']:.3f} y={detection['y_m']:.3f} "
            f"yaw={detection['yaw_rad']:.3f} q={detection['quality']:.2f}"
        )
        cv2.putText(image, text, (center[0] + 8, center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(image, f"FPS={fps:.1f} UDPv3->{target}", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def draw_detect_only_overlay(image: np.ndarray, detections: Iterable[Mapping], fps: float) -> None:
    for detection in detections:
        center = (int(round(detection["pixel_x"])), int(round(detection["pixel_y"])))
        cv2.circle(image, center, 5, (0, 255, 255), -1)
        text = (
            f"{detection['entity_id']} ID={detection['aruco_id']} "
            f"px=({detection['pixel_x']:.1f},{detection['pixel_y']:.1f}) "
            f"q={detection['quality']:.2f}"
        )
        cv2.putText(image, text, (center[0] + 8, center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, f"DETECT-ONLY FPS={fps:.1f} no UDP", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def capture_preprocessed_frame(config: Mapping, config_dir: Path, output: Path) -> None:
    camera = build_camera(config, config_dir)
    try:
        frame, _ = camera.read(int(config["camera"].get("timeout_ms", 1000)))
        if frame is None:
            raise RuntimeError("failed to capture frame")
        frame = preprocess_image(frame, config)
        output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output), frame):
            raise RuntimeError(f"failed to save frame: {output}")
        LOGGER.info("saved preprocessed frame: %s", output)
    finally:
        camera.close()
        cv2.destroyAllWindows()


def run(config_path: Path, save_frame: Optional[Path], dry_run: bool, detect_only: bool) -> int:
    config = load_yaml(config_path)
    configure_logging(str(config.get("logging", {}).get("level", "INFO")))
    if config.get("frame_id") != "world":
        raise ValueError("Task 15 frame_id must be world")
    config_dir = config_path.parent
    if save_frame is not None:
        capture_preprocessed_frame(config, config_dir, save_frame)
        return 0

    by_aruco, by_entity = parse_entities(config)
    detector = make_detector(config.get("aruco", {}))
    processing_width = int(config["processing"]["width"])
    processing_height = int(config["processing"]["height"])
    camera_matrix, distortion = load_camera_model(
        config, processing_width, processing_height
    )
    plane_heights = load_plane_heights(config)
    ground_calibrator: Optional[GroundReferenceCalibrator] = None
    ground_reference_config = load_ground_reference_config(config)
    if set(by_aruco).intersection(ground_reference_config.marker_ids):
        raise ValueError(
            "entity ArUco IDs must be distinct from ground-reference IDs"
        )
    entity_marker_length_m = float(
        config.get("entities_geometry", {}).get("marker_length_m", 0.0)
    )
    if (
        not math.isfinite(entity_marker_length_m)
        or entity_marker_length_m <= 0.0
    ):
        raise ValueError("entities_geometry.marker_length_m must be positive")
    if not detect_only:
        ground_calibrator = GroundReferenceCalibrator(
            ground_reference_config,
            camera_matrix,
            distortion,
        )

    network = config["network"]
    bind_ip = str(network.get("windows_bind_ip", "192.168.6.100"))
    bind_port = int(network.get("windows_bind_port", 15000))
    target_ip = str(network.get("robot1_ip", "192.168.6.101"))
    target_port = int(network.get("robot1_port", 15001))
    target = (target_ip, target_port)
    if (bind_ip, bind_port) != ("192.168.6.100", 15000):
        raise ValueError("Windows UDP endpoint must be 192.168.6.100:15000")
    if target != ("192.168.6.101", 15001):
        raise ValueError("Robot1 UDP endpoint must be 192.168.6.101:15001")
    try:
        socket.inet_aton(bind_ip)
        socket.inet_aton(target_ip)
    except OSError as exc:
        raise ValueError("network addresses must be valid IPv4 addresses") from exc
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
    if not dry_run and not detect_only:
        try:
            udp_socket.bind((bind_ip, bind_port))
        except OSError as exc:
            raise RuntimeError(
                f"cannot bind UDP source {bind_ip}:{bind_port}; verify the Windows NIC"
            ) from exc

    csv_cfg = config.get("csv_log", {})
    log_dir = Path(str(csv_cfg.get("directory", "logs")))
    if not log_dir.is_absolute():
        log_dir = config_dir / log_dir
    pose_logger = CsvPoseLogger(
        enabled=bool(csv_cfg.get("enabled", True)) and not detect_only,
        directory=log_dir,
        flush_interval_sec=float(csv_cfg.get("flush_interval_sec", 1.0)),
    )

    camera = build_camera(config, config_dir)
    session_id = uuid.uuid4().hex
    seq = 0
    next_send_deadline = 0.0
    last_frame_perf = time.perf_counter()
    smoothed_fps = 0.0
    max_rate_hz = float(network.get("max_send_rate_hz", 30.0))
    min_send_period = 0.0 if max_rate_hz <= 0.0 else 1.0 / max_rate_hz
    preview = bool(config.get("display", {}).get("enabled", True))
    window_name = str(config.get("display", {}).get("window_name", "Task15 ArUco"))
    camera_failures = 0

    try:
        while True:
            frame, frame_number = camera.read(int(config["camera"].get("timeout_ms", 1000)))
            capture_time_unix_ns = time.time_ns()
            frame_perf = time.perf_counter()
            if frame is None:
                camera_failures += 1
                if camera_failures % 10 == 1:
                    LOGGER.warning("camera frame timeout/failure count=%d", camera_failures)
                time.sleep(0.01)
                continue
            camera_failures = 0
            frame = preprocess_image(frame, config)
            start_process = time.perf_counter()
            ground_solution: Optional[GroundReferenceSolution] = None
            if detect_only:
                detections, marker_corners, marker_ids = detect_pixel_only(
                    frame,
                    detector,
                    by_aruco,
                    config.get("detection", {}),
                    ground_reference_config.marker_ids,
                )
            else:
                if ground_calibrator is None:
                    raise RuntimeError("ground calibrator was not initialized")
                (
                    detections,
                    marker_corners,
                    marker_ids,
                    ground_solution,
                ) = detect_dynamic_world_tag_poses(
                    frame,
                    detector,
                    ground_calibrator,
                    camera_matrix,
                    distortion,
                    plane_heights,
                    entity_marker_length_m,
                    by_aruco,
                    config.get("detection", {}),
                )
            processing_ms = (time.perf_counter() - start_process) * 1000.0

            dt = max(frame_perf - last_frame_perf, 1e-6)
            instant_fps = 1.0 / dt
            smoothed_fps = instant_fps if smoothed_fps <= 0.0 else 0.9 * smoothed_fps + 0.1 * instant_fps
            last_frame_perf = frame_perf

            send_due = (
                min_send_period <= 0.0
                or next_send_deadline <= 0.0
                or frame_perf >= next_send_deadline
            )
            if not detect_only and ground_solution is not None and send_due:
                send_time_unix_ns = time.time_ns()
                body = {
                    "protocol": PROTOCOL_NAME,
                    "version": PROTOCOL_VERSION,
                    "session_id": session_id,
                    "seq": seq,
                    "capture_time_unix_ns": capture_time_unix_ns,
                    "send_time_unix_ns": send_time_unix_ns,
                    "source_ip": bind_ip,
                    "frame_id": str(config.get("frame_id", "world")),
                    "camera_alive": True,
                    "frame_number": int(frame_number) if frame_number is not None else seq,
                    "processing_ms": round(processing_ms, 3),
                    "ground_reference": ground_reference_payload(
                        ground_reference_config, ground_solution
                    ),
                    "calibration": dynamic_calibration_payload(
                        ground_solution, plane_heights
                    ),
                    "detections": detections,
                }
                packet = encode_packet(body)
                if not dry_run:
                    udp_socket.sendto(packet, target)
                pose_logger.write(body)
                seq += 1
                if min_send_period > 0.0:
                    if next_send_deadline <= 0.0:
                        next_send_deadline = frame_perf + min_send_period
                    else:
                        # Advance from the scheduled deadline rather than the
                        # actual frame time. This prevents frame quantization
                        # latency from accumulating and pulling a requested
                        # 30 Hz stream down to roughly 26 Hz.
                        next_send_deadline += min_send_period
                        if next_send_deadline <= frame_perf:
                            missed_periods = (
                                math.floor(
                                    (frame_perf - next_send_deadline)
                                    / min_send_period
                                )
                                + 1
                            )
                            next_send_deadline += (
                                missed_periods * min_send_period
                            )

            if preview:
                if marker_ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
                if detect_only:
                    draw_detect_only_overlay(frame, detections, smoothed_fps)
                else:
                    draw_world_overlay(frame, detections, smoothed_fps, f"{target_ip}:{target_port}")
                    if ground_calibrator is not None:
                        cv2.putText(
                            frame,
                            "GROUND="
                            + ground_calibrator.status
                            + " monitor_max="
                            + (
                                f"{ground_calibrator.last_monitor_max_error_m:.3f}m"
                                if math.isfinite(
                                    ground_calibrator.last_monitor_max_error_m
                                )
                                else "n/a"
                            ),
                            (12, 48),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.50,
                            (0, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
    finally:
        pose_logger.close()
        camera.close()
        udp_socket.close()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--save-frame", default="", help="save one preprocessed frame and exit")
    parser.add_argument("--dry-run", action="store_true", help="formal world_T_tag processing without UDP")
    parser.add_argument(
        "--detect-only",
        action="store_true",
        help="read camera and display ArUco IDs/pixel centers without calibration or UDP",
    )
    args = parser.parse_args()
    try:
        return run(
            Path(args.config).resolve(),
            Path(args.save_frame).resolve() if args.save_frame else None,
            bool(args.dry_run),
            bool(args.detect_only),
        )
    except (CameraError, OSError, ValueError, RuntimeError, KeyError, cv2.error) as exc:
        LOGGER.error("startup/runtime failure: %s", exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
