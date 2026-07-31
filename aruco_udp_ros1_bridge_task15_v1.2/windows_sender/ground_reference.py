# -*- coding: utf-8 -*-
"""Dynamic metric world frame from four fixed coplanar ArUco markers.

No surveyed world coordinates are required.  Metric scale comes from the
known ground-marker side length and the calibrated pinhole camera model.
The resulting frame is deterministic by marker role:

* ``origin_id`` is world (0, 0, 0);
* ``x_axis_id`` defines world +X;
* ``y_side_id`` selects the world +Y half-plane;
* the fourth marker contributes to plane fitting and stability validation.

This module performs geometry only.  It does not filter entity poses, apply
tag-to-body transforms, open a camera, or send UDP packets.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np


def _finite_array(value, shape, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def marker_object_points(marker_length_m: float) -> np.ndarray:
    if not math.isfinite(marker_length_m) or marker_length_m <= 0.0:
        raise ValueError("marker length must be positive and finite")
    half = 0.5 * marker_length_m
    # OpenCV ArUco decoded-corner order; also required by IPPE_SQUARE.
    return np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def quadrilateral_center(corners_px: np.ndarray) -> np.ndarray:
    corners = _finite_array(corners_px, (4, 2), "marker corners")
    return np.mean(corners, axis=0)


def undistorted_ray(
    pixel_xy: Sequence[float],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    point = np.asarray(pixel_xy, dtype=np.float64).reshape(1, 1, 2)
    normalized = cv2.undistortPoints(point, camera_matrix, distortion)
    ray = np.asarray(
        [normalized[0, 0, 0], normalized[0, 0, 1], 1.0],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(ray))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("pixel produced a degenerate camera ray")
    return ray / norm


def intersect_camera_ray_with_plane(
    ray: np.ndarray,
    plane_point_camera: np.ndarray,
    plane_normal_camera: np.ndarray,
) -> Optional[np.ndarray]:
    denominator = float(np.dot(plane_normal_camera, ray))
    if abs(denominator) < 1.0e-10:
        return None
    distance = float(
        np.dot(plane_normal_camera, plane_point_camera) / denominator
    )
    if not math.isfinite(distance) or distance <= 0.0:
        return None
    return np.asarray(ray, dtype=np.float64) * distance


def _solve_square_pose(
    corners_px: np.ndarray,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, float]:
    object_points = marker_object_points(marker_length_m)
    corners = _finite_array(corners_px, (4, 2), "marker corners")
    # ITERATIVE is intentionally used here.  Some OpenCV builds return a
    # noticeably biased IPPE_SQUARE branch for nearly fronto-parallel tags;
    # the iterative solution is stable for the available four-corner square.
    flag = cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        corners,
        camera_matrix,
        distortion,
        flags=flag,
    )
    if not ok:
        raise ValueError("solvePnP failed for square marker")
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    errors = np.linalg.norm(projected.reshape(4, 2) - corners, axis=1)
    rms_px = float(np.sqrt(np.mean(errors * errors)))
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(translation)) or translation[2] <= 0.0:
        raise ValueError("solvePnP produced an invalid marker translation")
    return (
        np.asarray(rvec, dtype=np.float64).reshape(3),
        translation,
        rms_px,
    )


def _world_transform(
    origin_camera: np.ndarray,
    plane_normal_camera: np.ndarray,
    x_point_camera: np.ndarray,
    y_hint_camera: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    normal = np.asarray(plane_normal_camera, dtype=np.float64)
    normal /= np.linalg.norm(normal)
    # Ground lies in front of the camera. World +Z must point from ground
    # towards the camera, so its dot product with the ground position is < 0.
    if float(np.dot(normal, origin_camera)) > 0.0:
        normal = -normal
    x_hint = np.asarray(x_point_camera) - np.asarray(origin_camera)
    x_axis = x_hint - float(np.dot(x_hint, normal)) * normal
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm < 1.0e-9:
        raise ValueError("origin and x-axis markers are degenerate")
    x_axis /= x_norm
    y_axis = np.cross(normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    if float(np.dot(y_axis, np.asarray(y_hint_camera) - origin_camera)) <= 0.0:
        raise ValueError(
            "y-side marker is on the wrong side of the directed +X axis"
        )
    # Do not flip Y independently: with +X fixed by two marker IDs and +Z
    # fixed physically toward the overhead camera, right-handed +Y is unique.
    # Flipping Y would also flip Z and make a positive marker height point
    # below the ground plane.
    z_axis = normal

    rotation_world_from_camera = np.vstack((x_axis, y_axis, z_axis))
    world_from_camera = np.eye(4, dtype=np.float64)
    world_from_camera[:3, :3] = rotation_world_from_camera
    world_from_camera[:3, 3] = (
        -rotation_world_from_camera @ np.asarray(origin_camera)
    )
    camera_from_world = np.linalg.inv(world_from_camera)
    return world_from_camera, camera_from_world


@dataclass(frozen=True)
class GroundReferenceConfig:
    marker_ids: Tuple[int, int, int, int]
    origin_id: int
    x_axis_id: int
    y_side_id: int
    marker_length_m: float
    collection_seconds: float
    minimum_samples: int
    maximum_samples: int
    maximum_marker_reprojection_error_px: float
    maximum_repeatability_rmse_m: float
    maximum_repeatability_error_m: float
    minimum_axis_baseline_m: float
    monitor_maximum_error_m: float
    monitor_consecutive_failures: int
    maximum_reference_absence_sec: float


@dataclass(frozen=True)
class GroundReferenceSolution:
    world_from_camera: np.ndarray
    camera_from_world: np.ndarray
    plane_point_camera: np.ndarray
    plane_normal_camera: np.ndarray
    marker_positions_world: Dict[int, np.ndarray]
    fit_rmse_m: float
    fit_max_error_m: float
    repeatability_rmse_m: float
    repeatability_max_error_m: float
    inlier_count: int
    total_count: int
    validation_count: int
    sample_count: int
    calibrated_at_unix_ns: int


@dataclass
class _GroundSample:
    centers_px: Dict[int, np.ndarray]
    translations_camera: Dict[int, np.ndarray]
    reprojection_errors_px: Dict[int, float]


class GroundReferenceCalibrator:
    """Collect, solve and continuously monitor the four-marker world frame."""

    def __init__(
        self,
        config: GroundReferenceConfig,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
    ) -> None:
        if len(config.marker_ids) != 4 or len(set(config.marker_ids)) != 4:
            raise ValueError("ground reference must contain four unique IDs")
        roles = {config.origin_id, config.x_axis_id, config.y_side_id}
        if not roles.issubset(set(config.marker_ids)) or len(roles) != 3:
            raise ValueError("ground marker role IDs are invalid")
        if config.minimum_samples < 2:
            raise ValueError("minimum ground samples must be at least two")
        if config.maximum_samples < config.minimum_samples:
            raise ValueError("maximum samples is below minimum samples")
        positive = (
            config.marker_length_m,
            config.collection_seconds,
            config.maximum_marker_reprojection_error_px,
            config.maximum_repeatability_rmse_m,
            config.maximum_repeatability_error_m,
            config.minimum_axis_baseline_m,
            config.monitor_maximum_error_m,
            config.maximum_reference_absence_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("ground calibration limits must be positive and finite")
        if config.monitor_consecutive_failures <= 0:
            raise ValueError("monitor_consecutive_failures must be positive")
        self.config = config
        self.camera_matrix = _finite_array(
            camera_matrix, (3, 3), "camera matrix"
        )
        self.distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
        if self.distortion.size not in (4, 5, 8, 12, 14):
            raise ValueError("unsupported distortion coefficient count")
        if not np.all(np.isfinite(self.distortion)):
            raise ValueError("distortion coefficients must be finite")
        self.samples: List[_GroundSample] = []
        self.solution: Optional[GroundReferenceSolution] = None
        self.collection_start_monotonic: Optional[float] = None
        self.last_reference_monotonic: Optional[float] = None
        self.monitor_failures = 0
        self.last_monitor_max_error_m = math.nan

    @property
    def ready(self) -> bool:
        return self.solution is not None

    @property
    def status(self) -> str:
        if self.solution is not None:
            return "locked"
        return (
            f"collecting:{len(self.samples)}/{self.config.minimum_samples}"
        )

    def reset(self) -> None:
        self.samples = []
        self.solution = None
        self.collection_start_monotonic = None
        self.last_reference_monotonic = None
        self.monitor_failures = 0
        self.last_monitor_max_error_m = math.nan

    @staticmethod
    def _best_corner_map(
        ids: Optional[np.ndarray], corners_list: Sequence[np.ndarray]
    ) -> Dict[int, np.ndarray]:
        result: Dict[int, np.ndarray] = {}
        areas: Dict[int, float] = {}
        if ids is None:
            return result
        for raw_id, raw_corners in zip(ids.reshape(-1), corners_list):
            marker_id = int(raw_id)
            corners = np.asarray(raw_corners, dtype=np.float64).reshape(4, 2)
            area = abs(float(cv2.contourArea(corners.astype(np.float32))))
            if marker_id not in result or area > areas[marker_id]:
                result[marker_id] = corners
                areas[marker_id] = area
        return result

    def update(
        self,
        ids: Optional[np.ndarray],
        corners_list: Sequence[np.ndarray],
        now_monotonic: Optional[float] = None,
    ) -> Optional[GroundReferenceSolution]:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        corner_map = self._best_corner_map(ids, corners_list)
        visible = {
            marker_id: corner_map[marker_id]
            for marker_id in self.config.marker_ids
            if marker_id in corner_map
        }
        # At least two fixed points are required to observe an extrinsic change.
        # A lone visible marker must not keep a stale calibration alive forever.
        if len(visible) >= 2:
            self.last_reference_monotonic = now

        if self.solution is not None:
            self._monitor(visible)
            if (
                self.last_reference_monotonic is not None
                and now - self.last_reference_monotonic
                > self.config.maximum_reference_absence_sec
            ):
                self.reset()
            return self.solution

        if len(visible) != 4:
            return None
        if self.collection_start_monotonic is None:
            self.collection_start_monotonic = now
        sample = self._make_sample(visible)
        if sample is None:
            return None
        self.samples.append(sample)
        self.samples = self.samples[-self.config.maximum_samples :]
        elapsed = now - self.collection_start_monotonic
        if (
            len(self.samples) >= self.config.minimum_samples
            and elapsed >= self.config.collection_seconds
        ):
            try:
                self.solution = self._solve()
            except ValueError:
                # Keep a rolling window and continue collecting rather than
                # entering a permanent startup failure.
                self.samples = self.samples[-max(1, self.config.minimum_samples - 1) :]
                self.collection_start_monotonic = now
        return self.solution

    def _make_sample(
        self, corners_by_id: Mapping[int, np.ndarray]
    ) -> Optional[_GroundSample]:
        centers: Dict[int, np.ndarray] = {}
        translations: Dict[int, np.ndarray] = {}
        errors: Dict[int, float] = {}
        for marker_id in self.config.marker_ids:
            corners = corners_by_id[marker_id]
            try:
                _, translation, reprojection = _solve_square_pose(
                    corners,
                    self.config.marker_length_m,
                    self.camera_matrix,
                    self.distortion,
                )
            except (ValueError, cv2.error):
                return None
            if reprojection > self.config.maximum_marker_reprojection_error_px:
                return None
            centers[marker_id] = quadrilateral_center(corners)
            translations[marker_id] = translation
            errors[marker_id] = reprojection
        return _GroundSample(centers, translations, errors)

    def _solve(self) -> GroundReferenceSolution:
        median_translation = {
            marker_id: np.median(
                np.asarray(
                    [
                        sample.translations_camera[marker_id]
                        for sample in self.samples
                    ]
                ),
                axis=0,
            )
            for marker_id in self.config.marker_ids
        }
        points = np.asarray(
            [median_translation[marker_id] for marker_id in self.config.marker_ids]
        )
        centroid = np.mean(points, axis=0)
        _, singular_values, vh = np.linalg.svd(points - centroid)
        if singular_values[1] < 1.0e-8:
            raise ValueError("ground reference marker centers are collinear")
        normal = vh[-1]
        if float(np.dot(normal, centroid)) > 0.0:
            normal = -normal

        median_center = {
            marker_id: np.median(
                np.asarray(
                    [sample.centers_px[marker_id] for sample in self.samples]
                ),
                axis=0,
            )
            for marker_id in self.config.marker_ids
        }
        plane_points: Dict[int, np.ndarray] = {}
        for marker_id in self.config.marker_ids:
            ray = undistorted_ray(
                median_center[marker_id], self.camera_matrix, self.distortion
            )
            point = intersect_camera_ray_with_plane(ray, centroid, normal)
            if point is None:
                raise ValueError("ground reference ray does not intersect plane")
            plane_points[marker_id] = point

        axis_baseline = float(
            np.linalg.norm(
                plane_points[self.config.x_axis_id]
                - plane_points[self.config.origin_id]
            )
        )
        if axis_baseline < self.config.minimum_axis_baseline_m:
            raise ValueError("ground x-axis baseline is too short")
        normal_unit = normal / np.linalg.norm(normal)
        x_axis_unit = (
            plane_points[self.config.x_axis_id]
            - plane_points[self.config.origin_id]
        )
        x_axis_unit -= (
            float(np.dot(x_axis_unit, normal_unit)) * normal_unit
        )
        x_axis_unit /= np.linalg.norm(x_axis_unit)
        positive_y_axis = np.cross(normal_unit, x_axis_unit)
        y_side_baseline = float(
            np.dot(
                positive_y_axis,
                plane_points[self.config.y_side_id]
                - plane_points[self.config.origin_id],
            )
        )
        if y_side_baseline < self.config.minimum_axis_baseline_m:
            raise ValueError(
                "ground y-side marker is on the wrong side or too close "
                "to the directed x-axis"
            )
        world_from_camera, camera_from_world = _world_transform(
            plane_points[self.config.origin_id],
            normal,
            plane_points[self.config.x_axis_id],
            plane_points[self.config.y_side_id],
        )
        marker_world = {
            marker_id: (
                world_from_camera[:3, :3] @ point
                + world_from_camera[:3, 3]
            )
            for marker_id, point in plane_points.items()
        }

        plane_errors = np.abs((points - centroid) @ normal)
        temporal_errors: List[float] = []
        for sample in self.samples:
            for marker_id in self.config.marker_ids:
                ray = undistorted_ray(
                    sample.centers_px[marker_id],
                    self.camera_matrix,
                    self.distortion,
                )
                point = intersect_camera_ray_with_plane(ray, centroid, normal)
                if point is None:
                    continue
                world = (
                    world_from_camera[:3, :3] @ point
                    + world_from_camera[:3, 3]
                )
                temporal_errors.append(
                    float(np.linalg.norm(world[:2] - marker_world[marker_id][:2]))
                )
        if not temporal_errors:
            raise ValueError("ground calibration has no temporal validation data")
        repeatability_rmse = float(
            np.sqrt(np.mean(np.square(temporal_errors)))
        )
        repeatability_max = float(np.max(temporal_errors))
        if (
            repeatability_rmse > self.config.maximum_repeatability_rmse_m
            or repeatability_max > self.config.maximum_repeatability_error_m
        ):
            raise ValueError("ground reference repeatability exceeds limits")
        inlier_count = int(
            np.count_nonzero(
                np.asarray(temporal_errors)
                <= self.config.maximum_repeatability_error_m
            )
        )
        return GroundReferenceSolution(
            world_from_camera=world_from_camera,
            camera_from_world=camera_from_world,
            plane_point_camera=centroid,
            plane_normal_camera=normal,
            marker_positions_world=marker_world,
            fit_rmse_m=float(np.sqrt(np.mean(plane_errors * plane_errors))),
            fit_max_error_m=float(np.max(plane_errors)),
            repeatability_rmse_m=repeatability_rmse,
            repeatability_max_error_m=repeatability_max,
            inlier_count=inlier_count,
            total_count=len(temporal_errors),
            validation_count=len(temporal_errors),
            sample_count=len(self.samples),
            calibrated_at_unix_ns=time.time_ns(),
        )

    def _monitor(self, visible: Mapping[int, np.ndarray]) -> None:
        if self.solution is None or len(visible) < 2:
            return
        errors: List[float] = []
        solution = self.solution
        for marker_id, corners in visible.items():
            ray = undistorted_ray(
                quadrilateral_center(corners),
                self.camera_matrix,
                self.distortion,
            )
            point = intersect_camera_ray_with_plane(
                ray,
                solution.plane_point_camera,
                solution.plane_normal_camera,
            )
            if point is None:
                continue
            world = (
                solution.world_from_camera[:3, :3] @ point
                + solution.world_from_camera[:3, 3]
            )
            errors.append(
                float(
                    np.linalg.norm(
                        world[:2]
                        - solution.marker_positions_world[marker_id][:2]
                    )
                )
            )
        if len(errors) < 2:
            return
        self.last_monitor_max_error_m = max(errors)
        if self.last_monitor_max_error_m > self.config.monitor_maximum_error_m:
            self.monitor_failures += 1
        else:
            self.monitor_failures = 0
        if self.monitor_failures >= self.config.monitor_consecutive_failures:
            self.reset()


def entity_pose_on_horizontal_plane(
    corners_px: np.ndarray,
    marker_height_m: float,
    marker_length_m: float,
    solution: GroundReferenceSolution,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> Tuple[float, float, float, float]:
    """Return raw ``world_T_tag`` x/y/yaw and PnP reprojection RMS."""
    if not math.isfinite(marker_height_m) or marker_height_m < 0.0:
        raise ValueError("entity marker height must be finite and non-negative")
    corners = _finite_array(corners_px, (4, 2), "entity corners")
    _, _, reprojection_rms_px = _solve_square_pose(
        corners, marker_length_m, camera_matrix, distortion
    )
    plane_point_world = np.asarray([0.0, 0.0, marker_height_m])
    plane_point_camera = (
        solution.camera_from_world[:3, :3] @ plane_point_world
        + solution.camera_from_world[:3, 3]
    )
    plane_normal_camera = (
        solution.camera_from_world[:3, :3]
        @ np.asarray([0.0, 0.0, 1.0])
    )

    center_px = quadrilateral_center(corners)
    # Decoded corner order is (-x,+y), (+x,+y), (+x,-y), (-x,-y).
    # The midpoint of corners 1/2 therefore lies on the marker +X axis.
    heading_px = 0.5 * (corners[1] + corners[2])
    center_camera = intersect_camera_ray_with_plane(
        undistorted_ray(center_px, camera_matrix, distortion),
        plane_point_camera,
        plane_normal_camera,
    )
    heading_camera = intersect_camera_ray_with_plane(
        undistorted_ray(heading_px, camera_matrix, distortion),
        plane_point_camera,
        plane_normal_camera,
    )
    if center_camera is None or heading_camera is None:
        raise ValueError("entity marker ray does not intersect its height plane")
    center_world = (
        solution.world_from_camera[:3, :3] @ center_camera
        + solution.world_from_camera[:3, 3]
    )
    heading_world = (
        solution.world_from_camera[:3, :3] @ heading_camera
        + solution.world_from_camera[:3, 3]
    )
    direction = heading_world[:2] - center_world[:2]
    if float(np.linalg.norm(direction)) < 1.0e-9:
        raise ValueError("entity marker heading is degenerate")
    yaw = normalize_angle(math.atan2(float(direction[1]), float(direction[0])))
    return (
        float(center_world[0]),
        float(center_world[1]),
        yaw,
        reprojection_rms_px,
    )
