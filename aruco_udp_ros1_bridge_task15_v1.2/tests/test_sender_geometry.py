import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import cv2

ROOT = Path(__file__).resolve().parents[1]
SENDER = ROOT / "windows_sender"
sys.path.insert(0, str(SENDER))
spec = importlib.util.spec_from_file_location("sender", SENDER / "aruco_udp_sender.py")
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)

ground_spec = importlib.util.spec_from_file_location(
    "ground_reference", SENDER / "ground_reference.py"
)
ground = importlib.util.module_from_spec(ground_spec)
sys.modules[ground_spec.name] = ground
ground_spec.loader.exec_module(ground)


def _camera_model():
    matrix = np.array(
        [[900.0, 0.0, 640.0], [0.0, 900.0, 512.0], [0.0, 0.0, 1.0]]
    )
    distortion = np.zeros(5)
    rotation_camera_from_world = np.diag([1.0, -1.0, -1.0])
    camera_world = np.array([0.5, 0.5, 2.0])
    translation = -rotation_camera_from_world @ camera_world
    rvec, _ = cv2.Rodrigues(rotation_camera_from_world)
    return matrix, distortion, rvec, translation


def _project_marker(center, height, length, matrix, distortion, rvec, translation):
    points = ground.marker_object_points(length).copy()
    points[:, 0] += center[0]
    points[:, 1] += center[1]
    points[:, 2] += height
    image, _ = cv2.projectPoints(
        points, rvec, translation, matrix, distortion
    )
    return image.reshape(1, 4, 2).astype(np.float32)


def _calibrator():
    matrix, distortion, rvec, translation = _camera_model()
    config = ground.GroundReferenceConfig(
        marker_ids=(8, 11, 15, 30),
        origin_id=8,
        x_axis_id=11,
        y_side_id=30,
        marker_length_m=0.150,
        collection_seconds=0.1,
        minimum_samples=3,
        maximum_samples=20,
        maximum_marker_reprojection_error_px=0.5,
        maximum_repeatability_rmse_m=0.002,
        maximum_repeatability_error_m=0.005,
        minimum_axis_baseline_m=0.2,
        monitor_maximum_error_m=0.02,
        monitor_consecutive_failures=2,
        maximum_reference_absence_sec=5.0,
    )
    calibrator = ground.GroundReferenceCalibrator(
        config, matrix, distortion
    )
    centers = {
        8: (0.0, 0.0),
        11: (1.0, 0.0),
        15: (1.0, 1.0),
        30: (0.0, 1.0),
    }
    ids = np.asarray(list(centers), dtype=np.int32).reshape(-1, 1)
    corners = [
        _project_marker(
            centers[marker_id], 0.0, 0.150,
            matrix, distortion, rvec, translation)
        for marker_id in centers
    ]
    for stamp in (0.0, 0.06, 0.12):
        calibrator.update(ids, corners, now_monotonic=stamp)
    return calibrator, matrix, distortion, rvec, translation


def test_four_ground_markers_recover_metric_world_without_surveyed_coordinates():
    calibrator, matrix, distortion, rvec, translation = _calibrator()
    assert calibrator.ready
    solution = calibrator.solution
    assert solution is not None
    assert np.allclose(solution.marker_positions_world[8], [0, 0, 0], atol=1e-5)
    assert np.allclose(solution.marker_positions_world[11], [1, 0, 0], atol=1e-5)
    assert np.allclose(solution.marker_positions_world[30], [0, 1, 0], atol=1e-5)

    entity_corners = _project_marker(
        (0.4, 0.3), 0.225, 0.080,
        matrix, distortion, rvec, translation)
    x_m, y_m, yaw, reprojection = ground.entity_pose_on_horizontal_plane(
        entity_corners.reshape(4, 2),
        0.225,
        0.080,
        solution,
        matrix,
        distortion,
    )
    assert abs(x_m - 0.4) < 1e-5
    assert abs(y_m - 0.3) < 1e-5
    assert abs(yaw) < 1e-5
    assert reprojection < 1e-4


def test_current_configuration_freezes_requested_ids_and_camera_model():
    config = module.load_yaml(SENDER / "config.yaml")
    assert config["aruco"]["dictionary"] == "DICT_4X4_50"
    assert config["processing"]["width"] == 1280
    assert config["processing"]["height"] == 1024
    assert config["processing"]["require_native_resolution"] is True
    assert config["external_calibration"]["marker_ids"] == [8, 11, 15, 30]
    assert config["external_calibration"]["marker_length_m"] == 0.150
    assert config["entities_geometry"]["marker_length_m"] == 0.080
    assert config["planes"]["agv_marker_plane"]["height_m"] == 0.225
    assert {
        item["entity_id"]: item["aruco_id"] for item in config["entities"]
    } == {"agv1": 1, "agv2": 2, "agv3": 3, "load": 0}
    assert config["entities"][-1]["enabled"] is False


def test_y_side_cannot_flip_world_z_below_the_ground_plane():
    origin = np.array([0.0, 0.0, 2.0])
    normal_toward_camera = np.array([0.0, 0.0, -1.0])
    x_point = np.array([1.0, 0.0, 2.0])
    wrong_y_side = np.array([0.0, 1.0, 2.0])
    # For this camera-coordinate convention, right-handed +Y is -camera-Y.
    # The solver must reject the opposite placement instead of silently
    # turning positive marker height into a below-ground plane.
    try:
        ground._world_transform(
            origin, normal_toward_camera, x_point, wrong_y_side
        )
    except ValueError:
        pass
    else:
        raise AssertionError("wrong y-side marker placement was accepted")
