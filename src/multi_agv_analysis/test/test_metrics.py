#!/usr/bin/env python3

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

import genpy
from agv_msgs.msg import CooperativeState, PathReference
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, UInt64

from multi_agv_analysis.conversion import RAW_SCHEMAS, _aligned_rows, _extract
from multi_agv_analysis.approval import verify_approved_configuration
from multi_agv_analysis.io_utils import (
    atomic_dump_yaml,
    read_csv,
    sha256_file,
    write_csv,
)
from multi_agv_analysis.metrics import compute_metrics
from multi_agv_analysis.validation import validate_converted_run


class ConfigurationApprovalTest(unittest.TestCase):
    def test_rehearsal_passes_but_formal_and_mutation_are_refused(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / "method.yaml"
            config.write_text("mode: M2b\n", encoding="utf-8")
            registry = root / "registry.yaml"
            atomic_dump_yaml(registry, {
                "approvals": {
                    "rehearsal": {
                        "status": "software_rehearsal_only",
                        "formal_statistics_authorized": False,
                        "config_sha256": {
                            config.name: sha256_file(config),
                        },
                    },
                },
            })
            result = verify_approved_configuration(
                registry, "rehearsal", [config])
            self.assertEqual(result["status"], "software_rehearsal_only")
            with self.assertRaises(RuntimeError):
                verify_approved_configuration(
                    registry, "rehearsal", [config], True)
            config.write_text("mode: M1\n", encoding="utf-8")
            with self.assertRaises(RuntimeError):
                verify_approved_configuration(
                    registry, "rehearsal", [config])


class CameraConversionTest(unittest.TestCase):
    def test_camera_and_world_pose_fields_are_losslessly_exported(self):
        stamp = genpy.Time.from_sec(12.5)
        camera = PoseStamped()
        camera.header.seq = 1234
        camera.header.stamp = stamp
        camera.header.frame_id = "world@00000011"
        camera.pose.position.x = 1.25
        camera.pose.position.y = -0.4
        filename, row = _extract(
            "/pose_provider/agv1/base_pose_raw", stamp, camera)
        self.assertEqual(filename, "camera_pose.csv")
        self.assertEqual(row["header_stamp"], 12.5)
        self.assertEqual(row["header_seq"], 1234)
        self.assertEqual(row["calibration_epoch_token"], 17)
        self.assertEqual(row["position_x"], 1.25)
        self.assertEqual(row["position_y"], -0.4)

        filename, row = _extract(
            "/camera/world/agv1_confidence", stamp, Float64(data=0.92))
        self.assertEqual(filename, "camera_confidence.csv")
        self.assertEqual(row["header_stamp"], 12.5)
        self.assertEqual(row["confidence"], 0.92)

        filename, row = _extract(
            "/vision/aruco/calibration_epoch", stamp,
            UInt64(data=1234567890123456789))
        self.assertEqual(filename, "camera_calibration_epoch.csv")
        self.assertEqual(row["epoch_unix_ns"], 1234567890123456789)

        state = CooperativeState()
        state.header.stamp = stamp
        state.robot_pose[0].x = 0.1
        state.robot_pose[0].y = 0.2
        state.robot_pose[0].theta = 0.3
        state.support_pose[0].x = 0.4
        state.load_pose.x = 0.5
        state.load_pose.y = 0.6
        state.load_pose.theta = 0.7
        filename, row = _extract(
            "/multi_agv/cooperative_state", stamp, state)
        self.assertEqual(filename, "cooperative_state.csv")
        self.assertEqual(row["robot_pose_x_1"], 0.1)
        self.assertEqual(row["robot_pose_y_1"], 0.2)
        self.assertEqual(row["robot_pose_yaw_1"], 0.3)
        self.assertEqual(row["support_pose_x_1"], 0.4)
        self.assertEqual(row["load_pose_x"], 0.5)
        self.assertEqual(row["load_pose_y"], 0.6)
        self.assertEqual(row["load_pose_yaw"], 0.7)

        reference = PathReference()
        reference.header.stamp = stamp
        reference.support_pose_reference[0].x = 1.0
        reference.support_pose_reference[0].y = 2.0
        reference.support_pose_reference[0].theta = 0.4
        filename, row = _extract(
            "/multi_agv/path_reference", stamp, reference)
        self.assertEqual(filename, "path_reference.csv")
        self.assertEqual(row["support_x_reference_1"], 1.0)
        self.assertEqual(row["support_y_reference_1"], 2.0)
        self.assertEqual(row["support_yaw_reference_1"], 0.4)

    def test_internal_debug_and_support_references_are_causally_aligned(self):
        raw = {name: [] for name in RAW_SCHEMAS}
        state = {
            "header_stamp": 1.0, "load_pose_valid": True,
            "load_path_state_valid": True, "load_s_actual": 0.1,
            "load_s_dot_actual": 0.2}
        for robot in range(1, 4):
            for prefix in (
                    "robot_pose_valid", "support_pose_valid",
                    "path_state_valid"):
                state["{}_{}".format(prefix, robot)] = True
            state["s_actual_{}".format(robot)] = 0.1
            state["s_dot_actual_{}".format(robot)] = 0.2
            state["support_pose_x_{}".format(robot)] = float(robot)
            state["support_pose_y_{}".format(robot)] = 0.0
        raw["cooperative_state.csv"] = [state]
        path = {
            "header_stamp": 1.0, "load_s_reference": 0.0,
            "load_velocity_reference": 0.2}
        for robot in range(1, 4):
            path["support_x_reference_{}".format(robot)] = float(robot)
            path["support_y_reference_{}".format(robot)] = 0.0
            path["support_yaw_reference_{}".format(robot)] = 0.0
        raw["path_reference.csv"] = [path]
        formal = [0.0] * 90
        formal[9 + 16] = 0.25
        raw["formal_algorithm_state.csv"] = [{
            "header_stamp": 1.0,
            "layout_label": "formal_algorithm_state_v1:header9+3x27",
            "data_json": json.dumps(formal)}]
        limiter = [1.0, 1.0, 0.75, 0.12]
        for robot in range(3):
            limiter.extend([
                0.10 + robot * 0.01, 0.12 + robot * 0.01,
                0.075 + robot * 0.01, 0.09 + robot * 0.01,
                0.08, 0.08, 0.0])
        raw["formal_execution_limiter_state.csv"] = [{
            "header_stamp": 1.0,
            "layout_label": "formal_execution_limiter_v1:header4+3x7",
            "data_json": json.dumps(limiter)}]
        m2b = [0.0] * 66
        m2b[6], m2b[26], m2b[46] = 0.0, 0.2, 0.5
        m2b[7], m2b[27], m2b[47] = 0.1, 0.15, 0.4
        raw["m2b_algorithm_state.csv"] = [{
            "header_stamp": 1.0,
            "layout_label": "m2b_algorithm_state_v1:header6+3x20",
            "data_json": json.dumps(m2b)}]
        aligned = _aligned_rows(raw, 0.2)
        self.assertEqual(len(aligned), 1)
        self.assertEqual(aligned[0]["agv1_support_reference_x"], 1.0)
        self.assertEqual(aligned[0]["agv1_psi"], 0.25)
        self.assertTrue(math.isnan(
            aligned[0]["agv1_mapped_path_velocity_upper"]))
        self.assertEqual(aligned[0]["agv1_boundary_lower"], 0.0)
        self.assertAlmostEqual(
            aligned[0]["agv1_wheel_right_pre_limit"], 0.12)
        self.assertAlmostEqual(aligned[0]["fleet_scale"], 0.75)
        self.assertAlmostEqual(aligned[0]["m2b_delta_z"], 0.5)
        self.assertAlmostEqual(aligned[0]["m2b_delta_w"], 0.3)

    def test_v2_keeps_mapped_capability_distinct_from_dynamic_boundary(self):
        raw = {name: [] for name in RAW_SCHEMAS}
        state = {
            "header_stamp": 1.0, "load_pose_valid": True,
            "load_path_state_valid": True}
        for robot in range(1, 4):
            state["robot_pose_valid_{}".format(robot)] = True
            state["support_pose_valid_{}".format(robot)] = True
            state["path_state_valid_{}".format(robot)] = True
        raw["cooperative_state.csv"] = [state]
        debug = [0.0] * 97
        debug[0] = 1.0
        debug[9] = 1.0
        for robot in range(3):
            offset = 10 + robot * 29
            debug[offset] = -0.04
            debug[offset + 1] = 0.06
            debug[offset + 27] = -0.09
            debug[offset + 28] = 0.09
        raw["formal_algorithm_state.csv"] = [{
            "header_stamp": 1.0,
            "layout_label": "formal_algorithm_state_v2:header10+3x29",
            "data_json": json.dumps(debug)}]
        aligned = _aligned_rows(raw, 0.2)[0]
        self.assertTrue(aligned["algorithm_state_available"])
        self.assertTrue(aligned["algorithm_valid"])
        self.assertEqual(aligned["agv1_boundary_upper"], 0.06)
        self.assertEqual(
            aligned["agv1_mapped_path_velocity_upper"], 0.09)
        self.assertTrue(aligned["mapped_path_capability_available"])


class MetricsTest(unittest.TestCase):
    @staticmethod
    def fixture():
        rows = []
        raw_left = [0.5, 1.2, 1.3, 0.8, 1.1, 0.7, 0.6, 0.6, 0.6, 0.6]
        applied_left = [0.5, 1.0, 1.0, 0.8, 1.1, 0.7, 0.6, 0.6, 0.6, 0.6]
        for index in range(10):
            row = {
                "stamp": index * 0.1,
                "localization_valid": True,
                "load_s_reference": index * 0.1,
                "load_s_actual": round(index * 0.1 + 0.2, 10),
                "agv2_reported_velocity_limit": (
                    0.5 if index < 6 else 0.95),
                "agv2_mapped_path_velocity_upper": (
                    0.5 if index < 6 else 0.95),
                "fleet_scale": 0.8 if index in (1, 2) else 1.0,
                "common_velocity_reference": (
                    0.5 if index < 8 else 0.95),
            }
            for robot in range(1, 4):
                for side in ("left", "right"):
                    prefix = "agv{}_wheel_{}".format(robot, side)
                    raw = (
                        raw_left[index]
                        if robot == 1 and side == "left" else 0.4)
                    applied = (
                        applied_left[index]
                        if robot == 1 and side == "left" else raw)
                    actual = applied
                    if robot == 1 and side == "left":
                        actual += 0.1
                    row[prefix + "_raw"] = raw
                    row[prefix + "_pre_limit"] = raw
                    row[prefix + "_fleet_scaled"] = raw
                    row[prefix + "_applied"] = applied
                    row[prefix + "_actual"] = actual
                    row[prefix + "_reported_limit"] = 1.0
            rows.append(row)
        return rows

    def test_hand_computed_ten_sample_fixture(self):
        result = compute_metrics(
            self.fixture(), sample_period=0.1, command_epsilon=1e-9,
            nominal_agv2_capability=1.0,
            nominal_common_velocity=1.0,
            evaluation_target=1.1)
        self.assertEqual(result["wheel"]["demand_exceedance_samples"], 3)
        self.assertAlmostEqual(
            result["wheel"]["demand_exceedance_time"], 0.3)
        self.assertEqual(result["wheel"]["limited_samples"], 2)
        self.assertAlmostEqual(result["wheel"]["limited_time"], 0.2)
        self.assertAlmostEqual(
            result["wheel"]["maximum_demand_ratio"], 1.3)
        self.assertAlmostEqual(
            result["wheel"]["maximum_demand_exceedance"], 0.3)
        self.assertAlmostEqual(result["wheel"]["minimum_fleet_scale"], 0.8)
        self.assertAlmostEqual(
            result["wheel"]["fleet_scaling_duration"], 0.2)
        self.assertAlmostEqual(
            result["wheel"]["fleet_scaling_ratio"], 0.2)
        self.assertAlmostEqual(
            result["wheel"]["tracking_rmse"],
            math.sqrt(10.0 * 0.1 ** 2 / 60.0))
        self.assertAlmostEqual(result["wheel"]["maximum_actual_speed"], 1.2)
        self.assertEqual(
            result["wheel"]["actual_above_reported_limit_samples"], 3)
        self.assertAlmostEqual(
            result["wheel"]["actual_above_reported_limit_time"], 0.3)
        self.assertAlmostEqual(result["path"]["progress_rmse"], 0.2)
        self.assertAlmostEqual(
            result["recovery"]["capability_95_time"], 0.6)
        self.assertAlmostEqual(
            result["recovery"]["common_velocity_after_capability_delay"], 0.2)
        self.assertAlmostEqual(result["task"]["completion_time"], 0.9)
        self.assertEqual(
            result["task"]["completion_scope"], "all_recorded_samples")
        self.assertTrue(result["causal_unfiltered"])
        self.assertEqual(
            result["evaluation_window"]["source"],
            "all_samples_fallback")
        self.assertFalse(
            result["evaluation_window"]["formal_statistics_ready"])

    def test_chassis_limiting_uses_same_feedback_raw_and_applied(self):
        rows = self.fixture()
        for row in rows:
            row["agv1_wheel_left_pre_limit"] = 1.2
            row["agv1_wheel_left_raw"] = 0.8
            row["agv1_wheel_left_applied"] = 0.8
        result = compute_metrics(rows, sample_period=0.1)
        self.assertEqual(result["wheel"]["limited_samples"], 0)
        self.assertAlmostEqual(result["wheel"]["limited_time"], 0.0)

    def test_invalid_localization_is_excluded_from_path_and_geometry(self):
        rows = self.fixture()
        rows[0].update({
            "localization_valid": False,
            "load_s_actual": 100.0,
            "load_pose_x": 100.0,
            "load_pose_y": 100.0,
            "load_x_reference": 0.0,
            "load_y_reference": 0.0,
        })
        rows[1].update({
            "load_pose_x": 0.0,
            "load_pose_y": 0.0,
            "load_x_reference": 0.0,
            "load_y_reference": 0.0,
        })
        result = compute_metrics(rows, sample_period=0.1)
        self.assertEqual(result["sample_counts"]["invalid_localization"], 1)
        self.assertLess(result["path"]["progress_max_absolute"], 1.0)
        self.assertLess(result["geometry"]["load_position_max"], 1.0)

    def test_invalid_algorithm_excludes_default_support_references(self):
        rows = self.fixture()[:2]
        points = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0))
        for row in rows:
            row["algorithm_valid"] = True
            for robot, point in enumerate(points, start=1):
                row["agv{}_support_pose_x".format(robot)] = point[0]
                row["agv{}_support_pose_y".format(robot)] = point[1]
                row["agv{}_support_reference_x".format(robot)] = point[0]
                row["agv{}_support_reference_y".format(robot)] = point[1]
        rows[0]["algorithm_valid"] = False
        for robot in range(1, 4):
            rows[0]["agv{}_support_reference_x".format(robot)] = 0.0
            rows[0]["agv{}_support_reference_y".format(robot)] = 0.0
        result = compute_metrics(rows, sample_period=0.1)
        self.assertEqual(result["sample_counts"]["invalid_algorithm"], 1)
        self.assertAlmostEqual(
            result["geometry"]["rigid_fit_residual_rmse"], 0.0)
        for robot in range(1, 4):
            self.assertAlmostEqual(result["geometry"]["support"][
                "agv{}".format(robot)]["position_rmse"], 0.0)

    def test_experiment_state_window_is_reported(self):
        rows = self.fixture()
        for index, row in enumerate(rows):
            row["experiment_state_available"] = True
            row["evaluation_active"] = 2 <= index < 8
        result = compute_metrics(rows, sample_period=0.1)
        self.assertEqual(result["sample_counts"]["aligned"], 6)
        self.assertEqual(result["sample_counts"]["experiment_state"], 10)
        self.assertEqual(
            result["evaluation_window"]["source"], "experiment_state")
        self.assertTrue(
            result["evaluation_window"]["formal_statistics_ready"])

    def test_geometry_capability_and_internal_metrics(self):
        row = self.fixture()[0]
        row.update({
            "load_pose_x": 0.1, "load_pose_y": 0.0,
            "load_pose_yaw": 0.1,
            "load_x_reference": 0.0, "load_y_reference": 0.0,
            "load_yaw_reference": 0.0,
            "common_boundary_upper": 0.4,
            "m2b_delta_z": 0.02, "m2b_delta_w": 0.03,
        })
        points = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0))
        for robot, point in enumerate(points, start=1):
            row["agv{}_s_actual".format(robot)] = 0.1
            row["agv{}_s_dot_actual".format(robot)] = 0.2
            row["agv{}_support_reference_x".format(robot)] = point[0]
            row["agv{}_support_reference_y".format(robot)] = point[1]
            row["agv{}_support_pose_x".format(robot)] = point[0] + 0.1
            row["agv{}_support_pose_y".format(robot)] = point[1]
            row["agv{}_mapped_path_velocity_upper".format(robot)] = 0.5
            row["m2b_agv{}_reported_capability".format(robot)] = 0.5
            row["agv{}_psi".format(robot)] = 0.01 * robot
            row["agv{}_composite_error".format(robot)] = 0.02 * robot
            row["agv{}_theta_hat_1".format(robot)] = 0.03 * robot
            row["agv{}_theta_hat_2".format(robot)] = -0.03 * robot
            row["agv{}_disturbance_estimate".format(robot)] = 0.04 * robot
        result = compute_metrics([row], sample_period=0.1)
        self.assertAlmostEqual(
            result["geometry"]["load_position_rmse"], 0.1)
        self.assertAlmostEqual(
            result["geometry"]["rigid_fit_residual_rmse"], 0.0)
        self.assertAlmostEqual(
            result["capability"]["link_margin_minimum"], 0.1)
        self.assertAlmostEqual(
            result["internal"]["m2b_delta_z_max"], 0.02)

    def test_invalid_metric_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            compute_metrics(
                self.fixture(), sample_period=0.1, command_epsilon=-1.0)
        with self.assertRaises(ValueError):
            compute_metrics(
                self.fixture(), sample_period=0.1,
                nominal_common_velocity=0.0)
        invalid_stamp = self.fixture()
        invalid_stamp[0]["stamp"] = float("nan")
        with self.assertRaises(ValueError):
            compute_metrics(invalid_stamp, sample_period=0.1)

    def test_task_completion_uses_samples_after_evaluation_window(self):
        rows = self.fixture()
        for index, row in enumerate(rows):
            row["evaluation_active"] = 1 <= index <= 7
            row["experiment_state_available"] = True
        result = compute_metrics(
            rows, sample_period=0.1, evaluation_target=1.1)
        self.assertAlmostEqual(result["task"]["completion_time"], 0.9)
        self.assertIsNotNone(result["task"]["completion_stamp"])


class ValidationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.converted = self.root / "converted"
        self.converted.mkdir()
        self.config = self.root / "config.yaml"
        self.config.write_text("gain: 1.0\n", encoding="utf-8")
        self.manifest_path = self.root / "manifest.yaml"
        self.rules = {
            "required_topics": ["/required"],
            "stamp_files": ["chassis_feedback.csv"],
            "sequence_rules": [{
                "file": "chassis_feedback.csv",
                "group_by": "robot_id",
                "field": "feedback_seq",
                "max_gap": 2,
                "allow_wrap": True,
            }],
            "sample_period": 0.01,
            "localization_exclusion": {
                "maximum_invalid_fraction": 0.1,
                "maximum_consecutive_invalid_seconds": 0.02,
            },
        }
        self.manifest = {
            "experiment_id": "E2a",
            "method_id": "M1_R1",
            "config_hashes": [{
                "archived_path": "config.yaml",
                "sha256": sha256_file(self.config),
            }],
            "command_authority": {
                "/agv1/chassis_command": ["/controller"],
                "/agv2/chassis_command": ["/controller"],
                "/agv3/chassis_command": ["/controller"],
            },
        }
        self._write_valid_files()

    def tearDown(self):
        self.temporary.cleanup()

    def _write_valid_files(self):
        atomic_dump_yaml(self.converted / "topic_inventory.yaml", {
            "topics": {"/required": {"messages": 3}},
        })
        write_csv(self.converted / "chassis_feedback.csv", [
            {"topic": "/feedback", "bag_stamp": 1.01,
             "header_stamp": 1.0, "robot_id": 1, "feedback_seq": 1},
            {"topic": "/feedback", "bag_stamp": 1.02,
             "header_stamp": 1.01, "robot_id": 1, "feedback_seq": 2},
            {"topic": "/feedback", "bag_stamp": 1.03,
             "header_stamp": 1.02, "robot_id": 1, "feedback_seq": 3},
        ])
        valid = {
            "topic": "/multi_agv/cooperative_state",
            "bag_stamp": 1.01,
            "header_stamp": 1.0,
            "load_pose_valid": True,
            "load_path_state_valid": True,
        }
        for index in range(1, 4):
            valid["robot_pose_valid_{}".format(index)] = True
            valid["support_pose_valid_{}".format(index)] = True
            valid["path_state_valid_{}".format(index)] = True
        write_csv(
            self.converted / "cooperative_state.csv",
            [dict(valid, header_stamp=1.0),
             dict(valid, header_stamp=1.01),
             dict(valid, header_stamp=1.02)])
        write_csv(self.converted / "controller_state.csv", [{
            "header_stamp": 1.0, "experiment_id": "E2a",
            "method_id": "M1_R1",
        }])
        atomic_dump_yaml(self.manifest_path, self.manifest)

    def _validate(self):
        return validate_converted_run(
            self.converted, self.manifest_path, self.rules)

    def test_valid_fixture_and_communication_age_disclaimer(self):
        report = self._validate()
        self.assertTrue(report["valid"], report["issues"])
        self.assertIn("not evidence", report["communication_ages"][
            "chassis_feedback.csv"]["note"])
        self.assertIn("no independent", report["communication_age_claim"])

    def test_missing_recommended_topic_warns_without_accepting_formal_stats(self):
        self.rules["recommended_topics"] = ["/multi_agv/experiment_state"]
        report = self._validate()
        self.assertTrue(report["valid"], report["issues"])
        self.assertIn(
            "MISSING_RECOMMENDED_TOPIC",
            [warning["code"] for warning in report["warnings"]])

    def test_missing_topic_fails(self):
        atomic_dump_yaml(
            self.converted / "topic_inventory.yaml", {"topics": {}})
        self.assertIn(
            "MISSING_TOPIC",
            [issue["code"] for issue in self._validate()["issues"]])

    def test_method_specific_topic_is_required(self):
        self.rules["method_required_topics"] = {
            "M1_R1": ["/multi_agv/method_internal_state"]}
        report = self._validate()
        self.assertIn(
            "MISSING_TOPIC",
            [issue["code"] for issue in report["issues"]])
        inventory = {
            "topics": {
                "/required": {"messages": 3},
                "/multi_agv/method_internal_state": {"messages": 3},
            }}
        atomic_dump_yaml(
            self.converted / "topic_inventory.yaml", inventory)
        self.assertTrue(self._validate()["valid"])

    def test_topic_rate_below_registered_minimum_fails(self):
        self.rules["minimum_topic_rates"] = {"/required": 90.0}
        atomic_dump_yaml(self.converted / "topic_inventory.yaml", {
            "topics": {
                "/required": {
                    "messages": 21,
                    "first_bag_stamp": 1.0,
                    "last_bag_stamp": 2.0,
                },
            },
        })
        report = self._validate()
        self.assertIn(
            "TOPIC_RATE_LOW",
            [issue["code"] for issue in report["issues"]])
        self.assertAlmostEqual(
            report["topic_rates"]["/required"]["observed"], 20.0)

    def test_topic_rate_above_registered_minimum_passes(self):
        self.rules["minimum_topic_rates"] = {"/required": 90.0}
        atomic_dump_yaml(self.converted / "topic_inventory.yaml", {
            "topics": {
                "/required": {
                    "messages": 101,
                    "first_bag_stamp": 1.0,
                    "last_bag_stamp": 2.0,
                },
            },
        })
        report = self._validate()
        self.assertTrue(report["valid"], report["issues"])
        self.assertAlmostEqual(
            report["topic_rates"]["/required"]["observed"], 100.0)

    def test_camera_source_requires_raw_filtered_and_confidence_evidence(self):
        cooperative = read_csv(
            self.converted / "cooperative_state.csv")
        cooperative[0]["robot_localization_source_1"] = 2
        write_csv(
            self.converted / "cooperative_state.csv", cooperative)
        self.rules["camera_required_topics"] = [
            "/camera/raw", "/camera/confidence", "/camera/filtered"]

        report = self._validate()
        self.assertFalse(report["valid"])
        self.assertEqual(
            report["camera_evidence"]["missing_topics"],
            self.rules["camera_required_topics"])
        self.assertIn(
            "CAMERA_EVIDENCE_MISSING",
            [issue["code"] for issue in report["issues"]])

        atomic_dump_yaml(self.converted / "topic_inventory.yaml", {
            "topics": {
                "/required": {"messages": 3},
                "/camera/raw": {"messages": 3},
                "/camera/confidence": {"messages": 3},
                "/camera/filtered": {"messages": 3},
            },
        })
        report = self._validate()
        self.assertTrue(report["valid"], report["issues"])
        self.assertTrue(
            report["camera_evidence"]["camera_or_fused_used"])

    def test_camera_world_frame_generation_change_invalidates_run(self):
        cooperative = read_csv(
            self.converted / "cooperative_state.csv")
        cooperative[0]["robot_localization_source_1"] = 2
        write_csv(
            self.converted / "cooperative_state.csv", cooperative)
        self.rules["camera_required_topics"] = [
            "/vision/aruco/calibration_epoch", "/camera/world/agv1_tag_pose"]
        self.rules["camera_calibration_epoch_required"] = True
        self.rules["maximum_camera_calibration_epoch_changes"] = 0
        atomic_dump_yaml(self.converted / "topic_inventory.yaml", {
            "topics": {
                "/required": {"messages": 3},
                "/vision/aruco/calibration_epoch": {"messages": 1},
                "/camera/world/agv1_tag_pose": {"messages": 2},
            },
        })
        write_csv(self.converted / "camera_calibration_epoch.csv", [{
            "topic": "/vision/aruco/calibration_epoch",
            "bag_stamp": 1.0,
            "header_stamp": 1.0,
            "epoch_unix_ns": "1780000000000000000",
        }])
        write_csv(self.converted / "camera_pose.csv", [
            {"topic": "/camera/world/agv1_tag_pose",
             "calibration_epoch_token": "17",
             "bag_stamp": 1.0, "header_stamp": 1.0},
        ])
        self.assertTrue(self._validate()["valid"])

        write_csv(self.converted / "camera_pose.csv", [
            {"topic": "/camera/world/agv1_tag_pose",
             "calibration_epoch_token": "17",
             "bag_stamp": 1.0, "header_stamp": 1.0},
            {"topic": "/camera/world/agv1_tag_pose",
             "calibration_epoch_token": "29",
             "bag_stamp": 1.1, "header_stamp": 1.1},
        ])
        report = self._validate()
        self.assertFalse(report["valid"])
        self.assertIn(
            "CAMERA_CALIBRATION_TOKEN_CHANGED",
            [issue["code"] for issue in report["issues"]])

    def test_backward_stamp_fails(self):
        rows = [
            {"topic": "/feedback", "bag_stamp": 1.0, "header_stamp": 1.0,
             "robot_id": 1, "feedback_seq": 1},
            {"topic": "/feedback", "bag_stamp": 1.1, "header_stamp": 0.9,
             "robot_id": 1, "feedback_seq": 2},
        ]
        write_csv(self.converted / "chassis_feedback.csv", rows)
        self.assertIn(
            "BACKWARD_STAMP",
            [issue["code"] for issue in self._validate()["issues"]])

    def test_sequence_gap_fails(self):
        rows = [
            {"topic": "/feedback", "bag_stamp": 1.0, "header_stamp": 1.0,
             "robot_id": 1, "feedback_seq": 1},
            {"topic": "/feedback", "bag_stamp": 1.1, "header_stamp": 1.1,
             "robot_id": 1, "feedback_seq": 9},
        ]
        write_csv(self.converted / "chassis_feedback.csv", rows)
        self.assertIn(
            "SEQUENCE_GAP",
            [issue["code"] for issue in self._validate()["issues"]])

    def test_localization_exclusion_fails(self):
        rows = []
        for index in range(3):
            row = {
                "header_stamp": 1.0 + 0.01 * index,
                "load_pose_valid": False,
                "load_path_state_valid": True,
            }
            for robot in range(1, 4):
                row["robot_pose_valid_{}".format(robot)] = True
                row["support_pose_valid_{}".format(robot)] = True
                row["path_state_valid_{}".format(robot)] = True
            rows.append(row)
        write_csv(self.converted / "cooperative_state.csv", rows)
        codes = [issue["code"] for issue in self._validate()["issues"]]
        self.assertIn("LOCALIZATION_INVALID_FRACTION", codes)
        self.assertIn("LOCALIZATION_INVALID_DURATION", codes)

    def test_localization_exclusion_uses_evaluation_window(self):
        invalid = {
            "stamp": 1.0,
            "localization_valid": False,
            "evaluation_active": False,
            "experiment_state_available": True,
        }
        valid = {
            "stamp": 1.01,
            "localization_valid": True,
            "evaluation_active": True,
            "experiment_state_available": True,
        }
        write_csv(
            self.converted / "aligned_samples.csv",
            [invalid, valid, dict(invalid, stamp=1.02)])
        write_csv(self.converted / "experiment_state.csv", [
            {"header_stamp": 0.99, "experiment_id": "E2a",
             "method_id": "M1_R1", "evaluation_active": False},
            {"header_stamp": 1.01, "experiment_id": "E2a",
             "method_id": "M1_R1", "evaluation_active": True},
            {"header_stamp": 1.02, "experiment_id": "E2a",
             "method_id": "M1_R1", "evaluation_active": False},
        ])

        report = self._validate()
        self.assertTrue(report["valid"], report["issues"])
        self.assertEqual(
            report["localization"]["scope"], "evaluation_active")
        self.assertEqual(report["localization"]["samples"], 1)
        self.assertEqual(report["localization"]["invalid_samples"], 0)
        self.assertEqual(report["localization"]["excluded_samples"], 2)

    def test_algorithm_fail_zero_in_evaluation_window_fails(self):
        self.rules["algorithm_exclusion"] = {
            "maximum_invalid_fraction": 0.0,
            "maximum_consecutive_invalid_seconds": 0.0,
        }
        write_csv(self.converted / "aligned_samples.csv", [
            {"stamp": 1.0, "algorithm_state_available": True,
             "algorithm_valid": True},
            {"stamp": 1.01, "algorithm_state_available": True,
             "algorithm_valid": False},
        ])
        report = self._validate()
        codes = [issue["code"] for issue in report["issues"]]
        self.assertIn("ALGORITHM_INVALID_FRACTION", codes)
        self.assertIn("ALGORITHM_INVALID_DURATION", codes)
        self.assertEqual(report["algorithm"]["invalid_samples"], 1)

    def test_required_task_completion_fails_when_target_is_not_reached(self):
        self.rules["require_task_completion"] = True
        manifest = copy.deepcopy(self.manifest)
        manifest["metrics"] = {"evaluation_target": 1.0}
        atomic_dump_yaml(self.manifest_path, manifest)
        write_csv(self.converted / "aligned_samples.csv", [
            {"stamp": 1.0, "load_s_actual": 0.0},
            {"stamp": 2.0, "load_s_actual": 0.99},
        ])
        report = self._validate()
        self.assertIn(
            "TASK_INCOMPLETE",
            [issue["code"] for issue in report["issues"]])

    def test_method_hash_and_authority_mismatch_fail(self):
        write_csv(self.converted / "controller_state.csv", [{
            "header_stamp": 1.0, "experiment_id": "E2a",
            "method_id": "M2a_R1",
        }])
        self.config.write_text("gain: 2.0\n", encoding="utf-8")
        manifest = copy.deepcopy(self.manifest)
        manifest["command_authority"]["/agv3/chassis_command"] = [
            "/controller", "/move_base"]
        atomic_dump_yaml(self.manifest_path, manifest)
        codes = [issue["code"] for issue in self._validate()["issues"]]
        self.assertIn("METHOD_MISMATCH", codes)
        self.assertIn("CONFIG_HASH_MISMATCH", codes)
        self.assertIn("COMMAND_AUTHORITY", codes)

    def test_mid_run_authority_change_fails(self):
        manifest = copy.deepcopy(self.manifest)
        manifest["command_authority_violations"] = [{
            "stamp": 1.5,
            "expected": manifest["command_authority"],
            "observed": {
                "/agv1/chassis_command": ["/controller", "/move_base"],
            },
        }]
        atomic_dump_yaml(self.manifest_path, manifest)
        self.assertIn(
            "COMMAND_AUTHORITY_CHANGED",
            [issue["code"] for issue in self._validate()["issues"]])

    def test_manual_abort_fails(self):
        write_csv(self.converted / "experiment_state.csv", [{
            "header_stamp": 1.0,
            "experiment_id": "E2a",
            "method_id": "M1_R1",
            "manual_abort": True,
            "abort_reason": "operator stop",
        }])
        self.assertIn(
            "MANUAL_ABORT",
            [issue["code"] for issue in self._validate()["issues"]])

    def test_experiment_identifier_mismatch_fails(self):
        write_csv(self.converted / "controller_state.csv", [{
            "header_stamp": 1.0,
            "experiment_id": "wrong_experiment",
            "method_id": "M1_R1",
        }])
        self.assertIn(
            "EXPERIMENT_MISMATCH",
            [issue["code"] for issue in self._validate()["issues"]])

    def test_empty_method_identifier_fails(self):
        write_csv(self.converted / "controller_state.csv", [{
            "header_stamp": 1.0,
            "experiment_id": "E2a",
            "method_id": "",
        }])
        self.assertIn(
            "METHOD_MISMATCH",
            [issue["code"] for issue in self._validate()["issues"]])


if __name__ == "__main__":
    unittest.main()
