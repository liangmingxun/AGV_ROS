#!/usr/bin/env python3

import copy
import math
import tempfile
import unittest
from pathlib import Path

from multi_agv_analysis.io_utils import (
    atomic_dump_yaml,
    sha256_file,
    write_csv,
)
from multi_agv_analysis.metrics import compute_metrics
from multi_agv_analysis.validation import validate_converted_run


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
                "agv2_mapped_velocity_upper": (
                    0.5 if index < 6 else 0.95),
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
            result["wheel"]["tracking_rmse"],
            math.sqrt(10.0 * 0.1 ** 2 / 60.0))
        self.assertAlmostEqual(result["path"]["progress_rmse"], 0.2)
        self.assertAlmostEqual(
            result["recovery"]["capability_95_time"], 0.6)
        self.assertAlmostEqual(
            result["recovery"]["common_velocity_after_capability_delay"], 0.2)
        self.assertAlmostEqual(result["task"]["completion_time"], 0.9)
        self.assertTrue(result["causal_unfiltered"])
        self.assertEqual(
            result["evaluation_window"]["source"],
            "all_samples_fallback")
        self.assertFalse(
            result["evaluation_window"]["formal_statistics_ready"])

    def test_experiment_state_window_is_reported(self):
        rows = self.fixture()
        for index, row in enumerate(rows):
            row["experiment_state_available"] = True
            row["evaluation_active"] = index >= 2
        result = compute_metrics(rows, sample_period=0.1)
        self.assertEqual(result["sample_counts"]["aligned"], 8)
        self.assertEqual(result["sample_counts"]["experiment_state"], 10)
        self.assertEqual(
            result["evaluation_window"]["source"], "experiment_state")
        self.assertTrue(
            result["evaluation_window"]["formal_statistics_ready"])

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
