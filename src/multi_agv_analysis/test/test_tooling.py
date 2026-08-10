#!/usr/bin/env python3

import math
import tempfile
import unittest
from pathlib import Path

from multi_agv_analysis.calibration import (
    ENTITIES,
    calibration_candidate,
    compose,
    solve_tag_to_target,
    solve_world_alignment,
)
from multi_agv_analysis.camera_quality import summarize_camera_quality
from multi_agv_analysis.evidence import (
    STAGES,
    approval_candidate,
    attach_evidence,
    audit_registry,
    create_registry,
    mark_passed,
    sign_stage,
)
from multi_agv_analysis.experiment_design import (
    create_paired_plan,
    next_pending,
    render_run_instructions,
    update_run,
)
from multi_agv_analysis.health import audit_ros_graph, validate_inventory
from multi_agv_analysis.io_utils import atomic_dump_json
from multi_agv_analysis.monitoring import SoftwareMonitor
from multi_agv_analysis.reporting import aggregate_plan


class CalibrationTest(unittest.TestCase):
    def test_se2_solvers_generate_non_authorizing_candidate(self):
        fixed = {
            "agv1": (0.10, -0.02, 0.03),
            "agv2": (-0.04, 0.05, -0.02),
            "agv3": (0.02, 0.01, 0.04),
            "load": (0.0, -0.08, 0.01),
        }
        rows = []
        for entity in ENTITIES:
            for index in range(4):
                tag = (0.2 * index, -0.1 * index, 0.15 * index)
                target = compose(tag, fixed[entity])
                rows.append({
                    "entity": entity,
                    "tag_x": tag[0], "tag_y": tag[1],
                    "tag_yaw": tag[2],
                    "target_x": target[0], "target_y": target[1],
                    "target_yaw": target[2],
                })
        policy = {
            "minimum_samples_per_entity": 4,
            "minimum_world_samples": 4,
            "maximum_translation_residual": 1.0e-8,
            "maximum_yaw_residual": 1.0e-8,
            "maximum_world_point_residual": 1.0e-8,
        }
        tag_result = solve_tag_to_target(rows, policy)
        self.assertTrue(tag_result["passed"])
        yaw = 0.25
        translation = (0.4, -0.2)
        world_rows = []
        for x, y in ((0, 0), (1, 0), (0, 1), (1, 1)):
            world_rows.append({
                "source_x": x, "source_y": y,
                "world_x": (
                    math.cos(yaw) * x - math.sin(yaw) * y +
                    translation[0]),
                "world_y": (
                    math.sin(yaw) * x + math.cos(yaw) * y +
                    translation[1]),
            })
        world = solve_world_alignment(world_rows, policy)
        self.assertTrue(world["passed"])
        candidate = calibration_candidate(tag_result, world, {"x": "y"})
        self.assertTrue(candidate["numeric_policy_passed"])
        self.assertFalse(candidate["calibration_authorized"])
        self.assertFalse(candidate["extrinsics_frozen"])

    def test_missing_physical_policy_is_refused(self):
        with self.assertRaises(ValueError):
            solve_tag_to_target([], {})

    def test_three_agv_calibration_leaves_load_explicitly_unresolved(self):
        rows = []
        for entity in ("agv1", "agv2", "agv3"):
            for index in range(2):
                rows.append({
                    "entity": entity,
                    "tag_x": float(index), "tag_y": 0.0, "tag_yaw": 0.0,
                    "target_x": float(index) + 0.02,
                    "target_y": 0.0, "target_yaw": 0.0,
                })
        result = solve_tag_to_target(rows, {
            "required_entities": ["agv1", "agv2", "agv3"],
            "minimum_samples_per_entity": 2,
            "maximum_translation_residual": 1.0e-8,
            "maximum_yaw_residual": 1.0e-8,
        })
        self.assertTrue(result["passed"])
        self.assertEqual(result["unresolved_entities"], ["load"])


class CameraQualityTest(unittest.TestCase):
    def test_report_is_descriptive_and_detects_timing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            from multi_agv_analysis.io_utils import write_csv
            write_csv(root / "camera_pose.csv", [
                {"topic": "/camera/world/agv1_tag_pose",
                 "bag_stamp": 1.01, "header_stamp": 1.0,
                 "calibration_epoch_token": 17},
                {"topic": "/camera/world/agv1_tag_pose",
                 "bag_stamp": 1.11, "header_stamp": 1.1,
                 "calibration_epoch_token": 17},
                {"topic": "/pose_provider/agv1/base_pose_raw",
                 "bag_stamp": 1.01, "header_stamp": 1.0},
                {"topic": "/pose_provider/agv1/base_pose_filtered",
                 "bag_stamp": 1.01, "header_stamp": 1.0},
            ])
            write_csv(root / "camera_confidence.csv", [
                {"topic": "/camera/world/agv1_confidence",
                 "bag_stamp": 1.01, "header_stamp": 1.01,
                 "confidence": 0.9},
            ])
            write_csv(root / "camera_calibration_epoch.csv", [{
                "topic": "/vision/aruco/calibration_epoch",
                "bag_stamp": 1.0, "header_stamp": 1.0,
                "epoch_unix_ns": "1780000000000000000",
            }])
            report = summarize_camera_quality(root)
            self.assertFalse(report["quality_authorized"])
            stream = report["streams"][
                "/camera/world/agv1_tag_pose"]
            self.assertAlmostEqual(stream["observed_rate"], 10.0)
            self.assertAlmostEqual(stream["arrival_delay_mean"], 0.01)
            self.assertEqual(
                report["filter_acceptance"]["agv1"]
                ["filtered_to_raw_ratio"], 1.0)
            self.assertTrue(
                report["calibration_epoch"]["continuous"])


class ExperimentDesignTest(unittest.TestCase):
    def test_plan_is_paired_and_state_machine_is_strict(self):
        spec = {
            "experiment_id": "exp2a",
            "methods": ["M1_R1", "M2a_R1"],
            "paired_repetitions": 3,
            "random_seed": 7,
        }
        plan = create_paired_plan(spec)
        self.assertEqual(len(plan["runs"]), 6)
        for pair in range(1, 4):
            values = [run["method_id"] for run in plan["runs"]
                      if run["pair_index"] == pair]
            self.assertEqual(set(values), set(spec["methods"]))
        run = next_pending(plan)
        update_run(plan, run["run_id"], "running")
        commands = render_run_instructions(run, "rehearsal")
        self.assertEqual(len(commands), 2)
        self.assertTrue(all("roslaunch" in value for value in commands))
        with self.assertRaises(ValueError):
            update_run(plan, run["run_id"], "complete")


class ReportingTest(unittest.TestCase):
    def test_completed_pairs_are_aggregated(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            plan = create_paired_plan({
                "experiment_id": "exp",
                "methods": ["A", "B"],
                "paired_repetitions": 2,
                "random_seed": 1,
            })
            values = {"A": [1.0, 2.0], "B": [2.0, 4.0]}
            indices = {"A": 0, "B": 0}
            for run in plan["runs"]:
                value = values[run["method_id"]][indices[run["method_id"]]]
                indices[run["method_id"]] += 1
                path = root / (run["run_id"] + ".json")
                manifest = root / (run["run_id"] + "_manifest.yaml")
                atomic_dump_json(path, {
                    "run_id": run["run_id"],
                    "method_id": run["method_id"],
                    "evaluation_window": {
                        "formal_statistics_ready": True},
                    "path": {"progress_rmse": value},
                })
                atomic_dump_json(manifest, {
                    "formal_statistics_requested": True,
                    "configuration_approval": {
                        "formal_statistics_authorized": True},
                })
                run["status"] = "complete"
                run["manifest"] = str(manifest)
                run["metrics"] = str(path)
            report = aggregate_plan(
                plan, ["path.progress_rmse"], "A", True, seed=3)
            self.assertEqual(report["complete_paired_blocks"], 2)
            comparison = report["comparisons"][0]
            self.assertAlmostEqual(
                comparison["difference"]["mean"], 1.5)


class EvidenceTest(unittest.TestCase):
    def test_ordered_evidence_never_authorizes(self):
        with tempfile.TemporaryDirectory() as temporary:
            evidence = Path(temporary) / "evidence.txt"
            evidence.write_text("signed external artifact", encoding="utf-8")
            config = Path(temporary) / "method.yaml"
            config.write_text("mode: M1\n", encoding="utf-8")
            registry = create_registry("project")
            for stage in STAGES:
                attach_evidence(registry, stage, evidence, "artifact")
                sign_stage(registry, stage, "operator", "operator")
                sign_stage(registry, stage, "reviewer", "reviewer")
                mark_passed(registry, stage)
            audit = audit_registry(registry)
            self.assertTrue(audit["valid"])
            self.assertTrue(audit["complete"])
            candidate = approval_candidate(
                registry, "candidate", [config])
            self.assertFalse(candidate["formal_statistics_authorized"])
            self.assertFalse(candidate["hardware_execution_authorized"])

    def test_pending_registry_is_valid_but_not_complete(self):
        audit = audit_registry(create_registry("project"))
        self.assertTrue(audit["valid"])
        self.assertFalse(audit["complete"])
        self.assertFalse(audit["candidate_eligible"])


class HealthAndMonitoringTest(unittest.TestCase):
    def test_inventory_graph_and_monitor(self):
        inventory = {
            "robot_count": 3,
            "ros_master_role": "robot1",
            "hosts": {
                "robot1": {"ip": "192.168.6.101"},
                "robot2": {"ip": "192.168.6.102"},
                "robot3": {"ip": "192.168.6.103"},
                "windows_camera": {"ip": "192.168.6.100"},
            },
        }
        self.assertTrue(validate_inventory(inventory)["valid"])
        publishers = []
        subscribers = []
        for index in range(1, 4):
            for suffix in (
                    "chassis_feedback", "capability_report", "odom", "imu"):
                publishers.append((
                    "/agv{}/{}".format(index, suffix), ["/source"]))
            publishers.append((
                "/agv{}/chassis_command".format(index), ["/controller"]))
            subscribers.append((
                "/agv{}/chassis_command".format(index), ["/chassis"]))
        publishers.append(("/multi_agv/cooperative_state", ["/estimator"]))
        self.assertTrue(audit_ros_graph(
            publishers, subscribers, True)["valid"])

        monitor = SoftwareMonitor([{
            "topic": "/state", "maximum_age": 0.2,
            "minimum_publishers": 1, "maximum_publishers": 1,
            "critical": True,
        }], startup_grace=0.1)
        self.assertTrue(monitor.evaluate(
            0.0, {"/state": ["/source"]})["healthy"])
        monitor.receive("/state", 0.05)
        self.assertTrue(monitor.evaluate(
            0.2, {"/state": ["/source"]})["healthy"])
        self.assertFalse(monitor.evaluate(
            0.4, {"/state": ["/source"]})["healthy"])
        self.assertFalse(monitor.evaluate(
            0.41, {"/state": ["/a", "/b"]})["healthy"])


if __name__ == "__main__":
    unittest.main()
