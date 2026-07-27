#!/usr/bin/env python3

import shutil
import time
import unittest
from pathlib import Path

import rosbag
import rosnode
import rospkg
import rospy
import rostest
import yaml

from multi_agv_analysis.conversion import convert_bag
from multi_agv_analysis.io_utils import load_yaml, read_csv
from multi_agv_analysis.metrics import compute_metrics
from multi_agv_analysis.validation import validate_converted_run


class ExperimentRecordingTest(unittest.TestCase):
    def test_manifest_authority_hashes_and_bag(self):
        if not rospy.core.is_initialized():
            self.skipTest("rostest-only end-to-end case")
        run_dir = Path(rospy.get_param("~run_dir"))
        manifest_path = run_dir / "manifest.yaml"
        deadline = time.monotonic() + 25.0
        manifest = {}
        while time.monotonic() < deadline:
            if manifest_path.exists():
                with open(manifest_path, encoding="utf-8") as stream:
                    manifest = yaml.safe_load(stream)
                if manifest.get("recording_armed_at"):
                    break
            rospy.sleep(0.1)
        self.assertTrue(manifest_path.exists())
        self.assertTrue(
            manifest.get("recording_armed_at"),
            "experiment recorder never reached its subscription-ready state")
        self.assertEqual(manifest["interface_version"],
                         "agv_ros_interfaces_v1")
        self.assertEqual(manifest["method_id"], "M1_R1")
        self.assertEqual(len(manifest["git_sha"]), 40)
        for index in range(1, 4):
            topic = "/agv{}/chassis_command".format(index)
            self.assertEqual(manifest["command_authority"][topic],
                             ["/formal_fake_algorithm"])
        self.assertTrue(manifest["config_hashes"])
        for entry in manifest["config_hashes"]:
            self.assertTrue((run_dir / entry["archived_path"]).is_file())

        # Subscription readiness guarantees that commands published after the
        # armed transition cannot be missed. Keep the fake graph alive briefly
        # so every required periodic stream contributes at least one message.
        rospy.sleep(0.5)
        rosnode.kill_nodes(["/experiment_recorder"])
        bag_path = run_dir / manifest["bag"]
        deadline = time.monotonic() + 15.0
        while (not bag_path.exists() or
               yaml.safe_load(manifest_path.read_text(
                   encoding="utf-8")).get("finished_at") is None):
            if time.monotonic() >= deadline:
                self.fail("experiment recorder did not close the bag")
            rospy.sleep(0.1)

        with rosbag.Bag(str(bag_path), "r") as bag:
            topics = set(bag.get_type_and_topic_info().topics)
        self.assertTrue(set(manifest["required_topics"]).issubset(topics))

        converted = run_dir / "converted"
        conversion = convert_bag(bag_path, converted)
        self.assertGreater(conversion["aligned_samples"], 0)
        rules_path = (
            Path(rospkg.RosPack().get_path("multi_agv_analysis")) /
            "config" / "validation_defaults.yaml")
        rules = load_yaml(rules_path)["experiment_recording"]
        validation = validate_converted_run(
            converted, manifest_path, rules)
        self.assertTrue(validation["valid"], validation["issues"])
        metrics = compute_metrics(
            read_csv(converted / "aligned_samples.csv"),
            sample_period=rules["sample_period"],
            nominal_common_velocity=0.08,
            evaluation_target=1.0)
        self.assertGreater(metrics["sample_counts"]["aligned"], 0)
        self.assertTrue(metrics["causal_unfiltered"])
        shutil.rmtree(run_dir)


if __name__ == "__main__":
    rospy.init_node("experiment_recording_test")
    rostest.rosrun(
        "multi_agv_bringup", "experiment_recording_test",
        ExperimentRecordingTest)
