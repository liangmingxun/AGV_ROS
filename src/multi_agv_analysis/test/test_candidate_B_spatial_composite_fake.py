#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "multi_agv_analysis/scripts/analyze_candidate_B_spatial_composite_fake.py"
CONFIG = ROOT / "multi_agv_bringup/config/formal_fake_candidate_B_spatial_composite.yaml"
NODE = ROOT / "multi_agv_control/src/formal_fake_algorithm_node.cpp"


class CandidateBSpatialCompositeFakeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("candidate_b_v2", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_frozen_profile_and_candidate_a_disabled(self):
        runtime = yaml.safe_load(CONFIG.read_text())["formal_fake_runtime"]
        profile = runtime["candidate_b_spatial_composite"]
        self.assertEqual(profile["minimum_effectiveness"], .85)
        self.assertEqual(profile["longitudinal_amplitude"], .020)
        self.assertEqual(profile["yaw_amplitude"], .2625)
        self.assertEqual(profile["zone_start"], 2.0)
        self.assertEqual(profile["zone_end"], 2.8)
        self.assertEqual(profile["ramp_in_distance"], .05)
        self.assertEqual(profile["ramp_out_distance"], .05)
        self.assertFalse(runtime["candidate_b"]["enabled"])
        self.assertFalse(runtime["classic_additive_candidate_A"]["enabled"])

    def test_prepare_is_fake_only_and_records_new_topic(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "M1_R1"
            self.module.prepare(root, "M1")
            candidate = next(root.glob("configs/*_M1.yaml"))
            runtime = yaml.safe_load(candidate.read_text())["formal_fake_runtime"]
            self.assertEqual(runtime["experiment_id"], self.module.IDENTITY)
            self.assertTrue(runtime["candidate_b_spatial_composite"]["enabled"])
            self.assertFalse(runtime["candidate_b"]["enabled"])
            topics = yaml.safe_load((root / "configs/record_topics_v5.yaml").read_text())[
                "experiment_recording"]
            self.assertIn("/multi_agv/candidate_B_spatial_composite_state",
                          topics["required_topics"])
            self.assertNotIn("/multi_agv/candidate_B_disturbance_state",
                             topics["required_topics"])

    def test_execution_order_preserves_native_causal_demand(self):
        text = NODE.read_text()
        saved = text.index("wheel_demand_before_limit[index]", text.index(
            "auto execution_tracking = tracking;"))
        disturbed = text.index("applyCandidateBSpatialComposite(", saved)
        limited = text.index("applySerialExecutionLimitPolicy", disturbed)
        self.assertLess(saved, disturbed)
        self.assertLess(disturbed, limited)
        self.assertIn("supportZoneProgress", text)
        self.assertNotIn("2.598", text)
        self.assertIn(
            '(experiment_id_ == "candidate_B_spatial_composite_fake_validation" && !m2b_selected_)',
            text)

    def test_exact_fake_only_gate_and_no_direct_capability_path(self):
        text = NODE.read_text()
        gate = text.split("if (candidate_b_spatial_config_.enabled)", 1)[1].split(
            "if (classic_additive_config_.enabled)", 1)[0]
        self.assertIn('transport_type_ == "fake"', gate)
        self.assertIn(self.module.IDENTITY, gate)
        self.assertIn("!v5_hardware_authorized", gate)
        self.assertIn("!lower_hardware_authorized", gate)
        self.assertIn("!candidate_b_config_.enabled", gate)
        self.assertIn("!classic_additive_config_.enabled", gate)
        apply_body = text.split("void applyCandidateBSpatialComposite", 1)[1].split(
            "void publishCandidateBSpatialComposite", 1)[0]
        self.assertNotIn("capability_", apply_body)

    def test_state_layout_is_explicit_and_three_robot(self):
        text = NODE.read_text()
        publish = text.split("void publishCandidateBSpatialComposite", 1)[1].split(
            "void publishClassicAdditivePhysical", 1)[0]
        self.assertIn("header_fields = 14U", publish)
        self.assertIn("fields_per_robot = 32U", publish)
        self.assertIn("header14+3x32;robots=1,2,3", publish)
        self.assertNotIn("duration", CONFIG.read_text())


if __name__ == "__main__":
    unittest.main()
