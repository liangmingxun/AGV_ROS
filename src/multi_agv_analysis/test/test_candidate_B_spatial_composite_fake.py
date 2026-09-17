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
CONVERSION = ROOT / "multi_agv_analysis/src/multi_agv_analysis/conversion.py"


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
        self.assertIn("supportZoneProjection", text)
        self.assertNotIn("2.598", text)
        self.assertIn("candidate_B_spatial_composite_fake_validation", text)
        self.assertIn("!m2b_selected_", text)

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
        self.assertIn("fields_per_robot = 36U", publish)
        self.assertIn("header14+3x36;robots=1,2,3", publish)
        self.assertIn("candidate_b_spatial_projection_distance_", publish)
        self.assertNotIn("duration", CONFIG.read_text())

    def test_projection_quality_is_finite_and_bounded(self):
        text = NODE.read_text()
        projection = text.split("SupportZoneProjection supportZoneProjection", 1)[1].split(
            "void applyCandidateBSpatialComposite", 1)[0]
        self.assertIn("validateCandidateBSpatialProjectionQuality", projection)
        header = (ROOT / "multi_agv_control/include/multi_agv_control/"
                  "candidate_b_spatial_composite_disturbance.hpp").read_text()
        self.assertIn("kCandidateBSpatialMaximumProjectionDistance = 0.30", header)
        self.assertIn("std::isfinite(progress)", header)
        self.assertIn("std::isfinite(distance)", header)

    def test_three_independent_disturbance_states_and_tracks(self):
        text = NODE.read_text()
        self.assertIn("std::array<CandidateBSpatialCompositeDisturbance, 3>", text)
        self.assertIn("candidate_b_spatial_disturbance_[index].evaluate", text)
        self.assertIn("wheel_separation[index]", text)

    def test_conversion_accepts_legacy_and_projection_distance_layouts(self):
        text = CONVERSION.read_text()
        self.assertIn("header14+3x32;robots=1,2,3\": 32", text)
        self.assertIn("header14+3x33;robots=1,2,3\": 33", text)
        self.assertIn("header14+3x36;robots=1,2,3\": 36", text)
        self.assertIn('"projection_distance"', text)
        self.assertIn('"left_native_unbounded"', text)
        self.assertIn('"native_governor_scale"', text)

    def test_projection_statistics_are_reported_per_robot(self):
        rows = []
        for index, wall in enumerate((0.0, 0.01, 0.02)):
            row = {"wall": wall, "load_s_reference": 2.0 + index * .4}
            for robot in (1, 2, 3):
                prefix = f"agv{robot}_candidate_b_spatial_"
                row.update({
                    prefix + "triggered": 1.0,
                    prefix + "active": 1.0 if index == 1 else 0.0,
                    prefix + "finished": 1.0 if index == 2 else 0.0,
                    prefix + "entry_wall_time": 0.0,
                    prefix + "exit_wall_time": .02,
                    prefix + "progress": 2.0 + index * .4,
                    prefix + "d_v": .01,
                    prefix + "d_omega": .1,
                    prefix + "q": .5,
                    prefix + "rho": .925,
                    prefix + "projection_distance": .14 + robot * .01,
                })
            rows.append(row)
        result = self.module.disturbance_verification(rows)
        for robot in (1, 2, 3):
            item = result[f"Robot{robot}"]
            self.assertTrue(item["projection_distance_available"])
            self.assertAlmostEqual(item["projection_distance_mean"],
                                   .14 + robot * .01)
            self.assertAlmostEqual(item["projection_distance_max"],
                                   .14 + robot * .01)


if __name__ == "__main__":
    unittest.main()
