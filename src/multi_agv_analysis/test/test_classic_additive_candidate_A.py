#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / \
    "analyze_classic_additive_candidate_A.py"
spec = importlib.util.spec_from_file_location("classic", SCRIPT)
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


class ClassicAdditiveAnalysisTest(unittest.TestCase):
    def test_fixed_profile(self):
        self.assertEqual(C.classic_expected(-1), (0., 0., 0.))
        self.assertAlmostEqual(C.classic_expected(.25)[0], .5)
        self.assertAlmostEqual(C.classic_expected(1)[1], .03*__import__('math').sin(1))
        self.assertAlmostEqual(C.classic_expected(7.75)[0], .5)
        self.assertEqual(C.classic_expected(8), (0., 0., 0.))

    def test_prepare_is_fake_only_and_keeps_frozen_m2b(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "M1"
            C.prepare(root, "M1")
            candidate = next(iter(C.H.CANDIDATES))
            config = yaml.safe_load((root/"configs"/f"{candidate}_M1.yaml").read_text())
            self.assertEqual(config["formal_fake_runtime"]["experiment_id"], C.IDENTITY)
            self.assertEqual(config["formal_fake_runtime"]["classic_additive_candidate_A"], C.PROFILE)
            self.assertFalse(config["formal_fake_runtime"]["yaw_effectiveness_hold_v5b"]["enabled"])
            frozen = yaml.safe_load((C.CONFIG/"exp2b_M2b.yaml").read_text())
            generated = yaml.safe_load((root/"configs"/f"{candidate}_M2b.yaml").read_text())
            self.assertEqual(frozen, generated)

    def test_only_three_frozen_orders(self):
        self.assertEqual(C.ORDERS, [["M1b", "M1", "M2b"],
                                   ["M2b", "M1b", "M1"],
                                   ["M1", "M2b", "M1b"]])

    def test_shared_injection_routing_and_regression_guards(self):
        workspace = SCRIPT.parents[3]
        source = (workspace/"src/multi_agv_control/src/formal_fake_algorithm_node.cpp").read_text()
        self.assertEqual(source.count(
            "applyClassicAdditive(now, tracking, &execution_tracking);"), 2)
        self.assertIn("m2b_controller_->step", source)
        for block in source.split(
                "applyClassicAdditive(now, tracking, &execution_tracking);")[1:]:
            self.assertLess(block.find("trackingPassesSerialEmergencyGate"),
                            block.find("publishExecutionLimiter"))
        apply_body = source.split("void applyClassicAdditive", 1)[1].split(
            "void publishExecutionLimiter", 1)[0]
        self.assertNotIn("robust_margin", apply_body)
        self.assertNotIn("capability", apply_body.lower())
        self.assertIn("exp2c_v5b_s1_three_method_repeat_validation", source)
        self.assertIn("exp2c_v4_transient_yaw_recovery", source)
        # The physical qualification ceiling is a serial-only guard; the
        # frozen fake Candidate A remains an exact scale=1.0 validation.
        self.assertIn(
            "validateClassicAdditivePhysicalQualification(", source)
        self.assertIn(
            "if (classic_additive_physical_enabled_)", source)


if __name__ == "__main__":
    unittest.main()
