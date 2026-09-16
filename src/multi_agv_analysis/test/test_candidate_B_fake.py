#!/usr/bin/env python3
import importlib.util
import pathlib
import tempfile
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "multi_agv_analysis/scripts/analyze_candidate_B_fake.py"
CONFIG = ROOT / "multi_agv_bringup/config/formal_fake_candidate_B_effectiveness.yaml"
SOURCE = ROOT / "multi_agv_control/src/formal_fake_algorithm_node.cpp"


def load_module():
    spec = importlib.util.spec_from_file_location("candidate_b_analysis", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CandidateBFakeTest(unittest.TestCase):
    def test_frozen_profile_and_waveform(self):
        module = load_module()
        profile = yaml.safe_load(CONFIG.read_text())["formal_fake_runtime"]["candidate_b"]
        self.assertEqual(profile, module.PROFILE)
        self.assertEqual(profile["minimum_effectiveness"], .85)
        self.assertEqual(profile["yaw_amplitude"], .2625)
        self.assertEqual(module.expected(-1.), (0., 1., 0.))
        self.assertAlmostEqual(module.expected(.25)[0], .5)
        self.assertEqual(module.expected(4.)[1], .85)
        self.assertEqual(module.expected(8.), (0., 1., 0.))

    def test_prepare_is_fake_only_and_disables_candidate_a(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            module.prepare(root, "M2b")
            for path in (root / "configs").glob("*.yaml"):
                data = yaml.safe_load(path.read_text())
                self._audit_hardware_false(data, path)
            runtime = yaml.safe_load((root / "configs/M2b_runtime.yaml").read_text())[
                "formal_fake_runtime"]
            self.assertTrue(runtime["candidate_b"]["enabled"])
            self.assertFalse(runtime["classic_additive_candidate_A"]["enabled"])
            self.assertFalse(runtime["serial_execution_authorized"])
            topics = yaml.safe_load((root / "configs/record_topics_v5.yaml").read_text())[
                "experiment_recording"]
            self.assertIn("/multi_agv/candidate_B_disturbance_state", topics["topics"])
            self.assertIn("/multi_agv/risk_disturbance_state", topics["topics"])
            self.assertNotIn("/multi_agv/yaw_effectiveness_hold_state", topics["topics"])

    def _audit_hardware_false(self, value, path):
        if isinstance(value, dict):
            for key, item in value.items():
                if key.endswith("hardware_execution_authorized"):
                    self.assertIs(item, False, "{} in {}".format(key, path))
                self._audit_hardware_false(item, path)
        elif isinstance(value, list):
            for item in value:
                self._audit_hardware_false(item, path)

    def test_native_demand_is_saved_before_candidate_b(self):
        text = SOURCE.read_text()
        saved = text.index("wheel_demand_before_limit[i] = {{")
        disturbed = text.index("applyCandidateB(tracking, &execution_tracking)", saved)
        previous = text.index("previous_wheel_raw_ = wheel_demand_before_limit", disturbed)
        self.assertLess(saved, disturbed)
        self.assertLess(disturbed, previous)
        self.assertIn("v.longitudinal_after_effectiveness", text)
        self.assertIn("v.yaw_native", text)


if __name__ == "__main__":
    unittest.main()
