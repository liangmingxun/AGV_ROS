#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "multi_agv_analysis/scripts/analyze_candidate_B_spatial_gradient15_fake.py"
CONFIG = ROOT / "multi_agv_bringup/config/formal_fake_candidate_B_spatial_gradient15.yaml"
NODE = ROOT / "multi_agv_control/src/formal_fake_algorithm_node.cpp"


class CandidateBSpatialGradient15FakeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("gradient15", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)
        cls.base = cls.module.BASE

    def test_exact_frozen_gradient_and_mean(self):
        profile = yaml.safe_load(CONFIG.read_text())[
            "formal_fake_runtime"]["candidate_b_spatial_composite"]
        self.assertEqual(profile["severity_scale"], [1.0, 1.15, .85])
        self.assertAlmostEqual(sum(profile["severity_scale"]) / 3.0, 1.0)
        expected = [(.85, .020, .2625),
                    (.8275, .023, .301875),
                    (.8725, .017, .223125)]
        for scale, values in zip(profile["severity_scale"], expected):
            actual = (1.0 - .15 * scale, .020 * scale, .2625 * scale)
            for lhs, rhs in zip(actual, values):
                self.assertAlmostEqual(lhs, rhs)

    def test_prepare_uses_independent_fake_only_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "M1_R1"
            self.base.prepare(root, "M1")
            runtime = yaml.safe_load(next(root.glob("configs/*_M1.yaml")).read_text())[
                "formal_fake_runtime"]
            self.assertEqual(runtime["experiment_id"], self.base.IDENTITY)
            self.assertEqual(runtime["candidate_b_spatial_composite"],
                             self.base.PROFILE)
            self.assertFalse(runtime["candidate_b"]["enabled"])
            authorization = yaml.safe_load(next(
                root.glob("configs/*_M1_authorization.yaml")).read_text())
            self.assertFalse(authorization["formal_upper"][
                "hardware_execution_authorized"])
            self.assertFalse(authorization["formal_lower"][
                "hardware_execution_authorized"])

    def test_gradient_identity_is_exactly_fake_scoped(self):
        text = NODE.read_text()
        gate = text.split("if (candidate_b_spatial_config_.enabled)", 1)[1].split(
            "if (classic_additive_config_.enabled)", 1)[0]
        self.assertIn(self.base.IDENTITY, gate)
        self.assertIn('transport_type_ == "fake"', gate)
        self.assertIn("!v5_hardware_authorized", gate)
        self.assertIn("!lower_hardware_authorized", gate)

    def test_spatial_rmse_uses_common_progress_not_sample_count(self):
        runs = {
            "M1": [
                {"load_s_reference": str(s), "errors": {
                    "support": 1.0, "rigid_fit": 2.0,
                    "pairwise_side": 3.0}}
                for s in (0.0, .25, .5, .75, 1.0)],
            "M1b": [
                {"load_s_reference": str(s), "errors": {
                    "support": 2.0, "rigid_fit": 4.0,
                    "pairwise_side": 6.0}}
                for s in (0.0, .5, 1.0)],
            "M2b": [
                {"load_s_reference": str(s), "errors": {
                    "support": 4.0, "rigid_fit": 8.0,
                    "pairwise_side": 12.0}}
                for s in (0.0, 1.0)],
        }
        result = self.base.spatial_domain_metrics(runs, sample_count=101)
        self.assertEqual(result["common_interval_start_m"], 0.0)
        self.assertEqual(result["common_interval_end_m"], 1.0)
        self.assertAlmostEqual(result["methods"]["M1"][
            "support_error_spatial_rmse"], 1.0)
        self.assertAlmostEqual(result["methods"]["M1b"][
            "rigid_fit_error_spatial_rmse"], 4.0)
        self.assertAlmostEqual(result["methods"]["M2b"][
            "pairwise_side_error_spatial_rmse"], 12.0)
        self.assertAlmostEqual(result["comparisons"]["M1_vs_M1b"][
            "support_error_spatial_rmse"]["improvement_percent"], 50.0)


if __name__ == "__main__":
    unittest.main()
