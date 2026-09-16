#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "multi_agv_analysis/scripts/analyze_candidate_B_spatial_gradient15_physical.py"


class CandidateBSpatialGradient15PhysicalAnalysisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("physical", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_exact_identity_profile_and_output_contract(self):
        self.assertEqual(self.module.IDENTITY,
            "candidate_B_spatial_gradient15_physical_validation")
        self.assertEqual(self.module.PROFILE["severity_scale"],
                         [1.0, 1.15, .85])
        self.assertEqual(self.module.PROFILE["minimum_effectiveness"], .85)
        self.assertEqual(self.module.PROFILE["longitudinal_amplitude"], .020)
        self.assertEqual(self.module.PROFILE["yaw_amplitude"], .2625)
        self.assertEqual(self.module.BASE.SPATIAL_DOMAIN_GRID_SAMPLES, 5001)
        self.assertEqual(self.module.SUMMARY_FILE,
                         "candidate_b_spatial_gradient15_physical_summary.json")
        self.assertEqual(self.module.SUMMARY_CSV,
                         "candidate_b_spatial_gradient15_physical_summary.csv")

    def test_reuses_frozen_spatial_rmse_definition(self):
        runs = {
            method: [{"load_s_reference": str(value), "errors": {
                "support": scale, "rigid_fit": 2 * scale,
                "pairwise_side": 3 * scale}}
                for value in (2.0, 2.4, 2.8)]
            for method, scale in (("M1", 1.0), ("M1b", 2.0),
                                  ("M2b", 4.0))}
        result = self.module.BASE.spatial_domain_metrics(runs, 5001)
        self.assertEqual(result["coordinate"], "load_s_reference")
        self.assertEqual(result["grid_samples"], 5001)
        self.assertAlmostEqual(result["methods"]["M1"][
            "support_error_spatial_rmse"], 1.0)
        self.assertAlmostEqual(result["comparisons"]["M1_vs_M1b"][
            "support_error_spatial_rmse"]["improvement_percent"], 50.0)

    def test_six_plot_products_are_declared(self):
        text = SCRIPT.read_text()
        for name in (
            "figure1_trajectories_and_spatial_zone",
            "figure2_support_error",
            "figure3_rigid_fit_and_pairwise",
            "figure4_equivalent_load_errors",
            "figure5_mechanism_and_execution_zoom",
            "figure6_summary_table",
        ):
            self.assertIn(name, text)


if __name__ == "__main__":
    unittest.main()
