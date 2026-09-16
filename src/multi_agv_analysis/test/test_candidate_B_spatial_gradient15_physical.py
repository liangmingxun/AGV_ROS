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
        source = SCRIPT.read_text()
        self.assertIn('physical.get("software_qualification_passed") is not True',
                      source)
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

    def test_hard_failure_uses_runtime_events_not_invalid_fraction(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "physical_runtime.log"
            log.write_text(
                "[INFO] [100.0]: running\n"
                "[WARN] [101.0]: Formal algorithm held fail-zero: safety\n")
            self.assertTrue(self.module.BASE.hard_failure_in_valid_window(
                log, 100.0, 102.0))
            self.assertFalse(self.module.BASE.hard_failure_in_valid_window(
                log, 102.0, 103.0))
            log.write_text("[ERROR] [101.0]: emergency abort latched\n")
            self.assertTrue(self.module.BASE.hard_failure_in_valid_window(
                log, 100.0, 102.0, ("emergency abort", "abort latched")))

    def test_formal_evidence_requires_clean_runtime_overlay_provenance(self):
        classify = self.module.classify_physical_evidence
        self.assertEqual(classify(True, False, "runtime_operator_overlay"),
                         ("VALID_COMPLETED", True))
        self.assertEqual(classify(True, True, "runtime_operator_overlay"),
                         ("VALID_COMPLETED_NONFORMAL_PROVENANCE", True))
        self.assertEqual(classify(True, False, "tracked_config"),
                         ("VALID_COMPLETED_NONFORMAL_PROVENANCE", False))
        self.assertEqual(classify(False, False, "runtime_operator_overlay"),
                         ("INVALID_RUN", True))


if __name__ == "__main__":
    unittest.main()
