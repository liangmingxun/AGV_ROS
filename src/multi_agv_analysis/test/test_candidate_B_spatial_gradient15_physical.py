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
        self.assertEqual(self.module.METHODS["M2c"], "M2c_M2c")
        self.assertEqual(self.module.M2C_IDENTITY,
            "candidate_B_spatial_gradient15_m2c_physical_validation")

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

    def test_formal_evidence_uses_runtime_overlay_not_worktree_cleanliness(self):
        classify = self.module.classify_physical_evidence
        self.assertEqual(classify(True, "runtime_operator_overlay"),
                         ("VALID_COMPLETED", True))
        self.assertEqual(classify(True, "tracked_config"),
                         ("VALID_COMPLETED_NONFORMAL_PROVENANCE", False))
        self.assertEqual(classify(False, "runtime_operator_overlay"),
                         ("INVALID_RUN", True))

    def test_physical_analysis_uses_serial_algorithm_namespace(self):
        serial = {"formal_algorithm": {
            "formal_fake_runtime": {"experiment_id": self.module.IDENTITY},
            "formal_upper": {"hardware_execution_authorized": True},
            "formal_lower": {"hardware_execution_authorized": True},
        }}
        selected = self.module.physical_algorithm_params(serial)
        self.assertEqual(selected, serial["formal_algorithm"])
        with self.assertRaisesRegex(
                ValueError, "missing the formal_algorithm namespace"):
            self.module.physical_algorithm_params({
                "formal_fake_algorithm": serial["formal_algorithm"]})

    def test_historical_bag_nominal_upper_reconstruction(self):
        params = {"formal_upper": {"agents": {
            "nominal_upper": [.115, .115, .115],
            "inner_margin": [.005, .005, .005],
        }}}
        self.assertAlmostEqual(
            self.module.nominal_common_inner_upper(params), .110)

    def test_historical_nan_contraction_is_reconstructed(self):
        rows = []
        for wall in (0.0, 0.01):
            row = {
                "wall": wall,
                "risk_contraction": "nan",
                "common_boundary_upper": ".090",
                "candidate_common_velocity": ".100",
                "upper_effective_common_velocity": ".090",
                "common_velocity_reference": ".090",
            }
            for robot in (1, 2, 3):
                row[f"agv{robot}_robust_margin"] = ".5"
                row[f"agv{robot}_risk_factor"] = ".5"
                row[f"agv{robot}_risk_signal"] = ".475"
            rows.append(row)
        result = self.module.BASE.mechanism_metrics(rows, .110)
        self.assertAlmostEqual(result["risk_contraction_peak"], .020)
        self.assertAlmostEqual(result["risk_boundary_active_duration"], .020)
        self.assertAlmostEqual(result["risk_boundary_active_fraction"], 1.0)

    def test_execution_state_reconstruction_distinguishes_projection_policy(self):
        rows = []
        for index in range(6):
            row = {
                "wall": index * .1,
                "agv2_channel_input_limited": "1.0",
                "common_velocity_reference": ".10",
                "agv2_mapped_capability_diagnostic": ".12",
                "agv2_candidate_b_spatial_active": "1.0",
                "agv2_candidate_b_spatial_finished": "1.0" if index == 5 else "0.0",
                "agv2_candidate_b_spatial_triggered": "1.0",
                "agv2_candidate_b_spatial_entry_wall_time": "0.0",
                "agv2_candidate_b_spatial_exit_wall_time": ".5",
                "agv2_candidate_b_spatial_left_native": ".1",
                "agv2_candidate_b_spatial_right_native": ".1",
                "agv2_candidate_b_spatial_actual_left": ".1",
                "agv2_candidate_b_spatial_actual_right": ".1",
                "agv2_wheel_left_actual": ".1",
                "agv2_wheel_right_actual": ".1",
                "agv2_s_tracking_actual": str(index * .01),
                "load_s_reference": str(index * .01),
                "agv2_candidate_b_spatial_q": "1.0",
                "agv2_candidate_b_spatial_rho": ".8275",
                "agv2_candidate_b_spatial_d_v": ".023",
                "agv2_candidate_b_spatial_d_omega": ".301875",
                "agv2_candidate_b_spatial_projection_distance": "0.0",
            }
            rows.append(row)
        params = {"formal_fake_runtime": {"execution": {
            "startup_ramp_seconds": .01,
            "startup_catchup_margin_mps": .008,
            "startup_early_rise": .6,
        }}}
        m1 = self.module.reconstruct_robot2_execution_state(rows, "M1", params)
        m2b = self.module.reconstruct_robot2_execution_state(rows, "M2b", params)
        self.assertAlmostEqual(m1["active_state_peak_mps"], .12)
        self.assertGreater(m1["active_projection_duration_seconds"], 0.0)
        self.assertGreater(m2b["active_state_peak_mps"], .12)
        self.assertEqual(m2b["active_projection_duration_seconds"], 0.0)
        self.assertGreater(m1["counterfactual_active_state_peak_mps"],
                           m1["active_state_peak_mps"])
        self.assertAlmostEqual(m2b["counterfactual_active_state_peak_mps"],
                               .1205)


if __name__ == "__main__":
    unittest.main()
