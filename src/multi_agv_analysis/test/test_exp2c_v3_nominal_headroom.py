#!/usr/bin/env python3
import csv
import importlib.util
import pathlib
import tempfile
import unittest
import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "analyze_exp2c_v3_nominal_headroom.py"
SPEC = importlib.util.spec_from_file_location("exp2c_v3", str(SCRIPT))
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


class Exp2cV3NominalHeadroomTest(unittest.TestCase):
    def test_metadata_requires_fake_runtime_and_no_hardware_grant(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            (root / "manifest.yaml").write_text(yaml.safe_dump({
                "metrics": {"nominal_common_velocity": .10}}))
            params = {robot: {"chassis_controller": {"transport_type": "fake"}}
                      for robot in ("agv1", "agv2", "agv3")}
            params["formal_fake_algorithm"] = {"formal_fake_runtime": {
                "experiment_id": "exp2c_v3_nominal_headroom_fake",
                "leader": {"velocity": .10}, "hardware_execution_authorized": False}}
            (root / "rosparams.yaml").write_text(yaml.safe_dump(params))
            self.assertEqual(V3.manifest_nominal_velocity(root), .10)
            runtime = params["formal_fake_algorithm"]["formal_fake_runtime"]
            runtime["hardware_execution_authorized"] = True
            (root / "rosparams.yaml").write_text(yaml.safe_dump(params))
            with self.assertRaises(ValueError):
                V3.manifest_nominal_velocity(root)
            runtime["hardware_execution_authorized"] = False
            runtime["leader"] = {}
            (root / "rosparams.yaml").write_text(yaml.safe_dump(params))
            with self.assertRaises(ValueError):
                V3.manifest_nominal_velocity(root)

    def test_v3_changes_only_nominal_headroom_in_upper_control(self):
        config = PACKAGE.parent / "multi_agv_bringup" / "config"
        for method in ("M1", "M1b"):
            old = yaml.safe_load((config / (
                "exp2c_v2_{}_yaw_disturbance.yaml".format(method))).read_text())
            new = yaml.safe_load((config / (
                "exp2c_v3_{}_nominal_headroom.yaml".format(method))).read_text())
            old_upper, new_upper = old["formal_upper"], new["formal_upper"]
            for key in ("initial_upper", "nominal_upper"):
                self.assertEqual(old_upper["agents"][key], [.115] * 3)
                self.assertEqual(new_upper["agents"][key], [.112] * 3)
                old_upper["agents"].pop(key)
                new_upper["agents"].pop(key)
            old_upper.pop("configuration_status")
            new_upper.pop("configuration_status")
            self.assertEqual(old_upper, new_upper)
            self.assertFalse(new_upper["hardware_execution_authorized"])
            old_yaw = old["formal_fake_runtime"]["yaw_drive_disturbance_v2"]
            new_yaw = new["formal_fake_runtime"]["yaw_drive_disturbance_v2"]
            self.assertEqual(new_yaw["amplitude"], .020)
            old_yaw.pop("amplitude")
            new_yaw.pop("amplitude")
            self.assertEqual(old_yaw, new_yaw)

    def test_boundary_active_requires_contraction_and_contact(self):
        row = {
            "risk_contraction": "0.006", "common_velocity_reference": "0.101",
            "common_boundary_upper": "0.10105",
            "candidate_common_velocity": "0.102",
            "upper_effective_common_velocity": "0.10105"}
        self.assertTrue(V3.boundary_active(row))
        row["risk_contraction"] = "0"
        self.assertFalse(V3.boundary_active(row))
        self.assertTrue(V3.boundary_active(
            row, contraction_override=0.006))
        row.update(risk_contraction="0.006", upper_effective_common_velocity="0.100")
        self.assertFalse(V3.boundary_active(row))

    def test_stage1_gate_accepts_nonintrusive_reference(self):
        method = {
            "validation_valid": True,
            "manifest_nominal_common_velocity_mps": 0.10,
            "common_reference_velocity_mean_mps": 0.0999,
            "reference_below_0p099_fraction": 0.0,
            "risk_boundary_active_fraction": 0.0,
        }
        self.assertTrue(V3.stage1_passes({
            "M1_R1": dict(method), "M1b_R1": dict(method)}))
        intrusive = dict(method)
        intrusive["reference_below_0p099_fraction"] = 0.20
        self.assertFalse(V3.stage1_passes({
            "M1_R1": intrusive, "M1b_R1": dict(method)}))

    def test_paired_difference_is_not_actual_same_run_reduction(self):
        def rows(reference):
            output = []
            for index in range(3):
                output.append({
                    "stamp": str(index),
                    "yaw_drive_disturbance_path_progress": str(2.0 + 0.4 * index),
                    "common_velocity_reference": str(reference),
                    "common_boundary_upper": "0.101",
                    "risk_contraction": "0.006",
                    "agv2_robust_margin": "0.7",
                    "yaw_drive_disturbance_left_nominal": "0.12",
                    "yaw_drive_disturbance_right_nominal": "0.14",
                    "agv2_s_dot_actual": "0.10",
                })
            return output
        with tempfile.TemporaryDirectory() as temporary:
            result = V3.paired_alignment(
                rows(0.098), rows(0.100), "progress",
                pathlib.Path(temporary) / "aligned.csv")
            self.assertAlmostEqual(
                result["paired_reference_difference_peak_mps"], 0.002)
            self.assertNotIn("actual_reference_reduction_peak_mps", result)
            with (pathlib.Path(temporary) / "aligned.csv").open() as stream:
                saved = next(csv.DictReader(stream))
            self.assertAlmostEqual(
                float(saved["m1b_counterfactual_common_velocity_mps"]), 0.100)

    def test_actual_reduction_uses_own_candidate_and_preserves_sign(self):
        result = V3.reference_reduction_metrics([{
            "candidate_common_velocity": "0.1003",
            "upper_effective_common_velocity": "0.099",
            "common_velocity_reference": "0.0992"}])
        self.assertAlmostEqual(result["actual_reference_reduction_peak_mps"], 0.0011)
        self.assertAlmostEqual(result["upper_clipping_reduction_peak_mps"], 0.0013)
        result = V3.reference_reduction_metrics([{
            "candidate_common_velocity": "0.099",
            "upper_effective_common_velocity": "0.099",
            "common_velocity_reference": "0.100"}])
        self.assertAlmostEqual(result["actual_reference_reduction_mean_mps"], -0.001)

    def test_empty_or_missing_diagnostics_fail_closed(self):
        with self.assertRaises(ValueError):
            V3.reference_reduction_metrics([])
        with self.assertRaises(ValueError):
            V3.reference_reduction_metrics([{}])
        with self.assertRaises(ValueError):
            V3.paired_alignment([], [], "progress", pathlib.Path("unused.csv"))

    def test_boundary_contact_without_candidate_excess_is_not_active(self):
        self.assertFalse(V3.boundary_active({
            "risk_contraction": "0.006", "candidate_common_velocity": "0.100",
            "upper_effective_common_velocity": "0.100", "common_boundary_upper": "0.100"}))


if __name__ == "__main__":
    unittest.main()
