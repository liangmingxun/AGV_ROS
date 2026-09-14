#!/usr/bin/env python3
import csv
import importlib.util
import pathlib
import tempfile
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "analyze_exp2c_v3_nominal_headroom.py"
SPEC = importlib.util.spec_from_file_location("exp2c_v3", str(SCRIPT))
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


class Exp2cV3NominalHeadroomTest(unittest.TestCase):
    def test_boundary_active_requires_contraction_and_contact(self):
        row = {
            "risk_contraction": "0.006", "common_velocity_reference": "0.101",
            "common_boundary_upper": "0.10105"}
        self.assertTrue(V3.boundary_active(row))
        row["risk_contraction"] = "0"
        self.assertFalse(V3.boundary_active(row))
        row.update(risk_contraction="0.006", common_velocity_reference="0.100")
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

    def test_actual_reduction_uses_paired_candidate_not_nominal_upper(self):
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
                result["actual_reference_reduction_peak_mps"], 0.002)
            self.assertIn("M1b_candidate", result["definition"])
            with (pathlib.Path(temporary) / "aligned.csv").open() as stream:
                saved = next(csv.DictReader(stream))
            self.assertAlmostEqual(
                float(saved["candidate_common_velocity_mps"]), 0.100)


if __name__ == "__main__":
    unittest.main()
