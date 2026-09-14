#!/usr/bin/env python3
import csv
import importlib.util
import json
import pathlib
import tempfile
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "screen_exp2c_v2_yaw_disturbance.py"
SPEC = importlib.util.spec_from_file_location("exp2c_v2_screen", str(SCRIPT))
SCREEN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCREEN)


class Exp2cV2ScreenTest(unittest.TestCase):
    def make_run(self, root, name="0p004", nominal=0.154,
                 actual_velocity=0.100, speed_limited=False):
        run = root / name / "M1b_R1"
        converted = run / "converted"
        converted.mkdir(parents=True)
        summary = {
            "method_id": "M1b_R1", "sample_period": 0.01,
            "capability": {"reported_constancy_range": {
                "agv1": 0.0, "agv2": 0.0, "agv3": 0.0}},
            "geometry": {
                "rigid_fit_residual_rmse": 0.001,
                "vehicle_heading": {"agv2": {"rmse": 0.01}},
                "support": {"agv2": {"position_rmse": 0.002}}},
        }
        (run / "summary_metrics.json").write_text(json.dumps(summary))
        (run / "validation.json").write_text(json.dumps({
            "valid": True, "task_completion": {"complete": True}}))
        base = {
            "yaw_drive_disturbance_active": "0",
            "yaw_drive_disturbance_window": "1", "yaw_drive_disturbance_phase": "1.57079632679",
            "yaw_drive_disturbance_left_nominal": str(nominal),
            "yaw_drive_disturbance_right_nominal": str(nominal),
            "yaw_drive_disturbance_left_disturbed": str(nominal - 0.004),
            "yaw_drive_disturbance_right_disturbed": str(nominal + 0.004),
            "yaw_drive_disturbance_mean_longitudinal_delta": "0",
            "agv2_wheel_left_reported_limit": "0.16",
            "agv2_wheel_right_reported_limit": "0.16",
            "agv2_robust_margin": "0.6",
            "agv2_s_dot_actual": str(actual_velocity),
            "common_velocity_reference": "0.1",
            "agv2_wheel_left_speed_limit_active": "0",
            "agv2_wheel_right_speed_limit_active": "0",
            "agv2_hard_inner_upper": "0.12",
            "agv2_mapped_path_velocity_upper": "0.13",
            "agv2_capability_derating_active": "0",
        }
        rows = [dict(base)]
        for _ in range(4):
            row = dict(base)
            row["yaw_drive_disturbance_active"] = "1"
            if speed_limited:
                row["agv2_wheel_left_speed_limit_active"] = "1"
            rows.append(row)
        with (converted / "aligned_samples.csv").open(
                "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(base))
            writer.writeheader()
            writer.writerows(rows)
        return run

    def test_selector_accepts_predeclared_m1b_candidate(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.make_run(pathlib.Path(temporary))
            result = SCREEN.evaluate_candidate(run, 0.004)
            self.assertTrue(result["passes_predeclared_m1b_screen"])
            self.assertAlmostEqual(
                result["central_causal_wheel_margin_minimum"], 0.6)

    def test_selector_rejects_margin_outside_range(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.make_run(pathlib.Path(temporary), nominal=0.14)
            result = SCREEN.evaluate_candidate(run, 0.004)
            self.assertFalse(result["passes_predeclared_m1b_screen"])

    def test_selector_rejects_longitudinal_speed_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.make_run(
                pathlib.Path(temporary), actual_velocity=0.090)
            result = SCREEN.evaluate_candidate(run, 0.004)
            self.assertFalse(result["passes_predeclared_m1b_screen"])

    def test_selector_rejects_physical_speed_limiter(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.make_run(pathlib.Path(temporary), speed_limited=True)
            result = SCREEN.evaluate_candidate(run, 0.004)
            self.assertFalse(result["passes_predeclared_m1b_screen"])

    def test_selector_main_reads_only_m1b_directories(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            for name, _ in SCREEN.CANDIDATES:
                self.make_run(root, name=name)
                # This deliberately invalid M1 file must never be consulted.
                m1 = root / name / "M1_R1"
                m1.mkdir()
                (m1 / "summary_metrics.json").write_text("not json")
            old_argv = SCREEN.sys.argv
            try:
                SCREEN.sys.argv = [str(SCRIPT), str(root)]
                self.assertEqual(SCREEN.main(), 0)
            finally:
                SCREEN.sys.argv = old_argv
            report = json.loads((root / "selection_report.json").read_text())
            self.assertFalse(report["m1_results_consulted"])
            self.assertFalse(report["m1_run_started"])
            self.assertEqual(report["selected_amplitude_mps"], 0.004)

    def test_round2_candidate_set_is_explicit_and_bounded(self):
        self.assertEqual(SCREEN.CANDIDATE_SETS["round2"], (
            ("0p016", 0.016), ("0p020", 0.020), ("0p024", 0.024)))

    def test_invalid_incomplete_candidate_is_reported_not_selected(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.make_run(pathlib.Path(temporary))
            (run / "summary_metrics.json").unlink()
            (run / "validation.json").write_text(json.dumps({
                "valid": False, "task_completion": {"complete": False},
                "issues": [{"code": "TASK_INCOMPLETE"}]}))
            result = SCREEN.rejected_candidate(run, 0.024, "missing metrics")
            self.assertFalse(result["passes_predeclared_m1b_screen"])
            self.assertEqual(result["validation_issue_codes"],
                             ["TASK_INCOMPLETE"])


if __name__ == "__main__":
    unittest.main()
