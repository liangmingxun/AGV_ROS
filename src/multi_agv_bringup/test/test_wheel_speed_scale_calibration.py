#!/usr/bin/env python3
import importlib.util
import csv
import math
import pathlib
import subprocess
import sys
import tempfile
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "wheel_speed_scale_calibration.py"
WRAPPER = PACKAGE / "scripts" / "run_wheel_speed_scale_calibration.sh"
ANALYZER = PACKAGE / "scripts" / "analyze_wheel_speed_scale_calibration.py"
SPEC = importlib.util.spec_from_file_location("wheel_scale", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class WheelSpeedScaleCalibrationTest(unittest.TestCase):
    def test_speed_list_is_bounded_and_ordered(self):
        self.assertEqual(
            MODULE.parse_speed_list("0.06,0.08,0.10,0.12,0.14,0.16"),
            MODULE.DEFAULT_SPEEDS,
        )
        with self.assertRaises(Exception):
            MODULE.parse_speed_list("0.08,0.06")
        with self.assertRaises(Exception):
            MODULE.parse_speed_list("0.17")

    def test_command_sequence_handoff_follows_running_chassis(self):
        self.assertEqual(
            MODULE.synchronize_command_sequence(100000, 3100000000),
            3100000000,
        )
        self.assertEqual(
            MODULE.synchronize_command_sequence(3100000000, 100000),
            3100000000,
        )
        with self.assertRaisesRegex(ValueError, "uint32 exhaustion"):
            MODULE.synchronize_command_sequence(100000, 0xFFFFFF00)

    def test_changing_motion_commands_receive_strictly_newer_sequences(self):
        sequence = MODULE.synchronize_command_sequence(100000, 3100000000)
        published = []
        for _ in range(1000):
            sequence = MODULE.advance_command_sequence(sequence)
            published.append(sequence)
        self.assertEqual(len(published), len(set(published)))
        self.assertTrue(all(
            newer > older for older, newer in zip(published, published[1:])
        ))
        with self.assertRaisesRegex(ValueError, "uint32 exhaustion"):
            MODULE.advance_command_sequence(0xFFFFFF00)

    def test_actual_anomaly_gate_does_not_reuse_prelimit_threshold(self):
        self.assertAlmostEqual(
            MODULE.sustained_actual_abort_threshold(0.16), 0.19, places=12
        )
        self.assertAlmostEqual(
            MODULE.sustained_actual_abort_threshold(-0.14), 0.17, places=12
        )

    def test_camera_wheel_velocity_uses_body_and_yaw_fit(self):
        track = 0.14
        body = 0.10
        omega = 0.20
        samples = []
        for index in range(250):
            stamp = index * 0.01
            yaw = omega * stamp
            # This short synthetic arc is sufficiently represented by its
            # mid-window tangent for the fixed-window velocity estimator.
            radius = body / omega
            x = radius * math.sin(yaw)
            y = radius * (1.0 - math.cos(yaw))
            samples.append((stamp, x, y, yaw))
        result = MODULE.camera_wheel_velocity(samples, track)
        self.assertAlmostEqual(result["body_velocity_mps"], body, delta=0.003)
        self.assertAlmostEqual(result["yaw_rate_radps"], omega, places=6)
        self.assertAlmostEqual(
            result["wheel_left_physical_mps"], body - track * omega / 2,
            delta=0.003,
        )

    def test_camera_fit_accepts_normal_low_rate_coverage(self):
        samples = [
            (index * 0.05, index * 0.05 * 0.08, 0.0, 0.0)
            for index in range(41)
        ]
        result = MODULE.camera_wheel_velocity(samples, 0.14)
        self.assertAlmostEqual(result["body_velocity_mps"], 0.08, places=9)
        with self.assertRaisesRegex(ValueError, "insufficient camera coverage"):
            MODULE.camera_wheel_velocity(samples[:20], 0.14)

    def test_fit_separates_feedback_and_command_mapping(self):
        segments = []
        feedback_gain = {"left": 1.10, "right": 1.12}
        response_gain = {"left": 1.08, "right": 1.09}
        for repetition in (1, 2, 3):
            for speed in (0.06, 0.10, 0.14):
                for sign, direction in ((1.0, "forward"), (-1.0, "reverse")):
                    row = {
                        "repetition": repetition,
                        "direction": direction,
                        "physical_target_point_mps": sign * speed,
                        "active_feedback_scale_left": 1.0,
                        "active_feedback_scale_right": 1.0,
                    }
                    for side in ("left", "right"):
                        physical = sign * speed
                        row["camera_wheel_{}_physical_mps".format(side)] = physical
                        row["stm32_wheel_{}_measured_nominal_mps".format(side)] = (
                            physical / feedback_gain[side]
                        )
                        row["firmware_wheel_{}_target_nominal_mps".format(side)] = (
                            physical / response_gain[side]
                        )
                    segments.append(row)
        result = MODULE.fit_candidates(segments, 3)
        MODULE.annotate_segment_residuals(segments, result)
        self.assertEqual(result["status"], "REVIEW_REQUIRED")
        for side in ("left", "right"):
            self.assertAlmostEqual(
                result["wheels"][side]["wheel_feedback_scale_candidate"],
                feedback_gain[side], places=9,
            )
            self.assertAlmostEqual(
                result["wheels"][side][
                    "physical_to_firmware_command_scale_candidate"
                ], 1.0 / response_gain[side], places=9,
            )
        with tempfile.TemporaryDirectory() as directory:
            MODULE.generate_fit_plots(segments, result, directory)
            for name in (
                "scale_fit.png", "scale_fit.pdf",
                "scale_residuals.png", "scale_residuals.pdf",
            ):
                self.assertGreater((pathlib.Path(directory) / name).stat().st_size, 0)
            metrics = pathlib.Path(directory) / "speed_point_metrics.csv"
            with metrics.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(segments[0]))
                writer.writeheader()
                writer.writerows(segments)
            (pathlib.Path(directory) / "metadata.txt").write_text(
                "robot_id=agv2\noperator=TEST\n"
            )
            offline = subprocess.run(
                [sys.executable, str(ANALYZER), directory],
                text=True, capture_output=True,
            )
            self.assertEqual(offline.returncode, 0, offline.stderr)
            self.assertIn("OFFLINE WHEEL SCALE ANALYSIS COMPLETE", offline.stdout)
            self.assertTrue((pathlib.Path(directory) / "scale_candidate.yaml").is_file())

    def test_wrapper_refuses_without_all_confirmations_before_ros(self):
        result = subprocess.run(
            ["bash", str(WRAPPER), "--robot-id", "2", "--operator", "ETLAB"],
            text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("all five", result.stderr)


if __name__ == "__main__":
    unittest.main()
