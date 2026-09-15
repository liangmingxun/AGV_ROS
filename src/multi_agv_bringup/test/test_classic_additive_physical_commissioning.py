#!/usr/bin/env python3
import pathlib
import subprocess
import unittest

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent.parent
CONFIG = PACKAGE / "config"
SCRIPT = PACKAGE / "scripts/run_classic_additive_candidate_A_physical_commissioning.sh"
NODE = ROOT / "src/multi_agv_control/src/formal_fake_algorithm_node.cpp"


class ClassicAdditivePhysicalCommissioningTest(unittest.TestCase):
    def test_profile_is_frozen_and_default_is_dual_fail_closed(self):
        data = yaml.safe_load((CONFIG /
            "formal_serial_classic_additive_candidate_A_commissioning.yaml").read_text())
        runtime = data["formal_fake_runtime"]
        profile = runtime["classic_additive_candidate_A"]
        self.assertFalse(profile["enabled"])
        self.assertEqual(profile["trigger_progress"], 2.0)
        self.assertEqual(profile["linear_amplitude"], .030)
        self.assertEqual(profile["angular_amplitude"], .350)
        self.assertEqual(profile["linear_frequency"], 1.0)
        self.assertEqual(profile["angular_frequency"], 1.0)
        self.assertAlmostEqual(profile["angular_phase"], 1.5707963267948966)
        self.assertEqual(profile["duration"], 8.0)
        self.assertEqual(profile["ramp_in"], .5)
        self.assertEqual(profile["ramp_out"], .5)
        physical = runtime["classic_additive_physical"]
        self.assertFalse(physical["enabled"])
        self.assertFalse(physical["hardware_execution_authorized"])
        self.assertEqual(physical["allowed_scales"], [.25, .50, .75, 1.0])
        self.assertEqual(physical["maximum_qualified_scale"], .25)
        self.assertFalse(physical["m2b_physical_authorized"])

    def test_only_m1_m1b_and_exact_scales_reach_dry_run(self):
        bad_method = subprocess.run(
            [str(SCRIPT), "--method", "M2b", "--scale", "0.25",
             "--software-only-dry-run"], cwd=ROOT,
            capture_output=True, text=True)
        self.assertEqual(bad_method.returncode, 2)
        self.assertIn("permits only M1 or M1b", bad_method.stderr)
        bad_scale = subprocess.run(
            [str(SCRIPT), "--method", "M1", "--scale", "0.30",
             "--software-only-dry-run"], cwd=ROOT,
            capture_output=True, text=True)
        self.assertEqual(bad_scale.returncode, 2)
        baseline = subprocess.run(
            [str(SCRIPT), "--method", "M1", "--scale", "0",
             "--software-only-dry-run"], cwd=ROOT,
            capture_output=True, text=True, check=True)
        self.assertIn("LAMBDA_ZERO_BASELINE_EXACT=YES", baseline.stdout)
        self.assertIn("SERIAL_COMMAND_PUBLISHED=NO", baseline.stdout)

    def test_runtime_scope_and_insertion_order_are_fail_closed(self):
        source = NODE.read_text()
        self.assertIn("classic_additive_physical_hardware_authorized_", source)
        self.assertIn("transport_type_ == \"serial\"", source)
        self.assertIn(
            "classic_additive_candidate_A_physical_commissioning", source)
        self.assertIn("physical_method && classic_additive_config_.scale > 0.0", source)
        demand_index = source.index("wheel_demand_before_limit[index] =")
        injection_index = source.index(
            "applyClassicAdditive(now, tracking", demand_index)
        safety_index = source.index(
            "trackingPassesSerialEmergencyGate(execution_tracking",
            injection_index)
        self.assertLess(demand_index, injection_index)
        self.assertLess(injection_index, safety_index)
        self.assertLess(source.index("trackingPassesSerialEmergencyGate(execution_tracking"),
                        source.index("fillWheelPublication(index, execution_tracking"))
        self.assertIn(
            "previous_wheel_raw_[index] = wheel_demand_before_limit[index]",
            source)
        self.assertNotIn("classic_additive_output_", (
            ROOT / "src/multi_agv_control/src/upper_reference_generator.cpp").read_text())

    def test_stage_d_and_capability_paths_are_not_modified(self):
        status = subprocess.run(
            ["git", "diff", "--name-only", "--",
             "src/chassis_controller",
             "src/multi_agv_control/src/capability_mapper.cpp"],
            cwd=ROOT, capture_output=True, text=True, check=True)
        self.assertEqual(status.stdout.strip(), "")
        runtime = yaml.safe_load((CONFIG /
            "formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml").read_text())
        self.assertEqual(runtime["formal_fake_runtime"]["emergency_abort_limit"], .18)

    def test_recorder_keeps_four_wheel_chain_stages_separate(self):
        recorder = (ROOT /
            "src/multi_agv_analysis/scripts/record_experiment.py").read_text()
        launch = (PACKAGE / "launch/experiment.launch").read_text()
        conversion = (ROOT /
            "src/multi_agv_analysis/src/multi_agv_analysis/conversion.py").read_text()
        for topic in ("/multi_agv/classic_additive_disturbance_state",
                      "/multi_agv/classic_additive_physical_state"):
            self.assertIn(topic, recorder)
        self.assertIn("record_classic_additive_physical", launch)
        for field in ("wheel_pre_left", "wheel_post_disturbance_left",
                      "wheel_post_limit_left", "wheel_actual_left"):
            self.assertIn(field, conversion)

    def test_real_entry_refuses_before_ros_while_candidate_unauthorized(self):
        result = subprocess.run([
            str(SCRIPT), "--method", "M1", "--scale", "0.25",
            "--operator", "TEST", "--pair-block", "P2_TEST",
            "--confirm-area-clear", "--confirm-wheels-on-floor",
            "--confirm-unloaded-30cm-fixture", "--confirm-vision-valid",
            "--confirm-recorder-ready", "--confirm-hardware-authorization",
            "--confirm-physical-execution"], cwd=ROOT,
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 3)
        self.assertIn("remains fail-closed", result.stderr)


if __name__ == "__main__":
    unittest.main()
