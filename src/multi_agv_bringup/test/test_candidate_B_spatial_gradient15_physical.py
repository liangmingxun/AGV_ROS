#!/usr/bin/env python3
import pathlib
import subprocess
import unittest

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent.parent
CONFIG = PACKAGE / "config"
SCRIPT = PACKAGE / "scripts/run_candidate_B_spatial_gradient15_physical.sh"
NODE = ROOT / "src/multi_agv_control/src/formal_fake_algorithm_node.cpp"
RECORDER = ROOT / "src/multi_agv_analysis/scripts/record_experiment.py"


class CandidateBSpatialGradient15PhysicalTest(unittest.TestCase):
    def test_profile_is_identical_to_fake_and_dual_fail_closed(self):
        physical = yaml.safe_load((CONFIG /
            "formal_serial_candidate_B_spatial_gradient15.yaml").read_text())[
                "formal_fake_runtime"]
        fake = yaml.safe_load((CONFIG /
            "formal_fake_candidate_B_spatial_gradient15.yaml").read_text())[
                "formal_fake_runtime"]
        expected = dict(fake["candidate_b_spatial_composite"])
        expected["enabled"] = False
        self.assertEqual(physical["candidate_b_spatial_composite"], expected)
        self.assertTrue(physical["candidate_b_spatial_physical"][
            "software_qualification_passed"])
        self.assertFalse(physical["candidate_b_spatial_physical"]["enabled"])
        self.assertFalse(physical["candidate_b_spatial_physical"][
            "hardware_execution_authorized"])
        self.assertNotIn("authorization_status",
                         physical["candidate_b_spatial_physical"])

    def test_all_method_authorizations_default_false(self):
        for method in ("M1", "M1b", "M2b"):
            data = yaml.safe_load((CONFIG /
                ("formal_serial_candidate_B_spatial_gradient15_{}_authorization.yaml".format(
                    method))).read_text())
            self.assertFalse(data["formal_upper"][
                "hardware_execution_authorized"])
            self.assertFalse(data["formal_lower"][
                "hardware_execution_authorized"])
            self.assertNotIn("physical_authorization_status",
                             data["authorization_scope"])
            self.assertEqual(data["authorization_scope"]["experiment_id"],
                             "candidate_B_spatial_gradient15_physical_validation")

    def test_dry_run_never_contacts_ros_and_missing_confirmation_refuses(self):
        for method in ("M1", "M1b", "M2b"):
            result = subprocess.run(
                [str(SCRIPT), "--method", method,
                 "--software-only-dry-run"], cwd=ROOT,
                capture_output=True, text=True, check=True)
            self.assertIn("SERIAL_COMMAND_PUBLISHED=NO", result.stdout)
            self.assertIn("SOFTWARE_QUALIFICATION=PASSED", result.stdout)
            self.assertIn(
                "PHYSICAL_ENTRY=READY_WITH_PER_RUN_OPERATOR_AUTHORIZATION",
                result.stdout)
            self.assertIn("SEVERITY_SCALE=1.00,1.15,0.85", result.stdout)
        refused = subprocess.run([
            str(SCRIPT), "--method", "M1", "--operator", "TEST",
            "--pair-block", "B01", "--confirm-area-clear",
            "--confirm-wheels-on-floor", "--confirm-unloaded-30cm-fixture",
            "--confirm-emergency-stop-ready", "--confirm-vision-valid",
            "--confirm-recorder-ready", "--confirm-hardware-authorization"],
            cwd=ROOT,
            capture_output=True, text=True)
        self.assertEqual(refused.returncode, 2)
        self.assertIn("every physical confirmation", refused.stderr)

    def test_operator_authorization_uses_gitignored_runtime_overlay(self):
        source = SCRIPT.read_text()
        self.assertIn(".runtime_authorization", source)
        self.assertIn("runtime_operator_overlay", source)
        self.assertIn("FORMAL_EXECUTION_AUTHORIZATION_BASE_CONFIG", source)
        self.assertIn("FORMAL_CANDIDATE_B_SPATIAL_BASE_CONFIG", source)
        self.assertIn("physical_runtime.log", source)
        self.assertIn(".runtime_authorization/", (ROOT / ".gitignore").read_text())

    def test_node_scope_and_recorder_are_physical_specific(self):
        source = NODE.read_text()
        self.assertIn("candidate_b_spatial_physical_enabled_", source)
        self.assertIn("candidate_b_spatial_physical_hardware_authorized_",
                      source)
        self.assertIn("candidate_B_spatial_gradient15_physical_validation",
                      source)
        self.assertIn('transport_type_ == "serial"', source)
        recorder = RECORDER.read_text()
        self.assertIn("record_candidate_b_spatial_physical", recorder)
        self.assertIn("/multi_agv/candidate_B_spatial_composite_state",
                      recorder)
        self.assertIn("/multi_agv/formal_execution_limiter_state", recorder)

    def test_core_limits_and_stage_d_are_not_changed(self):
        runtime = yaml.safe_load((CONFIG /
            "formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml").read_text())
        self.assertEqual(runtime["formal_fake_runtime"][
            "emergency_abort_limit"], .18)
        status = subprocess.run(
            ["git", "diff", "--name-only", "--",
             "src/chassis_controller",
             "src/multi_agv_bringup/config/wheel_speed_scale_stage_d_freeze_v1.yaml",
             "src/multi_agv_control/src/capability_mapper.cpp"],
            cwd=ROOT, capture_output=True, text=True, check=True)
        self.assertEqual(status.stdout.strip(), "")

    def test_neutral_default_does_not_override_other_disturbances(self):
        neutral = yaml.safe_load((CONFIG /
            "formal_serial_candidate_B_spatial_disabled.yaml").read_text())[
                "formal_fake_runtime"]
        self.assertNotIn("classic_additive_candidate_A", neutral)
        self.assertNotIn("classic_additive_physical", neutral)
        self.assertFalse(neutral["candidate_b_spatial_composite"]["enabled"])


if __name__ == "__main__":
    unittest.main()
