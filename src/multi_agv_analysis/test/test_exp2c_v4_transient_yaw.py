#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("v4",str(PACKAGE/"scripts/analyze_exp2c_v4_transient_yaw.py"))
V4 = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(V4)


class TransientYawRecoveryTest(unittest.TestCase):
    def test_only_two_candidates_and_predeclared_recovery_rule(self):
        config = PACKAGE.parent/"multi_agv_bringup/config"
        freeze = yaml.safe_load((config/"exp2c_v4_transient_yaw_recovery_freeze.yaml").read_text())[V4.IDENTITY]
        self.assertEqual(freeze["candidate_order"],["A","B"])
        self.assertEqual(freeze["baseline_window_seconds"],V4.BASELINE_SECONDS)
        self.assertEqual(freeze["recovery_observation_seconds"],V4.RECOVERY_SECONDS)
        self.assertEqual(freeze["recovery_required_hold_seconds"],V4.HOLD_SECONDS)
        for method in ("M1","M1b"):
            old = yaml.safe_load((config/("exp2c_v3_{}_nominal_headroom.yaml".format(method))).read_text())["formal_upper"]
            old.pop("configuration_status")
            for candidate in ("A","B"):
                new = yaml.safe_load((config/("exp2c_v4_{}_candidate_{}.yaml".format(method,candidate))).read_text())
                upper = new["formal_upper"]; upper.pop("configuration_status")
                self.assertEqual(old,upper)
                runtime = new["formal_fake_runtime"]
                pulse = runtime["transient_yaw_v4"]
                self.assertEqual((pulse["amplitude"],pulse["duration"]),V4.CANDIDATES[candidate])
                self.assertFalse(runtime["yaw_drive_disturbance_v2"]["enabled"])
                self.assertFalse(runtime["risk_disturbance_v1"]["enabled"])
                auth = yaml.safe_load((config/("formal_exp2c_v4_{}_{}_authorization.yaml".format(method,candidate))).read_text())
                self.assertFalse(auth["formal_upper"]["hardware_execution_authorized"])
                self.assertFalse(auth["formal_lower"]["hardware_execution_authorized"])
                self.assertFalse(auth["transient_yaw_v4"]["hardware_execution_authorized"])

    def test_recovery_requires_continuous_half_second(self):
        rows = [{"relative_wall_time":2.+i*.01,"errors":{"heading":0. if i >= 20 else .1}} for i in range(100)]
        bounds = {"lower":-.01,"upper":.01}
        self.assertAlmostEqual(V4.recovery_time(rows,"heading",bounds,2.),.2)
        self.assertIsNone(V4.recovery_time(rows[:60],"heading",bounds,2.))

    def test_missing_samples_do_not_count_as_sustained_recovery(self):
        rows = [{"relative_wall_time":value,"errors":{"heading":0.}} for value in (2.,2.01,3.,3.01)]
        self.assertIsNone(V4.recovery_time(rows,"heading",{"lower":-.01,"upper":.01},2.))

    def test_mad_envelope_has_fixed_floor(self):
        bounds = V4.envelope([.002]*200,.001)
        self.assertAlmostEqual(bounds["lower"],.001)
        self.assertAlmostEqual(bounds["upper"],.003)
        with self.assertRaises(ValueError): V4.envelope([math.nan],.001)

    def test_wall_clock_durations_are_not_progress_based(self):
        rows = [{"relative_wall_time":value} for value in (0.,.01,.025)]
        self.assertAlmostEqual(sum(V4.durations(rows)),.0375)
        with self.assertRaises(ValueError): V4.durations([{"relative_wall_time":0.},{"relative_wall_time":.3}])

    def test_signed_actual_reduction_is_not_nominal_upper_gap(self):
        result = V4.V3.reference_reduction_metrics([{
            "candidate_common_velocity":.101,"common_velocity_reference":.098,
            "upper_effective_common_velocity":.098}])
        self.assertAlmostEqual(result["actual_reference_reduction_peak_mps"],.003)

    def test_startup_limiting_is_not_attributed_to_the_pulse(self):
        rows = [{"relative_wall_time":-.02+i*.01,
                 "transient_yaw_triggered":"0.0" if i < 2 else "1.0",
                 "transient_yaw_progress":.05 if i < 2 else 2.,
                 "agv2_wheel_right_speed_limit_active":i == 0} for i in range(4)]
        result = V4.limiter_locations(rows)
        self.assertEqual(result["samples"],1)
        self.assertTrue(result["all_before_transient_trigger"])
        self.assertAlmostEqual(result["duration_seconds"],.01)
        self.assertEqual(result["robot2_progress_range_m"],[.05,.05])

    def test_runner_is_isolated_and_does_not_search_or_reuse_screen(self):
        bringup = PACKAGE.parent/"multi_agv_bringup"
        script = bringup/"scripts/run_exp2c_v4_transient_yaw_recovery_fake.sh"
        subprocess.run(["bash","-n",str(script)],check=True)
        text = script.read_text()
        self.assertIn('http://127.0.0.1:${ros_port}',text)
        self.assertIn('for candidate in A B',text)
        self.assertIn('"$screen_status" == 10',text)
        self.assertIn('run_one paired M1b "$selected"',text)
        self.assertIn('run_one paired M1 "$selected"',text)
        self.assertNotIn('platform_transport_type:=serial',text)
        launch = ET.parse(bringup/"launch/experiment.launch")
        self.assertEqual(launch.find("arg[@name='record_topics_config']").get("default"),
            "$(find multi_agv_bringup)/config/record_topics.yaml")
        config = yaml.safe_load((bringup/"config/record_topics_exp2c_v4.yaml").read_text())
        self.assertIn('/multi_agv/transient_yaw_disturbance_state',config['experiment_recording']['required_topics'])


if __name__ == "__main__": unittest.main()
