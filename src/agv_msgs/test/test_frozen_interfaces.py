#!/usr/bin/env python3
import pathlib
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


EXPECTED = {
    "ChassisCommand.msg": [
        "std_msgs/Header header", "uint8 robot_id", "uint32 command_seq",
        "uint8 control_mode", "float64 linear_velocity_reference",
        "float64 angular_velocity_reference",
        "float64 wheel_linear_velocity_left_raw",
        "float64 wheel_linear_velocity_right_raw", "string experiment_id",
        "string method_id",
    ],
    "DeratingCommand.msg": [
        "std_msgs/Header header", "uint8 robot_id", "uint32 command_seq",
        "uint8 mode", "bool active", "float64 target_speed_ratio_left",
        "float64 target_speed_ratio_right", "float64 target_accel_ratio_left",
        "float64 target_accel_ratio_right", "float64 target_decel_ratio_left",
        "float64 target_decel_ratio_right", "float64 ramp_down_time",
        "float64 ramp_up_time", "string experiment_id",
    ],
}


class FrozenInterfaceTest(unittest.TestCase):
    def test_required_message_files_match_frozen_fields(self):
        for name, expected in EXPECTED.items():
            with self.subTest(name=name):
                content = (PACKAGE / "msg" / name).read_text(encoding="utf-8")
                fields = [line.strip() for line in content.splitlines()
                          if line.strip() and not line.lstrip().startswith("#")]
                self.assertEqual(expected, fields)

    def test_all_eight_messages_are_registered(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        for name in (
            "ChassisCommand.msg", "ChassisFeedback.msg", "CapabilityReport.msg",
            "DeratingCommand.msg", "CooperativeState.msg", "PathReference.msg",
            "ControllerState.msg", "ExperimentState.msg",
        ):
            self.assertIn(name, cmake)


if __name__ == "__main__":
    unittest.main()
