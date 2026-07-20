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
    "CapabilityReport.msg": [
        "std_msgs/Header header", "uint8 robot_id", "uint32 capability_seq",
        "float64 max_wheel_linear_velocity_left",
        "float64 max_wheel_linear_velocity_right",
        "float64 max_wheel_linear_acceleration_left",
        "float64 max_wheel_linear_acceleration_right",
        "float64 max_wheel_linear_deceleration_left",
        "float64 max_wheel_linear_deceleration_right",
        "float64 derating_ratio", "uint8 derating_mode",
        "bool derating_active", "bool speed_limit_active_left",
        "bool speed_limit_active_right", "bool accel_limit_active_left",
        "bool accel_limit_active_right", "bool decel_limit_active_left",
        "bool decel_limit_active_right", "float64 battery_voltage",
    ],
    "ChassisFeedback.msg": [
        "std_msgs/Header header", "uint8 robot_id", "uint32 feedback_seq",
        "uint32 command_seq_applied", "uint32 packet_seq",
        "time serial_receive_stamp", "float64 wheel_linear_velocity_left_raw",
        "float64 wheel_linear_velocity_right_raw",
        "float64 wheel_linear_velocity_left_applied",
        "float64 wheel_linear_velocity_right_applied",
        "float64 wheel_linear_velocity_left_actual",
        "float64 wheel_linear_velocity_right_actual",
        "float64 linear_velocity_actual", "float64 angular_velocity_actual",
        "sensor_msgs/Imu imu", "nav_msgs/Odometry odom",
        "float64 battery_voltage", "bool control_loop_overrun",
        "bool speed_limit_active_left", "bool speed_limit_active_right",
        "bool accel_limit_active_left", "bool accel_limit_active_right",
        "bool decel_limit_active_left", "bool decel_limit_active_right",
    ],
    "CooperativeState.msg": [
        "std_msgs/Header header", "uint8 SOURCE_UNKNOWN=0",
        "uint8 SOURCE_ODOM=1", "uint8 SOURCE_CAMERA=2",
        "uint8 SOURCE_FUSED=3", "uint8[3] robot_localization_source",
        "bool[3] robot_pose_valid", "time[3] robot_pose_stamp",
        "geometry_msgs/Pose2D[3] robot_pose", "bool[3] support_pose_valid",
        "geometry_msgs/Pose2D[3] support_pose", "bool[3] path_state_valid",
        "float64[3] s_actual", "float64[3] s_dot_actual",
        "uint8 load_localization_source", "bool load_pose_valid",
        "time load_pose_stamp", "geometry_msgs/Pose2D load_pose",
        "bool load_path_state_valid", "float64 load_s_actual",
        "float64 load_s_dot_actual",
    ],
    "PathReference.msg": [
        "std_msgs/Header header", "string path_id", "uint32 path_version",
        "float64 load_path_progress_reference",
        "float64 load_path_velocity_reference",
        "float64 load_path_acceleration_reference",
        "geometry_msgs/Pose2D load_pose_reference",
        "geometry_msgs/Pose2D[3] support_pose_reference",
        "float64[3] chassis_linear_velocity_feedforward",
        "float64[3] chassis_angular_velocity_feedforward",
    ],
    "ControllerState.msg": [
        "std_msgs/Header header", "string experiment_id", "string method_id",
        "float64[3] path_progress_actual", "float64[3] path_velocity_actual",
        "float64[3] path_progress_execute_reference",
        "float64[3] path_velocity_execute_reference",
        "float64[3] channel_input_raw", "float64[3] channel_input_limited",
        "float64 common_velocity_lower_bound",
        "float64 common_velocity_upper_bound",
        "float64 common_load_velocity_reference",
        "bool[3] channel_input_limit_active",
    ],
    "ExperimentState.msg": [
        "std_msgs/Header header", "string experiment_id", "string method_id",
        "string block_id", "string run_id", "uint8 phase",
        "bool run_active", "bool evaluation_active", "bool manual_abort",
        "string abort_reason",
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
