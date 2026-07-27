#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SingleRobotArchitectureTest(unittest.TestCase):
    def test_main_has_no_car_to_car_algorithm_or_udp_receiver(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for forbidden in ("SwarmData", "AlgorithmCommand", "udpReceiverThread",
                          "robot_ip", '"/agv_chassis/cmd_vel"'):
            self.assertNotIn(forbidden, source)

    def test_main_uses_relative_frozen_topics_and_fake_transport(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for required in ('"chassis_command"', '"derating_command"',
                         '"chassis_feedback"', '"capability_report"',
                         'transport_type_ != "fake"'):
            self.assertIn(required, source)

    def test_frozen_queue_sizes_and_reset_service_are_present(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for required in (
                'subscribe("chassis_command", 1',
                'subscribe("derating_command", 5',
                '"chassis_feedback", 5, false',
                '"capability_report", 5, false',
                'advertise<nav_msgs::Odometry>("odom", 10, false)',
                'advertise<sensor_msgs::Imu>("imu", 10, false)',
                '"reset_odometry"'):
            self.assertIn(required, source)
        self.assertGreaterEqual(
            source.count("ros::TransportHints().tcpNoDelay()"), 2)
        self.assertIn('"capability_publish_rate", 20.0', source)

    def test_identity_frames_and_calibration_are_fail_fast(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for required in (
                'robot_id_ != "agv" + std::to_string(robot_index)',
                'odom_frame_ != robot_id_ + "/odom"',
                'base_frame_ != robot_id_ + "/base_link"',
                'imu_frame_ != robot_id_ + "/imu_link"',
                'values.size() != 3',
                'IMU calibration parameters are invalid'):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
