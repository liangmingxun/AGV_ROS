#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest

from agv_msgs.msg import ChassisCommand
from nav_msgs.msg import Path


class SingleCarSPretestTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._commands = []
        self._reference_path = None
        self._command_subscriber = rospy.Subscriber(
            "/agv1/chassis_command", ChassisCommand,
            self._receive_command, queue_size=1000)
        self._path_subscriber = rospy.Subscriber(
            "/agv1/s_pretest/reference_path", Path,
            self._receive_path, queue_size=1)

    def _receive_command(self, message):
        with self._condition:
            self._commands.append(message)
            self._condition.notify_all()

    def _receive_path(self, message):
        with self._condition:
            self._reference_path = message
            self._condition.notify_all()

    def test_bounded_s_command_and_stop(self):
        deadline = rospy.Time.now() + rospy.Duration(12.0)
        with self._condition:
            while rospy.Time.now() < deadline:
                nonzero = [
                    command for command in self._commands
                    if abs(command.wheel_linear_velocity_left_raw) > 1.0e-5
                    or abs(command.wheel_linear_velocity_right_raw) > 1.0e-5
                ]
                if nonzero and len(self._commands) >= 5:
                    trailing = self._commands[-5:]
                    if all(
                        abs(command.wheel_linear_velocity_left_raw) <= 1.0e-9
                        and abs(command.wheel_linear_velocity_right_raw) <= 1.0e-9
                        for command in trailing
                    ):
                        break
                self._condition.wait(0.05)

        self.assertIsNotNone(self._reference_path)
        self.assertEqual(self._reference_path.header.frame_id, "agv1/odom")
        self.assertEqual(len(self._reference_path.poses), 501)

        nonzero = [
            command for command in self._commands
            if abs(command.wheel_linear_velocity_left_raw) > 1.0e-5
            or abs(command.wheel_linear_velocity_right_raw) > 1.0e-5
        ]
        self.assertGreater(len(nonzero), 20)
        self.assertTrue(all(command.robot_id == 1 for command in self._commands))
        self.assertTrue(all(
            abs(command.wheel_linear_velocity_left_raw) <= 0.08 + 1.0e-9
            and abs(command.wheel_linear_velocity_right_raw) <= 0.08 + 1.0e-9
            for command in nonzero
        ))

        wheel_differences = [
            command.wheel_linear_velocity_right_raw
            - command.wheel_linear_velocity_left_raw
            for command in nonzero
        ]
        self.assertLess(min(wheel_differences), -1.0e-3)
        self.assertGreater(max(wheel_differences), 1.0e-3)

        trailing = self._commands[-5:]
        self.assertEqual(len(trailing), 5)
        self.assertTrue(all(
            abs(command.wheel_linear_velocity_left_raw) <= 1.0e-9
            and abs(command.wheel_linear_velocity_right_raw) <= 1.0e-9
            for command in trailing
        ))


if __name__ == "__main__":
    rospy.init_node("test_single_car_s_pretest")
    rostest.rosrun(
        "multi_agv_control", "single_car_s_pretest",
        SingleCarSPretestTest)
