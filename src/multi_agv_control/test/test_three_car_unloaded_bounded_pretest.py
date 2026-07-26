#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest

from agv_msgs.msg import ChassisCommand, ControllerState, PathReference


class ThreeCarUnloadedBoundedPretestTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._commands = [[], [], []]
        self._references = []
        self._controllers = []
        self._subscribers = []
        for index in range(3):
            self._subscribers.append(rospy.Subscriber(
                "/agv{}/chassis_command".format(index + 1),
                ChassisCommand,
                lambda message, robot=index: self._receive_command(
                    robot, message),
                queue_size=1000))
        self._subscribers.append(rospy.Subscriber(
            "/multi_agv/bounded_pretest/path_reference",
            PathReference, self._receive_reference, queue_size=1000))
        self._subscribers.append(rospy.Subscriber(
            "/multi_agv/bounded_pretest/controller_state",
            ControllerState, self._receive_controller, queue_size=1000))

    def _receive_command(self, index, message):
        with self._condition:
            self._commands[index].append(message)
            self._condition.notify_all()

    def _receive_reference(self, message):
        with self._condition:
            self._references.append(message)
            self._condition.notify_all()

    def _receive_controller(self, message):
        with self._condition:
            self._controllers.append(message)
            self._condition.notify_all()

    @staticmethod
    def _nonzero(messages):
        return [
            message for message in messages
            if abs(message.wheel_linear_velocity_left_raw) > 1.0e-5
            or abs(message.wheel_linear_velocity_right_raw) > 1.0e-5
        ]

    @staticmethod
    def _stopped(messages):
        return len(messages) >= 5 and all(
            abs(message.wheel_linear_velocity_left_raw) <= 1.0e-9
            and abs(message.wheel_linear_velocity_right_raw) <= 1.0e-9
            for message in messages[-5:])

    def test_common_bounded_motion_and_all_car_stop(self):
        deadline = rospy.Time.now() + rospy.Duration(28.0)
        with self._condition:
            while rospy.Time.now() < deadline:
                if (all(len(self._nonzero(messages)) > 20
                        for messages in self._commands)
                        and all(self._stopped(messages)
                                for messages in self._commands)
                        and self._references
                        and abs(
                            self._references[-1]
                            .load_path_progress_reference - 1.00) < 1.0e-6
                        and abs(
                            self._references[-1]
                            .load_path_velocity_reference) < 1.0e-9
                        and self._controllers
                        and abs(
                            self._controllers[-1]
                            .common_load_velocity_reference) < 1.0e-9):
                    break
                self._condition.wait(0.05)

        for index, messages in enumerate(self._commands):
            self.assertGreater(
                len(self._nonzero(messages)), 20,
                "agv{} did not receive bounded motion".format(index + 1))
            self.assertTrue(all(
                message.robot_id == index + 1 for message in messages))
            self.assertTrue(all(
                message.experiment_id
                == "three_car_unloaded_s_1m"
                for message in messages))
            self.assertTrue(all(
                abs(message.wheel_linear_velocity_left_raw) <= 0.08 + 1.0e-9
                and abs(message.wheel_linear_velocity_right_raw)
                <= 0.08 + 1.0e-9
                for message in messages))
            self.assertTrue(self._stopped(messages))
            sequences = [message.command_seq for message in messages]
            self.assertEqual(sequences, sorted(set(sequences)))

        self.assertGreater(len(self._references), 20)
        self.assertGreater(len(self._controllers), 20)
        self.assertEqual(
            self._references[-1].path_id, "s_curve_1m_bounded")
        self.assertAlmostEqual(
            self._references[-1].load_path_progress_reference,
            1.00, places=6)
        self.assertAlmostEqual(
            self._references[-1].load_path_velocity_reference,
            0.0, places=9)
        self.assertAlmostEqual(
            self._controllers[-1].common_load_velocity_reference,
            0.0, places=9)


if __name__ == "__main__":
    rospy.init_node("test_three_car_unloaded_bounded_pretest")
    rostest.rosrun(
        "multi_agv_control",
        "three_car_unloaded_bounded_pretest",
        ThreeCarUnloadedBoundedPretestTest)
