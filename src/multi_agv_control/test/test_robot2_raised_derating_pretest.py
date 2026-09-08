#!/usr/bin/env python3
import threading
import unittest

import rospy
import rostest
from agv_msgs.msg import (CapabilityReport, ChassisCommand, ChassisFeedback,
                          DeratingCommand, ExperimentState)
from nav_msgs.msg import Odometry


class Robot2RaisedDeratingPretestTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.odom = None
        self.start_x = None
        self.derating = []
        self.capability = []
        self.feedback = []
        self.experiment = []
        self.command_publisher = rospy.Publisher(
            "/agv2/chassis_command", ChassisCommand, queue_size=1)
        rospy.Subscriber("/agv2/odom", Odometry, self._odom, queue_size=100)
        rospy.Subscriber("/agv2/derating_command", DeratingCommand,
                         self._derating, queue_size=200)
        rospy.Subscriber("/agv2/capability_report", CapabilityReport,
                         lambda message: self._append(self.capability, message),
                         queue_size=500)
        rospy.Subscriber("/agv2/chassis_feedback", ChassisFeedback,
                         lambda message: self._append(self.feedback, message),
                         queue_size=500)
        rospy.Subscriber(
            "/agv2/raised_derating_pretest/experiment_state", ExperimentState,
            lambda message: self._append(self.experiment, message),
            queue_size=200)

    def _append(self, target, message):
        with self.lock:
            target.append(message)
            del target[:-2000]

    def _odom(self, message):
        with self.lock:
            self.odom = message

    def _derating(self, message):
        with self.lock:
            progress = None
            if self.odom is not None and self.start_x is not None:
                progress = self.odom.pose.pose.position.x - self.start_x
            self.derating.append((message, progress))
            del self.derating[:-1000]

    def _command(self, sequence, speed):
        message = ChassisCommand()
        message.header.stamp = rospy.Time.now()
        message.robot_id = 2
        message.command_seq = sequence
        message.control_mode = 1
        message.linear_velocity_reference = speed
        message.wheel_linear_velocity_left_raw = speed
        message.wheel_linear_velocity_right_raw = speed
        message.experiment_id = "robot2_raised_derating_rostest"
        message.method_id = "BOUNDED_RAISED_STRAIGHT"
        self.command_publisher.publish(message)

    def test_actual_robot2_odom_triggers_and_restores(self):
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        rate = rospy.Rate(100)
        while rospy.Time.now() < deadline:
            with self.lock:
                ready = self.odom is not None
            if ready and self.command_publisher.get_num_connections() > 0:
                break
            rate.sleep()
        self.assertGreater(self.command_publisher.get_num_connections(), 0)
        with self.lock:
            self.assertIsNotNone(self.odom)
            self.start_x = self.odom.pose.pose.position.x
        # Allow the dedicated node to observe this test's derating subscription,
        # establish its odometry anchor, and publish the initial nominal target.
        rospy.sleep(0.5)

        sequence = 60000
        motion_deadline = rospy.Time.now() + rospy.Duration(14.0)
        while rospy.Time.now() < motion_deadline:
            sequence += 1
            self._command(sequence, 0.05)
            rate.sleep()
        for _ in range(10):
            sequence += 1
            self._command(sequence, 0.0)
            rate.sleep()

        finish_deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < finish_deadline:
            with self.lock:
                phases = [value.phase for value in self.experiment]
                stopped = (self.feedback and
                           abs(self.feedback[-1].wheel_linear_velocity_left_actual) < 0.01 and
                           abs(self.feedback[-1].wheel_linear_velocity_right_actual) < 0.01)
            if 6 in phases and stopped:
                break
            rate.sleep()

        with self.lock:
            derating = list(self.derating)
            capability = list(self.capability)
            feedback = list(self.feedback)
            phases = [value.phase for value in self.experiment]
        active = [(message, progress) for message, progress in derating
                  if message.active]
        self.assertTrue(active)
        first_active_sequence = active[0][0].command_seq
        first_active_progress = next(
            progress for message, progress in active if progress is not None)
        restored = [(message, progress) for message, progress in derating
                    if (not message.active and
                        message.command_seq > first_active_sequence and
                        progress is not None)]
        self.assertTrue(restored)
        self.assertGreaterEqual(first_active_progress, 0.28)
        self.assertLessEqual(first_active_progress, 0.34)
        self.assertGreaterEqual(restored[0][1], 0.58)
        self.assertLessEqual(restored[0][1], 0.65)
        self.assertTrue(any(value.derating_active for value in capability))
        self.assertLess(min(value.derating_ratio for value in capability), 0.45)
        derated_limits = [
            value.max_wheel_linear_velocity_left for value in capability
            if value.derating_active and value.derating_ratio <= 0.45]
        self.assertTrue(derated_limits)
        self.assertAlmostEqual(min(derated_limits), 0.06, delta=0.003)
        self.assertFalse(capability[-1].derating_active)
        self.assertGreater(capability[-1].derating_ratio, 0.99)
        self.assertAlmostEqual(
            capability[-1].max_wheel_linear_velocity_left, 0.15, delta=0.003)
        self.assertAlmostEqual(
            capability[-1].max_wheel_linear_velocity_right, 0.15, delta=0.003)
        self.assertTrue(feedback)
        self.assertLess(abs(feedback[-1].wheel_linear_velocity_left_actual), 0.01)
        self.assertLess(abs(feedback[-1].wheel_linear_velocity_right_actual), 0.01)
        for phase in (3, 4, 5, 6):
            self.assertIn(phase, phases)


if __name__ == "__main__":
    rospy.init_node("test_robot2_raised_derating_pretest")
    rostest.rosrun(
        "multi_agv_control", "test_robot2_raised_derating_pretest",
        Robot2RaisedDeratingPretestTest)
