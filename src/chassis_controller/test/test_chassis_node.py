#!/usr/bin/env python3
import math
import unittest

import rospy
import rostest
from agv_msgs.msg import CapabilityReport, ChassisCommand, ChassisFeedback
from std_srvs.srv import Trigger


class ChassisNodeTest(unittest.TestCase):
    def test_fake_transport_echoes_through_limiter(self):
        command_pub = rospy.Publisher("chassis_command", ChassisCommand,
                                      queue_size=1, latch=False)
        feedback = []
        capability = []
        rospy.Subscriber("chassis_feedback", ChassisFeedback, feedback.append,
                         queue_size=1)
        rospy.Subscriber("capability_report", CapabilityReport, capability.append,
                         queue_size=1)
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while command_pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.02)

        command = ChassisCommand(robot_id=1, command_seq=1,
                                 wheel_linear_velocity_left_raw=0.080072,
                                 wheel_linear_velocity_right_raw=0.080072)
        for _ in range(20):
            command.header.stamp = rospy.Time.now()
            command_pub.publish(command)
            rospy.sleep(0.02)
            if feedback and feedback[-1].command_seq_applied == 1:
                break

        self.assertTrue(feedback)
        self.assertTrue(capability)
        self.assertEqual(1, feedback[-1].robot_id)
        self.assertEqual(1, feedback[-1].command_seq_applied)
        self.assertAlmostEqual(
            0.08, capability[-1].max_wheel_linear_velocity_left,
            delta=1e-12)
        self.assertGreater(
            feedback[-1].wheel_linear_velocity_left_raw,
            capability[-1].max_wheel_linear_velocity_left)
        self.assertLessEqual(abs(feedback[-1].wheel_linear_velocity_left_applied),
                             capability[-1].max_wheel_linear_velocity_left)
        self.assertTrue(math.isfinite(
            feedback[-1].wheel_linear_velocity_left_actual))
        self.assertLessEqual(
            abs(feedback[-1].wheel_linear_velocity_left_actual),
            abs(feedback[-1].wheel_linear_velocity_left_applied) + 1e-12)
        self.assertTrue(feedback[-1].speed_limit_active_left)

        sample_deadline = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < sample_deadline and (
                len(feedback) < 10 or len(capability) < 10):
            rospy.sleep(0.02)
        feedback_sequences = [message.feedback_seq for message in feedback[-10:]]
        capability_sequences = [message.capability_seq for message in capability[-10:]]
        self.assertTrue(all(b == a + 1 for a, b in zip(
            feedback_sequences, feedback_sequences[1:])))
        self.assertTrue(all(b == a + 1 for a, b in zip(
            capability_sequences, capability_sequences[1:])))

        motion_deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < motion_deadline and (
                not feedback or feedback[-1].odom.pose.pose.position.x < 0.002):
            command.header.stamp = rospy.Time.now()
            command_pub.publish(command)
            rospy.sleep(0.02)

        watchdog_deadline = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < watchdog_deadline and (
                not feedback
                or abs(feedback[-1].wheel_linear_velocity_left_raw) >= 0.01
                or abs(feedback[-1].wheel_linear_velocity_left_actual) >= 0.01):
            rospy.sleep(0.02)
        self.assertLess(
            abs(feedback[-1].wheel_linear_velocity_left_raw), 0.01,
            "command watchdog did not replace a stale command with zero")
        self.assertLess(
            abs(feedback[-1].wheel_linear_velocity_left_actual), 0.01,
            "fake chassis did not stop after command watchdog expiration")

        stop = ChassisCommand(robot_id=1, command_seq=2,
                              wheel_linear_velocity_left_raw=0.0,
                              wheel_linear_velocity_right_raw=0.0)
        stop_deadline = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < stop_deadline:
            stop.header.stamp = rospy.Time.now()
            command_pub.publish(stop)
            rospy.sleep(0.02)
            if feedback and abs(feedback[-1].wheel_linear_velocity_left_actual) < 0.01:
                break

        rospy.wait_for_service("reset_odometry", timeout=2.0)
        reset_response = rospy.ServiceProxy("reset_odometry", Trigger)()
        self.assertTrue(reset_response.success, reset_response.message)
        reset_deadline = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < reset_deadline and (
                not feedback or abs(feedback[-1].odom.pose.pose.position.x) > 1e-6):
            rospy.sleep(0.02)
        self.assertAlmostEqual(0.0, feedback[-1].odom.pose.pose.position.x,
                               delta=1e-6)


if __name__ == "__main__":
    rospy.init_node("test_chassis_node")
    rostest.rosrun("chassis_controller", "test_chassis_node", ChassisNodeTest)
