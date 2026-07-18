#!/usr/bin/env python3
import unittest

import rospy
import rostest
from agv_msgs.msg import CapabilityReport, ChassisCommand, ChassisFeedback


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
                                 wheel_linear_velocity_left_raw=0.2,
                                 wheel_linear_velocity_right_raw=0.2)
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
        self.assertLessEqual(abs(feedback[-1].wheel_linear_velocity_left_applied),
                             capability[-1].max_wheel_linear_velocity_left)


if __name__ == "__main__":
    rospy.init_node("test_chassis_node")
    rostest.rosrun("chassis_controller", "test_chassis_node", ChassisNodeTest)
