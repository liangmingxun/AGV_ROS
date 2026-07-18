#!/usr/bin/env python3
import unittest

import rospy
import rostest
from agv_msgs.msg import ChassisCommand, ChassisFeedback


class NamespaceIsolationTest(unittest.TestCase):
    def test_each_command_reaches_only_its_chassis(self):
        desired = {1: 0.1, 2: 0.2, 3: 0.3}
        feedback = {1: [], 2: [], 3: []}
        publishers = {}
        for index in desired:
            publishers[index] = rospy.Publisher(
                f"/agv{index}/chassis_command", ChassisCommand,
                queue_size=1, latch=False)
            rospy.Subscriber(
                f"/agv{index}/chassis_feedback", ChassisFeedback,
                lambda message, i=index: feedback[i].append(message), queue_size=1)

        deadline = rospy.Time.now() + rospy.Duration(8.0)
        sequence = 1
        while rospy.Time.now() < deadline:
            for index, speed in desired.items():
                message = ChassisCommand(robot_id=index, command_seq=sequence,
                                         wheel_linear_velocity_left_raw=speed,
                                         wheel_linear_velocity_right_raw=speed)
                message.header.stamp = rospy.Time.now()
                publishers[index].publish(message)
            rospy.sleep(0.05)
            if all(feedback[index] and
                   abs(feedback[index][-1].wheel_linear_velocity_left_actual - speed)
                   < 0.02 for index, speed in desired.items()):
                break
            sequence += 1

        for index, speed in desired.items():
            self.assertTrue(feedback[index], f"agv{index} produced no feedback")
            latest = feedback[index][-1]
            self.assertEqual(index, latest.robot_id)
            self.assertAlmostEqual(
                speed, latest.wheel_linear_velocity_left_actual, delta=0.02)
            for other_speed in desired.values():
                if other_speed != speed:
                    self.assertGreater(
                        abs(latest.wheel_linear_velocity_left_actual - other_speed), 0.04)


if __name__ == "__main__":
    rospy.init_node("test_namespace_isolation")
    rostest.rosrun("multi_agv_bringup", "namespace_isolation",
                   NamespaceIsolationTest)
