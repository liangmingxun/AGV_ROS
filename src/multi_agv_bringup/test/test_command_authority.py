#!/usr/bin/env python3
import unittest

import rosgraph
import rospy
import rostest
from agv_msgs.msg import ChassisCommand


class CommandAuthorityTest(unittest.TestCase):
    def test_one_command_publisher_and_no_global_velocity_topics(self):
        publishers = [
            rospy.Publisher(f"/agv{i}/chassis_command", ChassisCommand,
                            queue_size=1, latch=False)
            for i in range(1, 4)
        ]
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline and any(
                publisher.get_num_connections() == 0 for publisher in publishers):
            rospy.sleep(0.05)

        system_publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        by_topic = dict(system_publishers)
        for index in range(1, 4):
            topic = f"/agv{index}/chassis_command"
            self.assertEqual(1, len(by_topic.get(topic, [])), topic)
        self.assertNotIn("/cmd_vel", by_topic)
        self.assertNotIn("/odom", by_topic)


if __name__ == "__main__":
    rospy.init_node("test_command_authority")
    rostest.rosrun("multi_agv_bringup", "command_authority",
                   CommandAuthorityTest)
