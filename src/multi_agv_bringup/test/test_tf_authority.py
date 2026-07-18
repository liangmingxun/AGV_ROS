#!/usr/bin/env python3
import collections
import unittest

import rospy
import rostest
from tf2_msgs.msg import TFMessage


class TfAuthorityTest(unittest.TestCase):
    def test_each_frozen_edge_has_one_publisher(self):
        authorities = collections.defaultdict(set)

        def collect(message):
            caller = message._connection_header.get("callerid", "unknown")
            for transform in message.transforms:
                edge = (transform.header.frame_id, transform.child_frame_id)
                authorities[edge].add(caller)

        rospy.Subscriber("/tf", TFMessage, collect, queue_size=100)
        rospy.Subscriber("/tf_static", TFMessage, collect, queue_size=100)
        deadline = rospy.Time.now() + rospy.Duration(6.0)
        expected = set()
        for index in range(1, 4):
            expected.update({
                (f"agv{index}/odom", f"agv{index}/base_link"),
                (f"agv{index}/base_link", f"agv{index}/imu_link"),
                (f"agv{index}/base_link", f"agv{index}/support_link"),
            })
        while rospy.Time.now() < deadline and not expected.issubset(authorities):
            rospy.sleep(0.05)

        for edge in expected:
            self.assertIn(edge, authorities, edge)
            self.assertEqual(1, len(authorities[edge]),
                             f"multiple TF publishers for {edge}: {authorities[edge]}")


if __name__ == "__main__":
    rospy.init_node("test_tf_authority")
    rostest.rosrun("multi_agv_bringup", "tf_authority", TfAuthorityTest)
