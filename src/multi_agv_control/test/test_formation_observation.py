#!/usr/bin/env python3
"""Fake measurements only, on an isolated ROS master. No commands published."""
import math
import time
import unittest
import rospy
import rostest
from agv_msgs.msg import CooperativeState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64


class ObservationTest(unittest.TestCase):
    def test_deformation_observable_and_range_latched(self):
        states = []
        sub = rospy.Subscriber('/multi_agv/cooperative_state', CooperativeState,
                               states.append, queue_size=100)
        pubs = [rospy.Publisher('/pose_provider/agv%d/base_motion_fused' % i,
                                Odometry, queue_size=10) for i in (1, 2, 3)]
        epoch = rospy.Publisher('/vision/aruco/calibration_epoch', UInt64,
                               queue_size=1, latch=True)
        deadline = time.monotonic() + 5
        while not all(p.get_num_connections() for p in pubs):
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.01)
        epoch.publish(UInt64(123))
        time.sleep(.1)
        offsets = ((.1732050807568877, 0), (-.0866025403784439, .15),
                   (-.0866025403784439, -.15))
        levers = (-.01783, .09908, .09908)

        def send_for(displacement, seconds=.7, lateral=0):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                stamp = rospy.Time.now()
                for i, pub in enumerate(pubs):
                    m = Odometry()
                    m.header.stamp = stamp
                    m.header.frame_id = 'world@0000007b'
                    m.child_frame_id = 'agv%d/base_link' % (i+1)
                    m.pose.pose.position.x = 1-offsets[i][1]+(lateral if i == 1 else 0)
                    m.pose.pose.position.y = 2+offsets[i][0]-levers[i]+(displacement if i == 1 else 0)
                    m.pose.pose.orientation.z = math.sin(math.pi/4)
                    m.pose.pose.orientation.w = math.cos(math.pi/4)
                    pub.publish(m)
                time.sleep(.01)

        send_for(0)
        self.assertTrue(states[-1].load_pose_valid)
        send_for(.30)
        self.assertTrue(states[-1].load_pose_valid)
        self.assertTrue(all(states[-1].path_state_valid))
        # >150 mm real off-path deviation is not localization loss.
        send_for(.30, lateral=.20)
        self.assertTrue(states[-1].load_pose_valid)
        self.assertTrue(all(states[-1].path_state_valid))
        send_for(.90)
        self.assertFalse(states[-1].load_pose_valid)
        send_for(0)
        self.assertFalse(states[-1].load_pose_valid)
        sub.unregister()


if __name__ == '__main__':
    rospy.init_node('formation_observation_test')
    rostest.rosrun('multi_agv_control', 'formation_observation', ObservationTest)
