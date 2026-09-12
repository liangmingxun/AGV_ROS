#!/usr/bin/env python3
"""Isolated master only: fake motion inputs, no chassis command publishers."""
import math
import time
import unittest

import rospy
import rostest
from agv_msgs.msg import CooperativeState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64


class FusedMotionStateTest(unittest.TestCase):
    def test_camera_correction_is_not_reverse_motion(self):
        states = []
        subscriber = rospy.Subscriber('/multi_agv/cooperative_state', CooperativeState,
                                      states.append, queue_size=300)
        publishers = [rospy.Publisher('/pose_provider/agv%d/base_motion_fused' % i,
                                      Odometry, queue_size=10) for i in (1, 2, 3)]
        epoch = rospy.Publisher('/vision/aruco/calibration_epoch', UInt64,
                               queue_size=1, latch=True)
        deadline = time.monotonic() + 5
        while not all(p.get_num_connections() for p in publishers) and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(all(p.get_num_connections() for p in publishers))
        epoch.publish(UInt64(123))
        time.sleep(.1)
        offsets = ((.1732050807568877, 0), (-.0866025403784439, .15),
                   (-.0866025403784439, -.15))
        levers = (-.01783, .09908, .09908)
        origin = rospy.Time.now().to_sec()

        def send(correction=0, invalid=False):
            stamp = rospy.Time.now()
            progress = .1 * (stamp.to_sec() - origin) + correction
            for i, publisher in enumerate(publishers):
                message = Odometry()
                message.header.stamp = stamp
                message.header.frame_id = 'world@0000007b'
                message.child_frame_id = 'agv%d/base_link' % (i + 1)
                # Fleet heading pi/2 exercises velocity rotation at anchoring.
                message.pose.pose.position.x = 1 - offsets[i][1]
                message.pose.pose.position.y = 2 + progress + offsets[i][0] - levers[i]
                message.pose.pose.orientation.z = math.sin(math.pi / 4)
                message.pose.pose.orientation.w = math.cos(math.pi / 4)
                message.twist.twist.linear.x = float('nan') if invalid and i == 0 else .1
                publisher.publish(message)
            time.sleep(.01)

        for _ in range(100):
            send()
        self.assertTrue(states[-1].load_path_state_valid)
        self.assertTrue(all(states[-1].path_state_valid))
        before = states[-1].load_s_actual
        before_stamp = states[-1].load_pose_stamp.to_sec()
        start = len(states)
        for _ in range(25):
            send(-.006)
        checked = states[start:]
        self.assertGreater(len(checked), 15)
        self.assertTrue(all(all(s.path_state_valid) and s.load_path_state_valid for s in checked))
        for state in checked:
            for speed in list(state.s_dot_actual) + [state.load_s_dot_actual]:
                self.assertAlmostEqual(speed, .1, delta=.001)
        # Absolute correction remains observable in progress, not hidden.
        elapsed = states[-1].load_pose_stamp.to_sec() - before_stamp
        self.assertAlmostEqual(states[-1].load_s_actual - before,
                               .1 * elapsed - .006, delta=.0005)
        for _ in range(40):
            send(-.006, invalid=True)
        self.assertFalse(all(states[-1].path_state_valid))
        self.assertIsNotNone(subscriber)


if __name__ == '__main__':
    rospy.init_node('test_fused_motion_state')
    rostest.rosrun('multi_agv_control', 'fused_motion_state', FusedMotionStateTest)
