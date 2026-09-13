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
        # Small initial deformation, including one robot behind its s=0 pose.
        offsets = ((.1632050807568877, 0), (-.0866025403784439, .15),
                   (-.0766025403784439, -.15))
        levers = (-.01783, .09908, .09908)
        origin = rospy.Time.now().to_sec()

        def send(correction=0, invalid=False, skip_robot2=False, speed=.1):
            stamp = rospy.Time.now()
            progress = speed * (stamp.to_sec() - origin) + correction
            for i, publisher in enumerate(publishers):
                if skip_robot2 and i == 1:
                    continue
                message = Odometry()
                message.header.stamp = stamp
                message.header.frame_id = 'world@0000007b'
                message.child_frame_id = 'agv%d/base_link' % (i + 1)
                # Fleet heading pi/2 exercises velocity rotation at anchoring.
                message.pose.pose.position.x = 1 - offsets[i][1]
                message.pose.pose.position.y = 2 + progress + offsets[i][0] - levers[i]
                message.pose.pose.orientation.z = math.sin(math.pi / 4)
                message.pose.pose.orientation.w = math.cos(math.pi / 4)
                message.twist.twist.linear.x = float('nan') if invalid and i == 0 else speed
                publisher.publish(message)
            time.sleep(.01)

        for _ in range(25):
            send(speed=0)
        self.assertTrue(all(states[-1].path_state_valid))
        self.assertLess(states[-1].s_actual[0], -.005)
        initial_signed_progress = states[-1].s_actual[0]
        origin = rospy.Time.now().to_sec()
        for _ in range(100):
            send()
        self.assertGreater(states[-1].s_actual[0] - initial_signed_progress, .08)
        self.assertTrue(states[-1].load_path_state_valid)
        self.assertTrue(all(states[-1].path_state_valid))
        before = states[-1].load_s_actual
        before_control_stamp = states[-1].header.stamp.to_sec()
        before_robot_x = states[-1].robot_pose[0].x
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
        # Compare at control time: both endpoints are projected estimates.
        elapsed = states[-1].header.stamp.to_sec() - before_control_stamp
        self.assertAlmostEqual(states[-1].load_s_actual - before,
                               .1 * elapsed - .006, delta=.0005)
        robot2_stamp = states[-1].robot_pose_stamp[1].to_sec()
        for _ in range(6):
            send(-.006, skip_robot2=True)
        self.assertTrue(all(states[-1].path_state_valid))
        self.assertLessEqual(states[-1].robot_pose_stamp[1].to_sec(), robot2_stamp + .015)
        for _ in range(10):
            send(-.006)
        # Only previously accepted live samples may be propagated: packets
        # arriving already older than 50 ms remain rejected at ingress.
        time.sleep(.08)
        state = states[-1]
        self.assertTrue(all(state.path_state_valid))
        source_age = (state.header.stamp - state.robot_pose_stamp[0]).to_sec()
        self.assertGreater(source_age, .075)
        self.assertLess(source_age, .12)
        # Coordinates advance to control time, while evidence keeps its old stamp.
        expected = before_robot_x + .1 * (state.header.stamp.to_sec() - before_control_stamp) - .006
        self.assertAlmostEqual(state.robot_pose[0].x, expected, delta=.002)
        # Exceeding the shorter projection horizon is not localization loss:
        # retain the still-fresh synchronized camera truth without extrapolation.
        time.sleep(.09)
        self.assertTrue(all(states[-1].path_state_valid))
        source_age = (states[-1].header.stamp -
                      states[-1].robot_pose_stamp[0]).to_sec()
        self.assertGreater(source_age, .12)
        self.assertLess(source_age, .22)
        # The independent freshness gate remains final and must invalidate a
        # genuinely stale camera snapshot.
        time.sleep(.07)
        self.assertFalse(all(states[-1].path_state_valid))
        for _ in range(40):
            send(-.006, invalid=True)
        self.assertFalse(all(states[-1].path_state_valid))
        self.assertIsNotNone(subscriber)


if __name__ == '__main__':
    rospy.init_node('test_fused_motion_state')
    rostest.rosrun('multi_agv_control', 'fused_motion_state', FusedMotionStateTest)
