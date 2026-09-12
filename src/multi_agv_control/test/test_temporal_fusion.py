#!/usr/bin/env python3
"""Fake input only; run on an isolated ROS master. Never sends chassis commands."""
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, UInt64


class TemporalFusionTest(unittest.TestCase):
    def test_backlog_is_not_live_feedback(self):
        outputs, timings, motions = [], [], []
        subscriptions = [
            rospy.Subscriber('/test/agv2/fused', PoseStamped, outputs.append),
            rospy.Subscriber('/pose_provider/agv2/base_motion_fused', Odometry, motions.append),
            rospy.Subscriber('/pose_provider/agv2/fusion_timing', Float64MultiArray, timings.append)]
        odom = rospy.Publisher('/test/agv2/odom', Odometry, queue_size=100)
        camera = rospy.Publisher('/test/agv2/camera', PoseStamped, queue_size=10)
        confidence = rospy.Publisher('/test/agv2/confidence', Float64, queue_size=10)
        epoch = rospy.Publisher('/vision/aruco/calibration_epoch', UInt64, queue_size=1, latch=True)
        deadline = time.monotonic()+5
        while (not odom.get_num_connections() or not camera.get_num_connections()) and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(odom.get_num_connections())
        epoch.publish(UInt64(123))
        confidence.publish(Float64(1))
        time.sleep(.1)
        origin = rospy.Time.now().to_sec()

        def send(stamp, visual=True):
            confidence.publish(Float64(1))
            if visual:
                c = PoseStamped()
                c.header.stamp = rospy.Time.from_sec(stamp-.005)
                c.header.frame_id = 'world@123'
                c.pose.orientation.w = 1
                c.pose.position.x = .1*(stamp-.005-origin)
                camera.publish(c)
            o = Odometry()
            o.header.stamp = rospy.Time.from_sec(stamp)
            o.pose.pose.orientation.w = 1
            o.pose.pose.position.x = .1*(stamp-origin)
            o.twist.twist.linear.x = .1
            odom.publish(o)

        for _ in range(30):
            send(rospy.Time.now().to_sec())
            time.sleep(.01)
        self.assertGreater(len(outputs), 10)
        self.assertGreater(len(motions), 10)
        self.assertTrue(all(m.twist.twist.linear.x == .1 for m in motions))
        self.assertTrue(all(m.child_frame_id == 'agv2/base_link' for m in motions))
        self.assertAlmostEqual(outputs[-1].pose.position.x,
                               .1*(outputs[-1].header.stamp.to_sec()-origin), delta=.003)
        last_stamp = outputs[-1].header.stamp.to_sec()
        time.sleep(.24)
        before = len(outputs)
        # Deliver consecutive old odometry; its deltas must be retained but
        # its output must not be sold downstream as a new live observation.
        for i in range(1, 11):
            send(last_stamp+.01*i, visual=False)
            time.sleep(.002)
        time.sleep(.025)
        self.assertEqual(len(outputs), before)
        self.assertTrue(any(t.data[5] == 0 for t in timings))
        for _ in range(20):
            send(rospy.Time.now().to_sec())
            time.sleep(.01)
        self.assertGreater(len(outputs), before+5)
        self.assertAlmostEqual(outputs[-1].pose.position.x,
                               .1*(outputs[-1].header.stamp.to_sec()-origin), delta=.003)
        self.assertTrue(all(t.data[6] >= 0 for t in timings))
        # Non-increasing odom must not produce another fused observation.
        before = len(outputs)
        send(last_stamp, visual=False)
        time.sleep(.05)
        self.assertEqual(len(outputs), before)
        self.assertEqual(len(subscriptions), 3)


if __name__ == '__main__':
    rospy.init_node('test_temporal_fusion')
    rostest.rosrun('multi_agv_control', 'temporal_fusion', TemporalFusionTest)
