#!/usr/bin/env python3
import math
import threading
import unittest

import rosgraph
import rospy
import rostest
from agv_msgs.msg import CooperativeState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class CameraPoseAdapterTest(unittest.TestCase):
    OFFSETS = (
        (0.1732050807568877, 0.0),
        (-0.0866025403784439, 0.1500000000000000),
        (-0.0866025403784439, -0.1500000000000000),
    )
    BASE_TO_SUPPORT_X = (-0.01783, 0.09908, 0.09908)

    def setUp(self):
        self._lock = threading.Lock()
        self._states = []
        self._raw_agv1 = []
        self._filtered_agv1 = []
        self._state_subscriber = rospy.Subscriber(
            "/multi_agv/cooperative_state", CooperativeState,
            self._receive_state, queue_size=200)
        self._raw_subscriber = rospy.Subscriber(
            "/pose_provider/agv1/base_pose_raw", PoseStamped,
            lambda message: self._raw_agv1.append(message), queue_size=20)
        self._filtered_subscriber = rospy.Subscriber(
            "/pose_provider/agv1/base_pose_filtered", PoseStamped,
            lambda message: self._filtered_agv1.append(message),
            queue_size=20)
        names = ("agv1", "agv2", "agv3", "load")
        self._pose_publishers = [
            rospy.Publisher(
                "/camera/world/{}_tag_pose".format(name),
                PoseStamped, queue_size=10)
            for name in names
        ]
        self._confidence_publishers = [
            rospy.Publisher(
                "/camera/world/{}_confidence".format(name),
                Float64, queue_size=10)
            for name in names
        ]

    def _receive_state(self, message):
        with self._lock:
            self._states.append(message)
            self._states = self._states[-500:]

    def _latest(self):
        with self._lock:
            return self._states[-1] if self._states else None

    def _wait_for_connections(self):
        deadline = rospy.Time.now() + rospy.Duration(6.0)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if (all(pub.get_num_connections() == 1
                    for pub in self._pose_publishers) and
                    all(pub.get_num_connections() == 1
                        for pub in self._confidence_publishers) and
                    self._state_subscriber.get_num_connections() == 1):
                return
            rospy.sleep(0.02)
        self.fail("camera adapter connections were not established")

    @staticmethod
    def _center_sample(xi):
        amplitude = 0.05
        wave_number = 2.0 * math.pi
        y = amplitude * math.sin(wave_number * xi)
        slope = amplitude * wave_number * math.cos(wave_number * xi)
        second = -amplitude * wave_number * wave_number * math.sin(
            wave_number * xi)
        speed = math.hypot(1.0, slope)
        tangent = (1.0 / speed, slope / speed)
        normal = (-tangent[1], tangent[0])
        curvature = second / (speed ** 3)
        return (xi, y), tangent, normal, curvature

    @classmethod
    def _support_sample(cls, xi, offset):
        center, tangent, normal, curvature = cls._center_sample(xi)
        qt, qn = offset
        position = (center[0] + qt * tangent[0] + qn * normal[0],
                    center[1] + qt * tangent[1] + qn * normal[1])
        dx = (1.0 - curvature * qn) * tangent[0] + \
            curvature * qt * normal[0]
        dy = (1.0 - curvature * qn) * tangent[1] + \
            curvature * qt * normal[1]
        return position, math.atan2(dy, dx)

    @staticmethod
    def _pose(stamp, target_x, target_y, yaw, tag_offset_x, epoch_token):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = "world@{:08x}".format(epoch_token)
        message.pose.position.x = target_x - tag_offset_x * math.cos(yaw)
        message.pose.position.y = target_y - tag_offset_x * math.sin(yaw)
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        return message

    def _publish(self, xi, indices=(0, 1, 2, 3), confidence=0.95,
                 epoch_token=1):
        stamp = rospy.Time.now()
        center, tangent, _, _ = self._center_sample(xi)
        expected_bases = []
        for index in range(3):
            support, yaw = self._support_sample(xi, self.OFFSETS[index])
            base_x = support[0] - self.BASE_TO_SUPPORT_X[index] * math.cos(yaw)
            base_y = support[1] - self.BASE_TO_SUPPORT_X[index] * math.sin(yaw)
            expected_bases.append((base_x, base_y, yaw))
            if index in indices:
                self._confidence_publishers[index].publish(
                    Float64(data=confidence))
                self._pose_publishers[index].publish(
                    self._pose(
                        stamp, base_x, base_y, yaw, 0.02, epoch_token))
        if 3 in indices:
            load_yaw = math.atan2(tangent[1], tangent[0])
            self._confidence_publishers[3].publish(
                Float64(data=confidence))
            self._pose_publishers[3].publish(
                self._pose(
                    stamp, center[0], center[1], load_yaw, 0.03,
                    epoch_token))
        return stamp, expected_bases

    def test_camera_transform_stamps_and_synchronized_dropout(self):
        self._wait_for_connections()
        rate = rospy.Rate(50)
        last_stamp = None
        expected = None
        for step in range(60):
            last_stamp, expected = self._publish(0.05 + 0.001 * step)
            rate.sleep()

        state = self._latest()
        self.assertIsNotNone(state)
        self.assertEqual(
            list(state.robot_localization_source),
            [CooperativeState.SOURCE_CAMERA] * 3)
        self.assertEqual(list(state.robot_pose_valid), [True] * 3)
        self.assertEqual(list(state.path_state_valid), [True] * 3)
        self.assertTrue(state.load_pose_valid)
        self.assertTrue(state.load_path_state_valid)
        self.assertEqual(state.load_localization_source,
                         CooperativeState.SOURCE_CAMERA)
        self.assertGreaterEqual(state.header.stamp, max(state.robot_pose_stamp))
        for index in range(3):
            self.assertAlmostEqual(
                state.robot_pose_stamp[index].to_sec(),
                last_stamp.to_sec(), delta=0.03)
            self.assertAlmostEqual(
                state.robot_pose[index].x, expected[index][0], delta=0.003)
            self.assertAlmostEqual(
                state.robot_pose[index].y, expected[index][1], delta=0.003)
        self.assertTrue(self._raw_agv1)
        self.assertEqual(
            self._raw_agv1[-1].header.frame_id, "world@00000001")

        # Drop only AGV2 for longer than maximum_state_age. The estimator's
        # three-robot synchronized-snapshot contract invalidates all robot
        # states (so formal control fails zero); the separately observed load
        # remains usable.
        for step in range(18):
            self._publish(0.11 + 0.001 * step, indices=(0, 2, 3))
            rate.sleep()
        state = self._latest()
        self.assertEqual(list(state.robot_pose_valid), [False] * 3)
        self.assertEqual(list(state.support_pose_valid), [False] * 3)
        self.assertEqual(list(state.path_state_valid), [False] * 3)
        self.assertEqual(
            list(state.robot_localization_source),
            [CooperativeState.SOURCE_UNKNOWN] * 3)
        self.assertTrue(state.load_pose_valid)
        self.assertTrue(state.load_path_state_valid)

        # A low-confidence AGV2 detection cannot revive the stale stream.
        self._publish(0.13, indices=(1,), confidence=0.1)
        rospy.sleep(0.05)
        self.assertFalse(self._latest().robot_pose_valid[1])

        # A new world-frame calibration epoch must not be blended with the
        # previous filtered trajectory.
        _, epoch_expected = self._publish(
            0.20, indices=(0,), epoch_token=2)
        rospy.sleep(0.05)
        self.assertEqual(
            self._raw_agv1[-1].header.frame_id, "world@00000002")
        self.assertTrue(self._filtered_agv1)
        self.assertEqual(
            self._filtered_agv1[-1].header.frame_id, "world@00000002")
        self.assertAlmostEqual(
            self._raw_agv1[-1].pose.position.x,
            epoch_expected[0][0], delta=1.0e-6)
        self.assertAlmostEqual(
            self._filtered_agv1[-1].pose.position.x,
            epoch_expected[0][0], delta=1.0e-6)

        publishers, _, _ = rosgraph.Master(
            rospy.get_name()).getSystemState()
        tf_publishers = {
            node for topic, nodes in publishers
            if topic in ("/tf", "/tf_static") for node in nodes
        }
        self.assertNotIn("/pose_provider", tf_publishers)


if __name__ == "__main__":
    rospy.init_node("test_camera_pose_adapter")
    rostest.rosrun(
        "multi_agv_control", "camera_pose_adapter",
        CameraPoseAdapterTest)
