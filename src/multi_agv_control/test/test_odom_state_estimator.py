#!/usr/bin/env python3
import math
import threading
import unittest

import rospy
import rostest
from agv_msgs.msg import CooperativeState, PathReference
from nav_msgs.msg import Odometry


class OdomStateEstimatorTest(unittest.TestCase):
    OFFSETS = (
        (0.230940107675850, 0.0),
        (-0.115470053837925, 0.20),
        (-0.115470053837925, -0.20),
    )
    WORLD_TO_ODOM = (
        (0.237333702114270, 0.074560581501146, 0.304395797364615),
        (-0.153094727127932, 0.161541278437113, 0.304395797364615),
        (-0.033208005692254, -0.220070008114273, 0.304395797364615),
    )
    BASE_TO_SUPPORT_X = -0.01783

    def setUp(self):
        self._lock = threading.Lock()
        self._messages = []
        self._state_subscriber = rospy.Subscriber(
            "/multi_agv/cooperative_state", CooperativeState,
            self._receive_state, queue_size=200)
        self._odom_publishers = [
            rospy.Publisher("/agv{}/odom".format(index), Odometry,
                            queue_size=10)
            for index in (1, 2, 3)
        ]
        self._reference_publisher = rospy.Publisher(
            "/multi_agv/path_reference", PathReference, queue_size=1)

    def _receive_state(self, message):
        with self._lock:
            self._messages.append(message)
            self._messages = self._messages[-1000:]

    def _wait_for_connections(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if (all(pub.get_num_connections() > 0
                    for pub in self._odom_publishers) and
                    self._reference_publisher.get_num_connections() == 0 and
                    self._state_subscriber.get_num_connections() > 0):
                return
            rospy.sleep(0.02)
        self.fail("path_state_estimator did not establish odometry connections")

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
        # dp_i/ds = (1-kappa*q_n)*t + kappa*q_t*n
        dx = (1.0 - curvature * qn) * tangent[0] + \
             curvature * qt * normal[0]
        dy = (1.0 - curvature * qn) * tangent[1] + \
             curvature * qt * normal[1]
        return position, math.atan2(dy, dx)

    def _publish_robot(self, index, xi, stamp):
        support, yaw = self._support_sample(xi, self.OFFSETS[index])
        base_x = support[0] - self.BASE_TO_SUPPORT_X * math.cos(yaw)
        base_y = support[1] - self.BASE_TO_SUPPORT_X * math.sin(yaw)
        transform_x, transform_y, transform_yaw = self.WORLD_TO_ODOM[index]
        dx = base_x - transform_x
        dy = base_y - transform_y
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        odom_x = cosine * dx + sine * dy
        odom_y = -sine * dx + cosine * dy
        odom_yaw = yaw - transform_yaw
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "agv{}/odom".format(index + 1)
        message.child_frame_id = "agv{}/base_link".format(index + 1)
        message.pose.pose.position.x = odom_x
        message.pose.pose.position.y = odom_y
        message.pose.pose.orientation.z = math.sin(0.5 * odom_yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * odom_yaw)
        self._odom_publishers[index].publish(message)

    def _latest(self):
        with self._lock:
            return self._messages[-1] if self._messages else None

    def test_three_robot_odometry_and_independent_staleness(self):
        self._wait_for_connections()
        contradictory = PathReference()
        contradictory.load_path_progress_reference = 1.9

        observed = [[], [], []]
        rate = rospy.Rate(50)
        for step in range(80):
            stamp = rospy.Time.now()
            xi = 0.03 + 0.002 * step
            for index in range(3):
                self._publish_robot(index, xi, stamp)
            self._reference_publisher.publish(contradictory)
            rate.sleep()
            state = self._latest()
            if state and all(state.path_state_valid):
                for index in range(3):
                    observed[index].append(state.s_actual[index])

        state = self._latest()
        self.assertIsNotNone(state)
        self.assertEqual(list(state.robot_localization_source),
                         [CooperativeState.SOURCE_ODOM] * 3)
        self.assertEqual(list(state.robot_pose_valid), [True] * 3)
        self.assertEqual(list(state.support_pose_valid), [True] * 3)
        self.assertEqual(list(state.path_state_valid), [True] * 3)
        self.assertTrue(state.load_pose_valid)
        self.assertEqual(state.load_localization_source,
                         CooperativeState.SOURCE_ODOM)
        self.assertTrue(state.load_path_state_valid)
        self.assertLess(state.load_s_actual, 0.5)
        for progress in observed:
            self.assertGreater(len(progress), 20)
            self.assertTrue(all(new + 1.0e-7 >= old
                                for old, new in zip(progress, progress[1:])))

        # AGV3 now arrives 40 ms behind AGV1/2 while retaining its original
        # measurement stamp. The estimator must select matching history from
        # AGV1/2 instead of combining three unrelated latest-arrival samples.
        delayed_agv3 = []
        synchronized_states = []
        for step in range(70):
            stamp = rospy.Time.now()
            xi = 0.19 + 0.001 * step
            self._publish_robot(0, xi, stamp)
            self._publish_robot(1, xi, stamp)
            delayed_agv3.append((xi, stamp))
            if len(delayed_agv3) > 2:
                delayed_xi, delayed_stamp = delayed_agv3.pop(0)
                self._publish_robot(2, delayed_xi, delayed_stamp)
            rate.sleep()
            state = self._latest()
            if step >= 10 and state:
                synchronized_states.append(state)

        self.assertGreater(len(synchronized_states), 30)
        for state in synchronized_states:
            self.assertEqual(list(state.robot_pose_valid), [True] * 3)
            self.assertEqual(list(state.path_state_valid), [True] * 3)
            self.assertTrue(state.load_pose_valid)
            self.assertTrue(state.load_path_state_valid)
            stamps = [stamp.to_sec() for stamp in state.robot_pose_stamp]
            self.assertLessEqual(max(stamps) - min(stamps), 0.020001)

        # Stop only AGV3. Once no timestamp-matched triplet remains, the
        # complete cooperative snapshot becomes invalid rather than exposing
        # three individually fresh but mutually inconsistent robot poses.
        for step in range(25):
            stamp = rospy.Time.now()
            xi = 0.19 + 0.002 * step
            self._publish_robot(0, xi, stamp)
            self._publish_robot(1, xi, stamp)
            rate.sleep()
        state = self._latest()
        self.assertEqual(list(state.robot_pose_valid), [False] * 3)
        self.assertEqual(list(state.support_pose_valid), [False] * 3)
        self.assertEqual(list(state.path_state_valid), [False] * 3)
        self.assertEqual(list(state.robot_localization_source),
                         [CooperativeState.SOURCE_UNKNOWN] * 3)
        self.assertFalse(state.load_pose_valid)
        self.assertFalse(state.load_path_state_valid)


if __name__ == "__main__":
    rospy.init_node("test_odom_state_estimator")
    rostest.rosrun("multi_agv_control", "odom_state_estimator",
                  OdomStateEstimatorTest)
