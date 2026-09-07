#!/usr/bin/env python3
import math
import threading
import unittest

import rosgraph
import rospy
import rostest
from agv_msgs.msg import (ChassisCommand, ChassisFeedback, ControllerState,
                          CooperativeState, PathReference)


class ConstantReferenceE2ETest(unittest.TestCase):
    OFFSETS = (
        (0.1732050807568877, 0.0),
        (-0.0866025403784439, 0.1500000000000000),
        (-0.0866025403784439, -0.1500000000000000),
    )
    BASE_TO_SUPPORT_X = (-0.01783, 0.09908, 0.09908)

    def setUp(self):
        self._lock = threading.Lock()
        self.commands = [[], [], []]
        self.feedback = [[], [], []]
        self.references = []
        self.controllers = []
        self.state_publisher = rospy.Publisher(
            "/multi_agv/cooperative_state", CooperativeState, queue_size=5)
        for index in range(3):
            rospy.Subscriber(
                "/agv{}/chassis_command".format(index + 1), ChassisCommand,
                lambda message, i=index: self._append(self.commands[i], message),
                queue_size=100)
            rospy.Subscriber(
                "/agv{}/chassis_feedback".format(index + 1), ChassisFeedback,
                lambda message, i=index: self._append(self.feedback[i], message),
                queue_size=100)
        rospy.Subscriber("/multi_agv/path_reference", PathReference,
                         lambda message: self._append(self.references, message),
                         queue_size=100)
        rospy.Subscriber("/multi_agv/controller_state", ControllerState,
                         lambda message: self._append(self.controllers, message),
                         queue_size=100)

    def _append(self, target, message):
        with self._lock:
            target.append(message)
            del target[:-500]

    @staticmethod
    def _sample(xi, offset):
        amplitude = 0.05
        wave_number = 2.0 * math.pi
        y = amplitude * math.sin(wave_number * xi)
        slope = amplitude * wave_number * math.cos(wave_number * xi)
        second = -amplitude * wave_number ** 2 * math.sin(wave_number * xi)
        scale = math.hypot(1.0, slope)
        tangent = (1.0 / scale, slope / scale)
        normal = (-tangent[1], tangent[0])
        curvature = second / scale ** 3
        qt, qn = offset
        position = (xi + qt * tangent[0] + qn * normal[0],
                    y + qt * tangent[1] + qn * normal[1])
        derivative = ((1.0 - curvature * qn) * tangent[0] +
                      curvature * qt * normal[0],
                      (1.0 - curvature * qn) * tangent[1] +
                      curvature * qt * normal[1])
        return position, math.atan2(derivative[1], derivative[0])

    def _state(self):
        message = CooperativeState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.robot_localization_source = [CooperativeState.SOURCE_ODOM] * 3
        for index, offset in enumerate(self.OFFSETS):
            position, yaw = self._sample(0.0, offset)
            message.robot_pose_valid[index] = True
            message.support_pose_valid[index] = True
            message.path_state_valid[index] = True
            message.robot_pose_stamp[index] = message.header.stamp
            message.robot_pose[index].x = (
                position[0] - self.BASE_TO_SUPPORT_X[index] * math.cos(yaw))
            message.robot_pose[index].y = (
                position[1] - self.BASE_TO_SUPPORT_X[index] * math.sin(yaw))
            message.robot_pose[index].theta = yaw
            message.support_pose[index].x = position[0]
            message.support_pose[index].y = position[1]
            message.support_pose[index].theta = yaw
        return message

    def test_fake_chassis_traceability_and_common_reference(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while (self.state_publisher.get_num_connections() == 0 and
               rospy.Time.now() < deadline):
            rospy.sleep(0.02)
        self.assertGreater(self.state_publisher.get_num_connections(), 0)
        rate = rospy.Rate(100)
        for _ in range(180):
            self.state_publisher.publish(self._state())
            rate.sleep()

        with self._lock:
            self.assertTrue(all(len(values) > 20 for values in self.commands))
            self.assertTrue(all(len(values) > 20 for values in self.feedback))
            self.assertGreater(len(self.references), 20)
            self.assertGreater(len(self.controllers), 20)
            latest_commands = [values[-1] for values in self.commands]
            latest_controller = self.controllers[-1]
            latest_reference = self.references[-1]

        wheel_pairs = {(round(value.wheel_linear_velocity_left_raw, 6),
                        round(value.wheel_linear_velocity_right_raw, 6))
                       for value in latest_commands}
        self.assertGreater(len(wheel_pairs), 1)
        self.assertTrue(all(value.command_seq > 0 for value in latest_commands))
        self.assertEqual([value.robot_id for value in latest_commands], [1, 2, 3])
        self.assertEqual(len(set(round(value, 12) for value in
                                 latest_controller.path_progress_execute_reference)),
                         1)
        self.assertGreater(latest_reference.load_path_progress_reference, 0.0)

        for index in range(3):
            with self._lock:
                samples = list(self.feedback[index])
            applied = [sample for sample in samples
                       if sample.command_seq_applied > 0]
            self.assertTrue(applied)
            self.assertTrue(any(abs(sample.wheel_linear_velocity_left_raw) > 1e-4
                                for sample in applied))
            self.assertTrue(all(abs(sample.wheel_linear_velocity_left_applied) <=
                                0.9 + 1e-9 for sample in applied))
            sequences = [sample.feedback_seq for sample in samples[-20:]]
            self.assertTrue(all(new == old + 1
                                for old, new in zip(sequences, sequences[1:])))

        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        publisher_map = dict(publishers)
        for index in range(1, 4):
            self.assertEqual(len(publisher_map["/agv{}/chassis_command".format(index)]),
                             1)
        self.assertNotIn("/cmd_vel", publisher_map)


if __name__ == "__main__":
    rospy.init_node("test_constant_reference_e2e")
    rostest.rosrun("multi_agv_control", "constant_reference_e2e",
                  ConstantReferenceE2ETest)
