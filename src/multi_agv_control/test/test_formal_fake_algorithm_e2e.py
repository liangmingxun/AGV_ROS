#!/usr/bin/env python3
import math
import threading
import unittest

import rosgraph
import rospy
import rostest
from agv_msgs.msg import (ChassisCommand, ControllerState,
                          CooperativeState, PathReference)
from std_msgs.msg import Bool, Float64MultiArray, String


class FormalFakeAlgorithmE2ETest(unittest.TestCase):
    OFFSETS = (
        (0.1732050807568877, 0.0),
        (-0.0866025403784439, 0.1500000000000000),
        (-0.0866025403784439, -0.1500000000000000),
    )
    BASE_TO_SUPPORT_X = (-0.01783, 0.09908, 0.09908)

    def setUp(self):
        self._lock = threading.Lock()
        self.commands = [[], [], []]
        self.controllers = []
        self.references = []
        self.debug = []
        self.m2b_debug = []
        self.state_publisher = rospy.Publisher(
            "/multi_agv/cooperative_state", CooperativeState, queue_size=5)
        self.armed_publisher = rospy.Publisher(
            "/experiment_recorder/armed", Bool, queue_size=1)
        self.recorder_method_publisher = rospy.Publisher(
            "/experiment_recorder/method_id", String, queue_size=1,
            latch=True)
        for index in range(3):
            rospy.Subscriber(
                "/agv{}/chassis_command".format(index + 1),
                ChassisCommand,
                lambda message, i=index: self._append(
                    self.commands[i], message),
                queue_size=100)
        rospy.Subscriber(
            "/multi_agv/controller_state", ControllerState,
            lambda message: self._append(self.controllers, message),
            queue_size=100)
        rospy.Subscriber(
            "/multi_agv/path_reference", PathReference,
            lambda message: self._append(self.references, message),
            queue_size=100)
        rospy.Subscriber(
            "/multi_agv/formal_algorithm_state", Float64MultiArray,
            lambda message: self._append(self.debug, message),
            queue_size=100)
        rospy.Subscriber(
            "/multi_agv/m2b_algorithm_state", Float64MultiArray,
            lambda message: self._append(self.m2b_debug, message),
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
        position = (
            xi + qt * tangent[0] + qn * normal[0],
            y + qt * tangent[1] + qn * normal[1])
        derivative = (
            (1.0 - curvature * qn) * tangent[0] +
            curvature * qt * normal[0],
            (1.0 - curvature * qn) * tangent[1] +
            curvature * qt * normal[1])
        return position, math.atan2(derivative[1], derivative[0])

    def _state(self):
        message = CooperativeState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.robot_localization_source = [
            CooperativeState.SOURCE_ODOM] * 3
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
            message.s_actual[index] = 0.0
            message.s_dot_actual[index] = 0.0
        message.load_localization_source = CooperativeState.SOURCE_ODOM
        message.load_pose_valid = True
        message.load_path_state_valid = True
        message.load_pose_stamp = message.header.stamp
        return message

    def test_formal_chain_records_state_and_fails_to_zero(self):
        expected_method = rospy.get_param("~expected_method", "M1_R1")
        exercise_recorder_gate = rospy.get_param(
            "~exercise_recorder_gate", False)
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while (self.state_publisher.get_num_connections() == 0 and
               rospy.Time.now() < deadline):
            rospy.sleep(0.02)
        self.assertGreater(self.state_publisher.get_num_connections(), 0)

        rate = rospy.Rate(100)
        for _ in range(180):
            if exercise_recorder_gate:
                self.recorder_method_publisher.publish(
                    String(data=expected_method))
                self.armed_publisher.publish(Bool(data=True))
            self.state_publisher.publish(self._state())
            rate.sleep()

        with self._lock:
            self.assertTrue(all(len(values) > 30 for values in self.commands))
            self.assertGreater(len(self.controllers), 30)
            self.assertGreater(len(self.references), 30)
            self.assertGreater(len(self.debug), 30)
            command_history = [list(values) for values in self.commands]
            controller_history = list(self.controllers)
            reference_history = list(self.references)
            debug_history = list(self.debug)
            m2b_debug_history = list(self.m2b_debug)

        nonzero_commands = [[
            value for value in values
            if abs(value.wheel_linear_velocity_left_raw) > 1e-4 or
            abs(value.wheel_linear_velocity_right_raw) > 1e-4
        ] for values in command_history]
        self.assertTrue(
            all(nonzero_commands),
            "nonzero command counts={}".format(
                [len(values) for values in nonzero_commands]))
        commands = [values[-1] for values in nonzero_commands]
        valid_controllers = [
            value for value in controller_history
            if value.common_load_velocity_reference > 0.0]
        valid_references = [
            value for value in reference_history
            if value.load_path_progress_reference > 0.0]
        valid_debug = [
            value for value in debug_history
            if value.data and value.data[0] == 1.0]
        self.assertTrue(valid_controllers)
        self.assertTrue(valid_references)
        self.assertTrue(valid_debug)
        controller = valid_controllers[-1]
        reference = valid_references[-1]
        debug = valid_debug[-1]

        self.assertEqual(controller.method_id, expected_method)
        self.assertEqual([value.method_id for value in commands],
                         [expected_method] * 3)
        for index in range(1, 4):
            self.assertEqual(rospy.get_param(
                "/agv{}/chassis_controller/transport_type".format(index)),
                "fake")
        self.assertTrue(all(value.command_seq > 0 for value in commands))
        self.assertGreater(reference.load_path_progress_reference, 0.0)
        self.assertGreater(controller.common_load_velocity_reference, 0.0)
        self.assertEqual(debug.layout.dim[0].label,
                         "formal_algorithm_state_v2:header10+3x29")
        self.assertEqual(len(debug.data), 97)
        self.assertEqual(debug.data[0], 1.0)
        self.assertGreater(debug.data[8], 0.0)
        self.assertEqual(debug.data[9], 1.0)
        self.assertTrue(all(math.isfinite(value) for value in debug.data))
        expected_reference = rospy.get_param(
            "~expected_reference_velocity", None)
        if expected_reference is not None:
            # This generic harness deliberately holds the reported plant state
            # at s=0, so the distributed reference slows later in the run.
            # Validate the initialized physical scale on the highest valid
            # reference sample rather than pretending this is a plant model.
            scale_debug = max(valid_debug, key=lambda value: value.data[5])
            self.assertAlmostEqual(
                float(expected_reference), scale_debug.data[5], delta=0.002)
            # M1's common upper is dynamic: a stationary fake plant may
            # legitimately contract it below the nominal 0.09 m/s ceiling.
            # The executable reference-to-bound margin is checked below.
            self.assertLessEqual(
                scale_debug.data[4],
                float(rospy.get_param("~expected_common_upper")) + 0.002)
            minimum_mapped = float(rospy.get_param(
                "~minimum_mapped_capability"))
            for robot in range(3):
                mapped_upper = scale_debug.data[10 + robot * 29 + 28]
                self.assertGreater(mapped_upper, minimum_mapped)
            self.assertGreaterEqual(
                scale_debug.data[4] - scale_debug.data[5],
                float(rospy.get_param(
                    "~expected_minimum_reference_margin", 0.005)) - 1e-6)
        if rospy.get_param("~expect_m2b_debug", False):
            valid_m2b = [
                value for value in m2b_debug_history
                if value.data and value.data[0] == 1.0]
            self.assertTrue(valid_m2b)
            self.assertEqual(
                valid_m2b[-1].layout.dim[0].label,
                "m2b_algorithm_state_v1:header6+3x20")
            self.assertEqual(len(valid_m2b[-1].data), 66)
            self.assertTrue(
                all(math.isfinite(value) for value in valid_m2b[-1].data))

        # A single temporal-resynchronization validity dropout must reuse the
        # last fully valid state instead of injecting one global zero command.
        # A sustained loss is tested separately below and must still fail zero.
        # Refresh the valid-state timestamp immediately before the injected
        # dropout; time spent evaluating the assertions above is unrelated to
        # the production 30 ms hold contract.
        for _ in range(3):
            self.state_publisher.publish(self._state())
            rate.sleep()
        with self._lock:
            for values in self.commands:
                values[:] = []
        invalid_state = self._state()
        invalid_state.path_state_valid[0] = False
        if exercise_recorder_gate:
            self.recorder_method_publisher.publish(
                String(data=expected_method))
            self.armed_publisher.publish(Bool(data=True))
        self.state_publisher.publish(invalid_state)
        rospy.sleep(0.015)
        for _ in range(5):
            if exercise_recorder_gate:
                self.recorder_method_publisher.publish(
                    String(data=expected_method))
                self.armed_publisher.publish(Bool(data=True))
            self.state_publisher.publish(self._state())
            rate.sleep()
        with self._lock:
            transient_commands = [
                command for values in self.commands for command in values]
        self.assertTrue(transient_commands)
        self.assertTrue(all(
            abs(command.wheel_linear_velocity_left_raw) > 1e-4 or
            abs(command.wheel_linear_velocity_right_raw) > 1e-4
            for command in transient_commands))

        if exercise_recorder_gate:
            # Keep primary state fresh while deliberately stopping only the
            # recorder heartbeat. The command path must fail zero after the
            # configured heartbeat age, proving abrupt recorder loss is not
            # masked by the original latched true value.
            for _ in range(80):
                self.state_publisher.publish(self._state())
                rate.sleep()
            with self._lock:
                recorder_loss_commands = [
                    values[-1] for values in self.commands]
                recorder_loss_debug = self.debug[-1]
            self.assertTrue(all(
                value.wheel_linear_velocity_left_raw == 0.0 and
                value.wheel_linear_velocity_right_raw == 0.0
                for value in recorder_loss_commands))
            self.assertEqual(recorder_loss_debug.data[0], 0.0)

        # Stop only the cooperative state. Capability reports continue from
        # fake chassis nodes; stale state alone must zero all three outputs.
        rospy.sleep(0.35)
        with self._lock:
            zero_commands = [values[-1] for values in self.commands]
            failed_controller = self.controllers[-1]
            failed_debug = self.debug[-1]
        self.assertTrue(all(
            value.linear_velocity_reference == 0.0 and
            value.angular_velocity_reference == 0.0 and
            value.wheel_linear_velocity_left_raw == 0.0 and
            value.wheel_linear_velocity_right_raw == 0.0
            for value in zero_commands))
        self.assertEqual(failed_controller.common_load_velocity_reference, 0.0)
        self.assertEqual(failed_debug.data[0], 0.0)

        publishers, _, _ = rosgraph.Master(
            rospy.get_name()).getSystemState()
        publisher_map = dict(publishers)
        for index in range(1, 4):
            topic = "/agv{}/chassis_command".format(index)
            self.assertEqual(len(publisher_map[topic]), 1)
        self.assertNotIn("/cmd_vel", publisher_map)


if __name__ == "__main__":
    rospy.init_node("test_formal_fake_algorithm_e2e")
    rostest.rosrun(
        "multi_agv_control", "formal_fake_algorithm_e2e",
        FormalFakeAlgorithmE2ETest)
