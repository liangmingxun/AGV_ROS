#!/usr/bin/env python3
import threading
import unittest

import rospy
import rostest
from agv_msgs.msg import (CapabilityReport, ChassisCommand, ChassisFeedback,
                          CooperativeState, DeratingCommand, ExperimentState)


class DeratingChainE2ETest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.capabilities = [[], [], []]
        self.feedback = []
        self.derating = []
        self.experiment = []
        self.state_publisher = rospy.Publisher(
            "/multi_agv/cooperative_state", CooperativeState, queue_size=5)
        self.command_publisher = rospy.Publisher(
            "/agv2/chassis_command", ChassisCommand, queue_size=1)
        for index in range(3):
            rospy.Subscriber(
                "/agv{}/capability_report".format(index + 1), CapabilityReport,
                lambda message, i=index: self._append(self.capabilities[i], message),
                queue_size=500)
        rospy.Subscriber("/agv2/chassis_feedback", ChassisFeedback,
                         lambda message: self._append(self.feedback, message),
                         queue_size=500)
        rospy.Subscriber("/agv2/derating_command", DeratingCommand,
                         lambda message: self._append(self.derating, message),
                         queue_size=100)
        rospy.Subscriber("/multi_agv/experiment_state", ExperimentState,
                         lambda message: self._append(self.experiment, message),
                         queue_size=100)
        self.command_sequence = 0

    def _append(self, target, message):
        with self.lock:
            target.append(message)
            del target[:-1000]

    def _publish(self, progress):
        state = CooperativeState()
        state.header.stamp = rospy.Time.now()
        state.path_state_valid[1] = True
        state.s_actual[1] = progress
        self.state_publisher.publish(state)
        self.command_sequence += 1
        command = ChassisCommand()
        command.header.stamp = state.header.stamp
        command.robot_id = 2
        command.command_seq = self.command_sequence
        command.wheel_linear_velocity_left_raw = 0.8
        command.wheel_linear_velocity_right_raw = 0.8
        self.command_publisher.publish(command)

    def _run_segment(self, progress, seconds):
        rate = rospy.Rate(100)
        for _ in range(int(seconds * 100)):
            self._publish(progress)
            rate.sleep()

    def test_actual_progress_drives_capability_and_applied_chain(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < deadline:
            if (self.state_publisher.get_num_connections() > 0 and
                    self.command_publisher.get_num_connections() > 0):
                break
            rospy.sleep(0.02)
        self.assertGreater(self.state_publisher.get_num_connections(), 0)
        self.assertGreater(self.command_publisher.get_num_connections(), 0)

        self._run_segment(0.20, 0.5)
        with self.lock:
            nominal_sequence = self.derating[-1].command_seq
        # A reference value is never published or subscribed; actual s2 alone
        # crosses the activation threshold.
        self._run_segment(0.31, 1.0)
        with self.lock:
            active_commands = [value for value in self.derating if value.active]
            car2_during = list(self.capabilities[1])
        self.assertTrue(active_commands)
        self.assertEqual(len({value.command_seq for value in active_commands}), 1)
        self.assertGreater(active_commands[0].command_seq, nominal_sequence)
        self.assertLess(min(value.max_wheel_linear_velocity_left
                            for value in car2_during), 0.37)

        self._run_segment(0.61, 1.0)
        with self.lock:
            car1 = list(self.capabilities[0])
            car2 = list(self.capabilities[1])
            car3 = list(self.capabilities[2])
            feedback = list(self.feedback)
            derating = list(self.derating)
            experiment = list(self.experiment)
        self.assertTrue(car1 and car2 and car3 and feedback and experiment)
        self.assertTrue(all(abs(value.max_wheel_linear_velocity_left - 0.9) < 1e-9
                            for value in car1[-20:] + car3[-20:]))
        self.assertGreater(car2[-1].max_wheel_linear_velocity_left, 0.85)
        self.assertFalse(derating[-1].active)
        self.assertGreater(derating[-1].command_seq,
                           active_commands[0].command_seq)

        # Compare each applied sample with the most recent capability report at
        # or before its generation time.
        checked = 0
        for sample in feedback:
            earlier = [cap for cap in car2
                       if cap.header.stamp <= sample.header.stamp]
            if earlier:
                limit = earlier[-1].max_wheel_linear_velocity_left
                self.assertLessEqual(abs(sample.wheel_linear_velocity_left_applied),
                                     limit + 1e-9)
                checked += 1
        self.assertGreater(checked, 50)
        self.assertTrue(any(value.phase == 3 for value in experiment))
        self.assertTrue(any(value.phase == 4 for value in experiment))
        self.assertTrue(any(value.phase == 5 for value in experiment))
        self.assertTrue(any(value.phase == 6 for value in experiment))
        self.assertTrue(all(new.capability_seq == old.capability_seq + 1
                            for old, new in zip(car2[-20:], car2[-19:])))

        first_active = active_commands[0].header.stamp
        changed_capability = next(
            value for value in car2
            if value.header.stamp >= first_active and value.derating_active)
        changed_feedback = next(
            value for value in feedback
            if value.header.stamp >= changed_capability.header.stamp)
        self.assertLessEqual(first_active, changed_capability.header.stamp)
        self.assertLessEqual(changed_capability.header.stamp,
                             changed_feedback.header.stamp)


if __name__ == "__main__":
    rospy.init_node("test_derating_chain_e2e")
    rostest.rosrun("multi_agv_control", "derating_chain_e2e",
                  DeratingChainE2ETest)
