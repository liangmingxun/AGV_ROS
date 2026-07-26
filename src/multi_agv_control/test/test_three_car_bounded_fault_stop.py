#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest

from agv_msgs.msg import ChassisCommand, CooperativeState


class ThreeCarBoundedFaultStopTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._commands = [[], [], []]
        self._subscribers = []
        for index in range(3):
            self._subscribers.append(rospy.Subscriber(
                "/agv{}/chassis_command".format(index + 1),
                ChassisCommand,
                lambda message, robot=index: self._receive(
                    robot, message),
                queue_size=1000))

    def _receive(self, index, message):
        with self._condition:
            self._commands[index].append(message)
            self._condition.notify_all()

    @staticmethod
    def _has_motion(messages):
        return any(
            abs(message.wheel_linear_velocity_left_raw) > 1.0e-5
            or abs(message.wheel_linear_velocity_right_raw) > 1.0e-5
            for message in messages)

    @staticmethod
    def _stopped(messages):
        return len(messages) >= 5 and all(
            abs(message.wheel_linear_velocity_left_raw) <= 1.0e-9
            and abs(message.wheel_linear_velocity_right_raw) <= 1.0e-9
            for message in messages[-5:])

    def test_invalid_cooperative_state_stops_all_three(self):
        motion_deadline = rospy.Time.now() + rospy.Duration(8.0)
        with self._condition:
            while rospy.Time.now() < motion_deadline:
                if all(self._has_motion(messages)
                       for messages in self._commands):
                    break
                self._condition.wait(0.02)
        self.assertTrue(all(
            self._has_motion(messages) for messages in self._commands))

        fault_publisher = rospy.Publisher(
            "/multi_agv/cooperative_state", CooperativeState,
            queue_size=10)
        connection_deadline = rospy.Time.now() + rospy.Duration(2.0)
        while (fault_publisher.get_num_connections() == 0
               and rospy.Time.now() < connection_deadline):
            rospy.sleep(0.01)
        self.assertGreater(fault_publisher.get_num_connections(), 0)

        invalid = CooperativeState()
        fault_end = rospy.Time.now() + rospy.Duration(0.4)
        rate = rospy.Rate(200)
        while rospy.Time.now() < fault_end:
            invalid.header.stamp = rospy.Time.now()
            fault_publisher.publish(invalid)
            rate.sleep()

        stop_deadline = rospy.Time.now() + rospy.Duration(3.0)
        with self._condition:
            while rospy.Time.now() < stop_deadline:
                if all(self._stopped(messages)
                       for messages in self._commands):
                    break
                self._condition.wait(0.02)
        self.assertTrue(all(
            self._stopped(messages) for messages in self._commands))


if __name__ == "__main__":
    rospy.init_node("test_three_car_bounded_fault_stop")
    rostest.rosrun(
        "multi_agv_control", "three_car_bounded_fault_stop",
        ThreeCarBoundedFaultStopTest)
