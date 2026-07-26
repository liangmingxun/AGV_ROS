#!/usr/bin/env python3
import threading
import unittest

import rosgraph
import rospy
import rostest
from agv_msgs.msg import ControllerState, CooperativeState, PathReference


class CentralReadonlyE2ETest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self.states = []
        self.references = []
        self.controllers = []
        rospy.Subscriber(
            "/multi_agv/cooperative_state", CooperativeState,
            lambda message: self._append(self.states, message), queue_size=100)
        rospy.Subscriber(
            "/multi_agv/path_reference", PathReference,
            lambda message: self._append(self.references, message),
            queue_size=100)
        rospy.Subscriber(
            "/multi_agv/controller_state", ControllerState,
            lambda message: self._append(self.controllers, message),
            queue_size=100)

    def _append(self, target, message):
        with self._lock:
            target.append(message)
            del target[:-500]

    def test_readonly_controller_never_registers_command_publishers(self):
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._lock:
                ready = (len(self.states) > 20 and
                         len(self.references) > 20 and
                         len(self.controllers) > 20)
            if ready:
                break
            rospy.sleep(0.05)

        with self._lock:
            self.assertGreater(len(self.states), 20)
            self.assertGreater(len(self.references), 20)
            self.assertGreater(len(self.controllers), 20)
            state = self.states[-1]
            reference = self.references[-1]

        self.assertTrue(all(state.robot_pose_valid))
        self.assertTrue(all(state.support_pose_valid))
        self.assertTrue(all(state.path_state_valid))
        self.assertGreater(reference.load_path_progress_reference, 0.0)

        publishers, subscribers, _ = rosgraph.Master(
            rospy.get_name()).getSystemState()
        publisher_map = dict(publishers)
        subscriber_map = dict(subscribers)
        for index in range(1, 4):
            topic = "/agv{}/chassis_command".format(index)
            self.assertNotIn(topic, publisher_map)
            self.assertIn(topic, subscriber_map)
            self.assertEqual(
                subscriber_map[topic], ["/agv{}/chassis_controller".format(index)])
        self.assertNotIn("/agv2/derating_command", publisher_map)


if __name__ == "__main__":
    rospy.init_node("test_central_readonly_e2e")
    rostest.rosrun(
        "multi_agv_control", "central_readonly_e2e",
        CentralReadonlyE2ETest)
