#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SingleRobotArchitectureTest(unittest.TestCase):
    def test_main_has_no_car_to_car_algorithm_or_udp_receiver(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for forbidden in ("SwarmData", "AlgorithmCommand", "udpReceiverThread",
                          "robot_ip", '"/agv_chassis/cmd_vel"'):
            self.assertNotIn(forbidden, source)

    def test_main_uses_relative_frozen_topics_and_fake_transport(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        for required in ('"chassis_command"', '"derating_command"',
                         '"chassis_feedback"', '"capability_report"',
                         'transport_type_ != "fake"'):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
