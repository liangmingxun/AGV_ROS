#!/usr/bin/env python3

import json
import sys

import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool

from multi_agv_analysis.monitoring import SoftwareMonitor


class SoftwareWatchdogNode:
    def __init__(self):
        config = rospy.get_param("~software_watchdog")
        self.monitor = SoftwareMonitor(
            config.get("topics", []),
            float(config.get("startup_grace_seconds", 2.0)))
        self.latch_failures = bool(config.get("latch_failures", True))
        self.latched_failure = False
        self.subscribers = []
        for topic in self.monitor.rules:
            self.subscribers.append(rospy.Subscriber(
                topic, rospy.AnyMsg,
                lambda _message, name=topic: self.monitor.receive(
                    name, rospy.Time.now().to_sec()),
                queue_size=10))
        self.ok_publisher = rospy.Publisher(
            "/multi_agv/software_watchdog_ok", Bool,
            queue_size=1, latch=True)
        self.diagnostic_publisher = rospy.Publisher(
            "/multi_agv/software_watchdog_diagnostics",
            DiagnosticArray, queue_size=5)
        rate = float(config.get("publish_rate", 10.0))
        if rate <= 0.0:
            raise ValueError("software watchdog publish_rate must be positive")
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / rate), self.step)

    def step(self, _event):
        publishers, _, _ = rosgraph.Master(
            rospy.get_name()).getSystemState()
        result = self.monitor.evaluate(
            rospy.Time.now().to_sec(), dict(publishers))
        if not result["healthy"] and not result["startup_grace"]:
            self.latched_failure = self.latched_failure or self.latch_failures
        ok = result["healthy"] and not self.latched_failure
        self.ok_publisher.publish(Bool(data=ok))
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = "multi_agv/software_watchdog"
        status.hardware_id = "linux_ros_observer_only"
        status.level = (
            DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR)
        status.message = (
            "healthy" if ok else
            "software receipt/authority fault; no hardware safety claim")
        status.values = [
            KeyValue(key=topic, value=json.dumps(value, sort_keys=True))
            for topic, value in sorted(result["topics"].items())]
        message.status = [status]
        self.diagnostic_publisher.publish(message)


def main():
    rospy.init_node("multi_agv_software_watchdog")
    try:
        SoftwareWatchdogNode()
        rospy.spin()
        return 0
    except Exception as error:
        rospy.logfatal("Software watchdog refused to start: %s", error)
        return 2


if __name__ == "__main__":
    sys.exit(main())
