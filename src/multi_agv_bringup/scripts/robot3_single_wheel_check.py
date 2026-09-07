#!/usr/bin/env python3
"""Bounded lifted-wheel direction check for Robot3."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rosgraph
import rospy
from agv_msgs.msg import ChassisCommand, ChassisFeedback


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--wheel", choices=("left", "right", "both"),
                        default="both")
    parser.add_argument("--confirm-wheels-lifted", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
    args = parser.parse_args()
    if not args.confirm_wheels_lifted or not args.confirm_emergency_stop_ready:
        parser.error("both physical safety confirmations are required")
    return args


class WheelCheck:
    SPEED = 0.01
    DURATION = 0.5
    OVERSPEED = 0.03
    INACTIVE_LIMIT = 0.006
    STOPPED_LIMIT = 0.005

    def __init__(self, output, wheel):
        self.output = Path(output)
        self.wheel = wheel
        self.feedback = None
        self.feedback_wall_time = 0.0
        self.segment_samples = []
        self.segment_name = None
        self.sequence = 900000
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        existing = dict(publishers).get("/agv3/chassis_command", [])
        if existing:
            raise RuntimeError(
                "agv3 command already has publisher(s): {}".format(
                    ", ".join(existing)))
        self.publisher = rospy.Publisher(
            "/agv3/chassis_command", ChassisCommand, queue_size=2)
        rospy.Subscriber(
            "/agv3/chassis_feedback", ChassisFeedback,
            self._feedback_callback, queue_size=50, tcp_nodelay=True)

    def _feedback_callback(self, message):
        if message.robot_id != 3:
            return
        self.feedback = message
        self.feedback_wall_time = time.monotonic()
        if self.segment_name is not None:
            self.segment_samples.append((
                time.monotonic(),
                float(message.wheel_linear_velocity_left_actual),
                float(message.wheel_linear_velocity_right_actual),
                float(message.wheel_linear_velocity_left_applied),
                float(message.wheel_linear_velocity_right_applied)))

    def wait_ready(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.feedback is not None:
                actual = (
                    self.feedback.wheel_linear_velocity_left_actual,
                    self.feedback.wheel_linear_velocity_right_actual)
                if max(abs(value) for value in actual) <= self.STOPPED_LIMIT:
                    self.sequence = max(
                        self.sequence,
                        int(self.feedback.command_seq_applied) + 100)
                    if self.publisher.get_num_connections() >= 2:
                        return
            time.sleep(0.05)
        raise RuntimeError(
            "fresh stopped feedback and both chassis/recorder subscribers "
            "were not ready")

    def publish(self, left, right, method):
        message = ChassisCommand()
        message.header.stamp = rospy.Time.now()
        message.robot_id = 3
        message.command_seq = self.sequence
        message.control_mode = 1
        message.linear_velocity_reference = 0.5 * (left + right)
        message.angular_velocity_reference = (
            (right - left) / 0.136802843)
        message.wheel_linear_velocity_left_raw = left
        message.wheel_linear_velocity_right_raw = right
        message.experiment_id = "robot3_single_wheel_check"
        message.method_id = method
        self.publisher.publish(message)

    def stop(self):
        self.sequence += 1
        for _ in range(15):
            self.publish(0.0, 0.0, "SINGLE_WHEEL_STOP")
            time.sleep(0.02)

    def wait_stopped(self):
        deadline = time.monotonic() + 2.0
        stable = 0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.feedback is not None and max(
                    abs(self.feedback.wheel_linear_velocity_left_actual),
                    abs(self.feedback.wheel_linear_velocity_right_actual)
                    ) <= self.STOPPED_LIMIT:
                stable += 1
                if stable >= 5:
                    return
            else:
                stable = 0
            self.publish(0.0, 0.0, "SINGLE_WHEEL_STOP")
            time.sleep(0.02)
        raise RuntimeError("both Robot3 wheels did not stop within 2 seconds")

    def run_segment(self, name, left, right):
        self.sequence += 1
        self.segment_name = name
        self.segment_samples = []
        start = time.monotonic()
        rate = rospy.Rate(50)
        try:
            while time.monotonic() - start < self.DURATION:
                if rospy.is_shutdown():
                    raise RuntimeError("ROS shutdown during wheel check")
                if time.monotonic() - self.feedback_wall_time > 0.25:
                    raise RuntimeError("Robot3 feedback became stale")
                actual = (
                    self.feedback.wheel_linear_velocity_left_actual,
                    self.feedback.wheel_linear_velocity_right_actual)
                if not all(math.isfinite(value) for value in actual):
                    raise RuntimeError("Robot3 wheel feedback is non-finite")
                if max(abs(value) for value in actual) > self.OVERSPEED:
                    raise RuntimeError(
                        "actual wheel speed exceeded {:.3f} m/s".format(
                            self.OVERSPEED))
                self.publish(left, right, "SINGLE_WHEEL_" + name)
                rate.sleep()
        finally:
            self.segment_name = None
            self.stop()
        self.wait_stopped()
        samples = [sample for sample in self.segment_samples
                   if sample[0] - start >= 0.15]
        if not samples:
            raise RuntimeError(name + " produced no settled feedback samples")
        target_index = 1 if abs(left) > 0.0 else 2
        inactive_index = 2 if target_index == 1 else 1
        target_values = [sample[target_index] for sample in samples]
        inactive_values = [sample[inactive_index] for sample in samples]
        expected = left if target_index == 1 else right
        median_actual = statistics.median(target_values)
        result = {
            "segment": name,
            "command_left_mps": left,
            "command_right_mps": right,
            "target_actual_median_mps": median_actual,
            "target_actual_peak_abs_mps": max(abs(v) for v in target_values),
            "inactive_actual_peak_abs_mps": max(abs(v) for v in inactive_values),
            "sample_count": len(samples),
        }
        if abs(median_actual) < 0.003:
            raise RuntimeError(name + " target wheel did not respond")
        if math.copysign(1.0, median_actual) != math.copysign(1.0, expected):
            raise RuntimeError(name + " target wheel direction is reversed")
        if result["inactive_actual_peak_abs_mps"] > self.INACTIVE_LIMIT:
            raise RuntimeError(name + " inactive wheel moved unexpectedly")
        return result

    def run(self):
        self.wait_ready()
        results = []
        segments = (
                ("LEFT_FORWARD", self.SPEED, 0.0),
                ("RIGHT_FORWARD", 0.0, self.SPEED),
                ("LEFT_REVERSE", -self.SPEED, 0.0),
                ("RIGHT_REVERSE", 0.0, -self.SPEED))
        for name, left, right in segments:
            if self.wheel == "left" and abs(left) == 0.0:
                continue
            if self.wheel == "right" and abs(right) == 0.0:
                continue
            rospy.logwarn(
                "%s: command left=%+.3f right=%+.3f m/s for %.1f s",
                name, left, right, self.DURATION)
            results.append(self.run_segment(name, left, right))
            time.sleep(0.5)
        payload = {
            "status": "PASS",
            "robot_id": "agv3",
            "requested_wheel": self.wheel,
            "wheels_lifted": True,
            "command_speed_mps": self.SPEED,
            "segment_duration_s": self.DURATION,
            "segments": results,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        return payload


def main():
    args = parse_args()
    rospy.init_node("robot3_single_wheel_check", disable_signals=True)
    check = None
    try:
        check = WheelCheck(args.output, args.wheel)
        result = check.run()
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except Exception as error:
        rospy.logerr("Robot3 single-wheel check failed: %s", error)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({
            "status": "FAIL",
            "robot_id": "agv3",
            "requested_wheel": args.wheel,
            "wheels_lifted": True,
            "reason": str(error),
        }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return 1
    finally:
        if check is not None:
            try:
                check.stop()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
