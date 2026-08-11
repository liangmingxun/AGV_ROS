#!/usr/bin/env python3
"""Validate the distributed three-AGV ROS graph without publishing commands."""

import argparse
import collections
import sys
import threading
import time

import rosgraph
import rospy
from agv_msgs.msg import CapabilityReport, ChassisFeedback, CooperativeState
from nav_msgs.msg import Odometry


ROBOT_COUNT = 3


class SampleBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._samples = collections.defaultdict(list)

    def append(self, key, message):
        with self._lock:
            self._samples[key].append((time.monotonic(), message))

    def get(self, key):
        with self._lock:
            return list(self._samples[key])


def topic_map(entries):
    return {topic: list(nodes) for topic, nodes in entries}


def sample_rate(samples):
    if len(samples) < 2:
        return 0.0
    elapsed = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0


def check_graph(errors):
    publishers, subscribers, _ = rosgraph.Master(
        rospy.get_name()).getSystemState()
    publisher_map = topic_map(publishers)
    subscriber_map = topic_map(subscribers)

    for index in range(1, ROBOT_COUNT + 1):
        for suffix in ("odom", "chassis_feedback", "capability_report"):
            topic = "/agv{}/{}".format(index, suffix)
            count = len(publisher_map.get(topic, []))
            if count != 1:
                errors.append(
                    "{} must have exactly one publisher, found {}: {}".format(
                        topic, count, publisher_map.get(topic, [])))

        command_topic = "/agv{}/chassis_command".format(index)
        if publisher_map.get(command_topic):
            errors.append(
                "{} must have no publisher in read-only mode: {}".format(
                    command_topic, publisher_map[command_topic]))
        if not subscriber_map.get(command_topic):
            errors.append(
                "{} has no chassis subscriber".format(command_topic))

        derating_topic = "/agv{}/derating_command".format(index)
        if publisher_map.get(derating_topic):
            errors.append(
                "{} must have no publisher in read-only mode: {}".format(
                    derating_topic, publisher_map[derating_topic]))

    cooperative_publishers = publisher_map.get(
        "/multi_agv/cooperative_state", [])
    if len(cooperative_publishers) != 1:
        errors.append(
            "/multi_agv/cooperative_state must have exactly one publisher, "
            "found {}: {}".format(
                len(cooperative_publishers), cooperative_publishers))


def validate_robot_samples(index, odometry, feedback, capability,
                           minimum_rate, minimum_capability_rate,
                           minimum_voltage,
                           maximum_serial_feedback_age,
                           maximum_future_stamp_offset, errors):
    label = "agv{}".format(index)
    for topic, samples, required_rate in (
            ("odom", odometry, minimum_rate),
            ("chassis_feedback", feedback, minimum_rate),
            ("capability_report", capability, minimum_capability_rate)):
        rate = sample_rate(samples)
        rospy.loginfo("%s/%s: %d samples, %.2f Hz",
                      label, topic, len(samples), rate)
        if rate < required_rate:
            errors.append(
                "{}/{} rate {:.2f} Hz is below {:.2f} Hz".format(
                    label, topic, rate, required_rate))

    if not odometry or not feedback or not capability:
        return

    latest_odom = odometry[-1][1]
    latest_feedback = feedback[-1][1]
    latest_capability = capability[-1][1]
    expected_odom = "{}/odom".format(label)
    expected_base = "{}/base_link".format(label)
    if latest_odom.header.frame_id != expected_odom:
        errors.append(
            "{} odom frame is {!r}, expected {!r}".format(
                label, latest_odom.header.frame_id, expected_odom))
    if latest_odom.child_frame_id != expected_base:
        errors.append(
            "{} odom child frame is {!r}, expected {!r}".format(
                label, latest_odom.child_frame_id, expected_base))

    for name, message in (
            ("chassis_feedback", latest_feedback),
            ("capability_report", latest_capability)):
        if message.robot_id != index:
            errors.append(
                "{}/{} robot_id is {}, expected {}".format(
                    label, name, message.robot_id, index))

    if abs(latest_feedback.wheel_linear_velocity_left_actual) > 0.01 or \
            abs(latest_feedback.wheel_linear_velocity_right_actual) > 0.01:
        errors.append(
            "{} is not stationary: left={:.6f}, right={:.6f} m/s".format(
                label,
                latest_feedback.wheel_linear_velocity_left_actual,
                latest_feedback.wheel_linear_velocity_right_actual))
    if any(message.control_loop_overrun for _, message in feedback):
        errors.append("{} reported control_loop_overrun".format(label))
    if minimum_voltage > 0.0 and \
            latest_feedback.battery_voltage < minimum_voltage:
        errors.append(
            "{} battery {:.3f} V is below {:.3f} V".format(
                label, latest_feedback.battery_voltage, minimum_voltage))

    packet_sequences = [message.packet_seq for _, message in feedback]
    if len(set(packet_sequences)) < 2:
        errors.append(
            "{} STM32 packet_seq did not advance during observation".format(
                label))
    serial_stamps = [
        message.serial_receive_stamp.to_sec() for _, message in feedback]
    if not serial_stamps or serial_stamps[-1] <= 0.0:
        errors.append("{} has no valid STM32 serial receive stamp".format(label))
    else:
        serial_age = rospy.Time.now().to_sec() - serial_stamps[-1]
        rospy.loginfo(
            "%s STM32 serial feedback age: %.6f s, packet_seq %u -> %u",
            label, serial_age, packet_sequences[0], packet_sequences[-1])
        if serial_age > maximum_serial_feedback_age:
            errors.append(
                "{} STM32 serial feedback age {:.6f} s exceeds {:.6f} s".
                format(label, serial_age, maximum_serial_feedback_age))
        if serial_age < -maximum_future_stamp_offset:
            errors.append(
                "{} STM32 serial stamp is {:.6f} s in the future; "
                "synchronise host clocks".format(label, -serial_age))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Observe the three-car distributed ROS graph without "
                    "publishing chassis or derating commands.")
    parser.add_argument("--observe-seconds", type=float, default=3.0)
    parser.add_argument("--minimum-rate", type=float, default=80.0)
    parser.add_argument(
        "--minimum-capability-rate", type=float, default=15.0,
        help="minimum capability-report rate; nominal publication is 20 Hz")
    parser.add_argument(
        "--minimum-voltage", type=float, default=10.5,
        help="minimum feedback voltage; use 0 only for fake-transport testing")
    parser.add_argument(
        "--maximum-stamp-spread", type=float, default=0.02,
        help="maximum allowed spread across the three odometry stamps")
    parser.add_argument(
        "--maximum-serial-feedback-age", type=float, default=0.15,
        help="maximum age of each chassis STM32 serial receive stamp")
    parser.add_argument(
        "--require-valid-state", action="store_true",
        help="also require all cooperative robot/support/path validity flags")
    parser.add_argument(
        "--require-fused-cooperative-state", action="store_true",
        help="require three_car_path and SOURCE_FUSED for all robots/load")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(rospy.myargv(argv=sys.argv)[1:] if argv is None else argv)
    if args.observe_seconds <= 0.0 or args.minimum_rate <= 0.0 or \
            args.minimum_capability_rate <= 0.0 or \
            args.minimum_voltage < 0.0 or args.maximum_stamp_spread <= 0.0 or \
            args.maximum_serial_feedback_age <= 0.0:
        raise ValueError(
            "observation, rate and stamp spread must be positive; "
            "voltage must be nonnegative")

    rospy.init_node("three_car_readonly_gate", anonymous=True)
    samples = SampleBuffer()
    for index in range(1, ROBOT_COUNT + 1):
        rospy.Subscriber(
            "/agv{}/odom".format(index), Odometry,
            lambda message, i=index: samples.append(("odom", i), message),
            queue_size=400, tcp_nodelay=True)
        rospy.Subscriber(
            "/agv{}/chassis_feedback".format(index), ChassisFeedback,
            lambda message, i=index: samples.append(("feedback", i), message),
            queue_size=400, tcp_nodelay=True)
        rospy.Subscriber(
            "/agv{}/capability_report".format(index), CapabilityReport,
            lambda message, i=index: samples.append(("capability", i), message),
            queue_size=400, tcp_nodelay=True)
    rospy.Subscriber(
        "/multi_agv/cooperative_state", CooperativeState,
        lambda message: samples.append(("cooperative", 0), message),
        queue_size=400, tcp_nodelay=True)

    rospy.loginfo(
        "Observing the read-only three-car graph for %.2f seconds",
        args.observe_seconds)
    deadline = time.monotonic() + args.observe_seconds
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.02)

    errors = []
    check_graph(errors)
    for index in range(1, ROBOT_COUNT + 1):
        validate_robot_samples(
            index,
            samples.get(("odom", index)),
            samples.get(("feedback", index)),
            samples.get(("capability", index)),
            args.minimum_rate,
            args.minimum_capability_rate,
            args.minimum_voltage,
            args.maximum_serial_feedback_age,
            args.maximum_stamp_spread,
            errors)

    cooperative = samples.get(("cooperative", 0))
    cooperative_rate = sample_rate(cooperative)
    rospy.loginfo(
        "multi_agv/cooperative_state: %d samples, %.2f Hz",
        len(cooperative), cooperative_rate)
    if cooperative_rate < args.minimum_rate:
        errors.append(
            "cooperative_state rate {:.2f} Hz is below {:.2f} Hz".format(
                cooperative_rate, args.minimum_rate))
    if cooperative:
        state = cooperative[-1][1]
        rospy.loginfo(
            "cooperative validity robot=%s support=%s path=%s "
            "load_pose=%s load_path=%s",
            list(state.robot_pose_valid), list(state.support_pose_valid),
            list(state.path_state_valid), state.load_pose_valid,
            state.load_path_state_valid)
        # Ignore only the estimator's first half-second of startup. Checking
        # just the final sample can miss periodic TCP delivery stalls and let
        # the state become invalid immediately after this gate passes.
        evaluation_start = cooperative[0][0] + min(
            0.5, args.observe_seconds * 0.1)
        evaluated = [
            message for received, message in cooperative
            if received >= evaluation_start]
        if not evaluated:
            evaluated = [state]
        stamp_spreads = []
        missing_stamp_samples = 0
        invalid_state_samples = 0
        wrong_authority_samples = 0
        for observed in evaluated:
            fully_valid = (
                all(observed.robot_pose_valid) and
                all(observed.support_pose_valid) and
                all(observed.path_state_valid) and
                observed.load_pose_valid and
                observed.load_path_state_valid)
            stamps = [
                stamp.to_sec() for stamp in observed.robot_pose_stamp
                if stamp.to_sec() > 0.0]
            # Invalid snapshots may intentionally carry latest diagnostic
            # stamps from unmatched streams. They are rejected by the
            # validity/authority gates and must not be misreported as a host
            # clock fault.
            if fully_valid and len(stamps) == ROBOT_COUNT:
                stamp_spreads.append(max(stamps) - min(stamps))
            elif fully_valid:
                missing_stamp_samples += 1
            if not fully_valid:
                invalid_state_samples += 1
            if args.require_fused_cooperative_state and not (
                    observed.header.frame_id == "three_car_path" and
                    all(source == CooperativeState.SOURCE_FUSED
                        for source in observed.robot_localization_source) and
                    observed.load_localization_source ==
                    CooperativeState.SOURCE_FUSED):
                wrong_authority_samples += 1
        if stamp_spreads:
            maximum_observed_spread = max(stamp_spreads)
            rospy.loginfo(
                "cooperative odometry stamp spread: latest %.6f s, "
                "maximum observed %.6f s over %d samples",
                stamp_spreads[-1], maximum_observed_spread, len(evaluated))
            if maximum_observed_spread > args.maximum_stamp_spread:
                errors.append(
                    "maximum observed odometry stamp spread {:.6f} s "
                    "exceeds {:.6f} s; synchronise host clocks or diagnose "
                    "network delivery jitter".format(
                        maximum_observed_spread,
                        args.maximum_stamp_spread))
        if missing_stamp_samples:
            errors.append(
                "{} cooperative samples do not contain three non-zero "
                "odometry timestamps".format(missing_stamp_samples))
        if args.require_valid_state and invalid_state_samples:
            errors.append(
                "{} of {} cooperative samples were not fully valid, "
                "including the virtual load fit".format(
                    invalid_state_samples, len(evaluated)))
        if args.require_fused_cooperative_state and wrong_authority_samples:
            errors.append(
                "{} of {} cooperative samples did not use three_car_path "
                "with fused robot/load authority".format(
                    wrong_authority_samples, len(evaluated)))

    if errors:
        for error in errors:
            rospy.logerr("%s", error)
        rospy.logerr("THREE-CAR READ-ONLY GATE: FAILED (%d issue(s))",
                     len(errors))
        return 1

    rospy.loginfo("THREE-CAR READ-ONLY GATE: PASSED")
    if not args.require_valid_state:
        rospy.loginfo(
            "Topology-only pass: motion remains prohibited until a second "
            "pass with --require-valid-state")
    return 0


if __name__ == "__main__":
    sys.exit(main())
