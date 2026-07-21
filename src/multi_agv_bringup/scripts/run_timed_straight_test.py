#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

import rosgraph
import rospy

from agv_msgs.msg import ChassisCommand


MAX_TEST_SPEED_MPS = 0.05
MAX_STANDARD_TEST_DURATION_SECONDS = 5.0
MAX_EXTENDED_TEST_DURATION_SECONDS = 20.0
EXTENDED_TEST_SPEED_MPS = 0.05
SUBSCRIBER_WAIT_SECONDS = 5.0
SUBSCRIBER_SETTLE_SECONDS = 0.5
STOP_REPEAT_COUNT = 3
STOP_REPEAT_INTERVAL_SECONDS = 0.05


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one bounded low-speed straight-line chassis test."
    )
    parser.add_argument("--robot-id", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--speed", type=float, required=True, help="m/s")
    parser.add_argument("--duration", type=float, required=True, help="seconds")
    parser.add_argument("--command-seq", type=int, default=1000)
    parser.add_argument("--experiment-id", default="single_car_floor_gate")
    parser.add_argument(
        "--required-command-subscribers",
        type=int,
        default=1,
        help=(
            "number of chassis-command subscribers required before motion; "
            "use 2 when the chassis controller and rosbag must both be connected"
        ),
    )
    parser.add_argument(
        "--confirm-wheels-on-floor",
        action="store_true",
        help="required acknowledgement that the floor test area is secured",
    )
    parser.add_argument(
        "--confirm-extended-floor-test",
        action="store_true",
        help=(
            "required for a test longer than 5 seconds; extended tests are "
            "limited to exactly 0.05 m/s and at most 20 seconds"
        ),
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if not args.confirm_wheels_on_floor:
        parser.error("--confirm-wheels-on-floor is required")
    if not 0.0 < abs(args.speed) <= MAX_TEST_SPEED_MPS:
        parser.error(
            "absolute speed must be greater than 0 and at most "
            f"{MAX_TEST_SPEED_MPS:.2f} m/s"
        )
    if not 0.0 < args.duration <= MAX_EXTENDED_TEST_DURATION_SECONDS:
        parser.error(
            "duration must be greater than 0 and at most "
            f"{MAX_EXTENDED_TEST_DURATION_SECONDS:.1f} seconds"
        )
    if not 1 <= args.command_seq <= 0xFFFFFFFB:
        parser.error("command sequence must leave room for three stop commands")
    if not 1 <= args.required_command_subscribers <= 8:
        parser.error("required command subscribers must be between 1 and 8")
    if args.duration > MAX_STANDARD_TEST_DURATION_SECONDS:
        if not args.confirm_extended_floor_test:
            parser.error(
                "--confirm-extended-floor-test is required when duration "
                f"exceeds {MAX_STANDARD_TEST_DURATION_SECONDS:.1f} seconds"
            )
        if not math.isclose(
            abs(args.speed), EXTENDED_TEST_SPEED_MPS, rel_tol=0.0, abs_tol=1e-9
        ):
            parser.error("extended tests require an absolute speed of 0.05 m/s")
        if args.required_command_subscribers < 2:
            parser.error(
                "extended tests require --required-command-subscribers 2 or more"
            )
    return args


def make_command(args, sequence, speed, method_id):
    command = ChassisCommand()
    command.header.stamp = rospy.Time.now()
    command.robot_id = args.robot_id
    command.command_seq = sequence
    command.control_mode = 1
    command.linear_velocity_reference = speed
    command.angular_velocity_reference = 0.0
    command.wheel_linear_velocity_left_raw = speed
    command.wheel_linear_velocity_right_raw = speed
    command.experiment_id = args.experiment_id
    command.method_id = method_id
    return command


def main():
    args = parse_args()
    topic = f"/agv{args.robot_id}/chassis_command"
    rospy.init_node("timed_straight_test", anonymous=True, disable_signals=True)

    master = rosgraph.Master(rospy.get_name())
    publishers, _, _ = master.getSystemState()
    existing_publishers = dict(publishers).get(topic, [])
    if existing_publishers:
        rospy.logerr(
            "Refusing motion because %s already has publisher(s): %s",
            topic,
            ", ".join(existing_publishers),
        )
        return 2

    publisher = rospy.Publisher(topic, ChassisCommand, queue_size=1, latch=False)

    stop_requested = False
    received_signal = None

    def request_stop(signum, _frame):
        nonlocal stop_requested, received_signal
        stop_requested = True
        received_signal = signum

    for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signal_number, request_stop)

    wait_deadline = time.monotonic() + SUBSCRIBER_WAIT_SECONDS
    while (
        publisher.get_num_connections() < args.required_command_subscribers
        and time.monotonic() < wait_deadline
    ):
        if stop_requested:
            return 130
        time.sleep(0.05)
    connected_subscribers = publisher.get_num_connections()
    if connected_subscribers < args.required_command_subscribers:
        rospy.logerr(
            "Only %d/%d required subscriber(s) connected on %s; motion not sent",
            connected_subscribers,
            args.required_command_subscribers,
            topic,
        )
        return 2

    rospy.loginfo(
        "%d required subscriber(s) connected on %s; settling for %.1f seconds",
        connected_subscribers,
        topic,
        SUBSCRIBER_SETTLE_SECONDS,
    )
    settle_deadline = time.monotonic() + SUBSCRIBER_SETTLE_SECONDS
    while time.monotonic() < settle_deadline:
        if stop_requested:
            return 130
        connected_subscribers = publisher.get_num_connections()
        if connected_subscribers < args.required_command_subscribers:
            rospy.logerr(
                "Subscriber count dropped to %d/%d during settling; motion not sent",
                connected_subscribers,
                args.required_command_subscribers,
            )
            return 2
        time.sleep(0.05)

    motion_started = False
    try:
        test_kind = (
            "extended" if args.duration > MAX_STANDARD_TEST_DURATION_SECONDS
            else "standard"
        )
        rospy.logwarn(
            "Starting bounded %s floor test: robot=%d speed=%+.3f m/s duration=%.2f s",
            test_kind,
            args.robot_id,
            args.speed,
            args.duration,
        )
        motion_started = True
        publisher.publish(
            make_command(args, args.command_seq, args.speed, "timed_straight_motion")
        )

        motion_deadline = time.monotonic() + args.duration
        while not stop_requested and time.monotonic() < motion_deadline:
            time.sleep(0.02)
    finally:
        if motion_started:
            for offset in range(1, STOP_REPEAT_COUNT + 1):
                sequence = args.command_seq + offset
                publisher.publish(
                    make_command(args, sequence, 0.0, "timed_straight_stop")
                )
                rospy.logwarn("Published zero-speed stop command_seq=%d", sequence)
                time.sleep(STOP_REPEAT_INTERVAL_SECONDS)

    if received_signal is not None:
        rospy.logwarn("Test interrupted by signal %d; stop commands were sent", received_signal)
        return 130
    rospy.loginfo("Timed floor test completed and stop commands were sent")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        if rospy.core.is_initialized() and not rospy.is_shutdown():
            rospy.signal_shutdown("timed straight test finished")
