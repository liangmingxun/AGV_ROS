#!/usr/bin/env python3
"""Confirm a simultaneous, sustained stop from all three chassis."""

import argparse
import threading
import time

import rospy

from agv_msgs.msg import ChassisFeedback


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--speed-threshold", type=float, default=0.01)
    parser.add_argument("--stable-seconds", type=float, default=0.10)
    parser.add_argument("--maximum-age", type=float, default=0.20)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("wait_three_car_wheel_stop", anonymous=True)
    lock = threading.Lock()
    samples = {}

    def receive(message, robot_id):
        with lock:
            samples[robot_id] = (
                time.monotonic(),
                message.wheel_linear_velocity_left_actual,
                message.wheel_linear_velocity_right_actual,
            )

    subscribers = [
        rospy.Subscriber(
            "/agv{}/chassis_feedback".format(robot_id),
            ChassisFeedback,
            receive,
            callback_args=robot_id,
            queue_size=10,
        )
        for robot_id in (1, 2, 3)
    ]
    start = time.monotonic()
    stable_since = None
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        now = time.monotonic()
        with lock:
            snapshot = dict(samples)
        stopped = len(snapshot) == 3 and all(
            now - snapshot[robot_id][0] <= args.maximum_age
            and abs(snapshot[robot_id][1]) <= args.speed_threshold
            and abs(snapshot[robot_id][2]) <= args.speed_threshold
            for robot_id in (1, 2, 3)
        )
        if stopped:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= args.stable_seconds:
                rospy.loginfo(
                    "All six wheels remained within %.3f m/s for %.3f s",
                    args.speed_threshold,
                    args.stable_seconds,
                )
                return 0
        else:
            stable_since = None
        if now - start >= args.timeout:
            rospy.logerr(
                "Wheel stop was not confirmed within %.1f s; latest=%s",
                args.timeout,
                snapshot,
            )
            return 1
        rate.sleep()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
