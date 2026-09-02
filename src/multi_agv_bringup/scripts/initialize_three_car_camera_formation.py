#!/usr/bin/env python3
"""Sequentially align three AGVs into a camera-fused 0.40 m triangle."""

import math
import signal
import sys
import time

import rosgraph
import rospy
from agv_msgs.msg import ChassisCommand, ChassisFeedback
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from std_msgs.msg import String


ROBOT_COUNT = 3


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_quaternion(value):
    return Quaternion(z=math.sin(0.5 * value), w=math.cos(0.5 * value))


def distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


class FormationInitializer:
    def __init__(self):
        root = "~three_car_camera_formation_init/"
        self.rate_hz = rospy.get_param(root + "publish_rate", 50.0)
        self.subscriber_wait = rospy.get_param(
            root + "subscriber_wait_seconds", 10.0)
        self.maximum_pose_age = rospy.get_param(root + "maximum_pose_age", 0.22)
        self.maximum_feedback_age = rospy.get_param(
            root + "maximum_feedback_age", 0.25)
        self.minimum_voltage = rospy.get_param(
            root + "minimum_battery_voltage", 10.0)
        self.minimum_voltage_duration = rospy.get_param(
            root + "minimum_battery_voltage_duration", 0.5)
        self.side = rospy.get_param(root + "geometry/side_length", 0.40)
        self.support_x = rospy.get_param(
            root + "geometry/base_to_support_x", -0.01783)
        self.final_side_tolerance = rospy.get_param(
            root + "geometry/final_side_tolerance", 0.015)
        self.final_heading_tolerance = rospy.get_param(
            root + "geometry/final_heading_tolerance", 0.05236)
        self.max_linear = rospy.get_param(
            root + "motion/maximum_linear_speed", 0.025)
        self.max_angular = rospy.get_param(
            root + "motion/maximum_angular_speed", 0.25)
        self.max_wheel = rospy.get_param(
            root + "motion/maximum_wheel_speed", 0.05)
        self.wheel_separation = rospy.get_param(
            root + "motion/wheel_separation",
            [0.135484339, 0.139284482, 0.136802843])
        self.realign_threshold = rospy.get_param(
            root + "motion/heading_realign_threshold", 0.30)
        self.near_target_distance = rospy.get_param(
            root + "motion/near_target_distance", 0.06)
        self.near_target_speed = rospy.get_param(
            root + "motion/near_target_speed", 0.010)
        self.docking_distance = rospy.get_param(
            root + "motion/docking_distance", 0.03)
        self.docking_speed = rospy.get_param(
            root + "motion/docking_speed", 0.005)
        self.position_tolerance = rospy.get_param(
            root + "motion/position_tolerance", 0.012)
        self.refinement_position_tolerance = rospy.get_param(
            root + "motion/refinement_position_tolerance", 0.006)
        self.maximum_refinement_passes = rospy.get_param(
            root + "motion/maximum_refinement_passes", 2)
        self.final_heading_recovery_distance = rospy.get_param(
            root + "motion/final_heading_recovery_distance", 0.025)
        self.heading_tolerance = rospy.get_param(
            root + "motion/heading_tolerance", 0.03491)
        self.stable_samples_required = rospy.get_param(
            root + "motion/stable_samples", 15)
        self.maximum_seconds_per_robot = rospy.get_param(
            root + "motion/maximum_seconds_per_robot", 45.0)
        self.maximum_no_progress_seconds = rospy.get_param(
            root + "motion/maximum_no_progress_seconds", 12.0)
        self.maximum_initial_target_distance = rospy.get_param(
            root + "safety/maximum_initial_target_distance", 0.65)
        self.minimum_separation = rospy.get_param(
            root + "safety/minimum_robot_separation", 0.22)
        self.maximum_stationary_position_drift = rospy.get_param(
            root + "safety/maximum_stationary_position_drift", 0.025)
        self.maximum_stationary_heading_drift = rospy.get_param(
            root + "safety/maximum_stationary_heading_drift", 0.05236)
        self.maximum_actual_wheel_speed = rospy.get_param(
            root + "safety/maximum_actual_wheel_speed", 0.105)
        self.stopped_wheel_tolerance = rospy.get_param(
            root + "safety/stopped_wheel_tolerance", 0.01)
        self.required_subscribers = rospy.get_param(
            root + "safety/required_command_subscribers", 1)
        self.authorized = rospy.get_param(
            root + "hardware_execution_authorized", False)
        self.confirm_area = rospy.get_param("~confirm_test_area_clear", False)
        self.confirm_floor = rospy.get_param("~confirm_wheels_on_floor", False)
        self.confirm_auto = rospy.get_param(
            "~confirm_automatic_formation", False)

        self._validate_parameters()
        self.poses = [None] * ROBOT_COUNT
        self.pose_received = [0.0] * ROBOT_COUNT
        self.feedback = [None] * ROBOT_COUNT
        self.feedback_received = [0.0] * ROBOT_COUNT
        self.low_voltage_since = [None] * ROBOT_COUNT
        self.publishers = []
        self.command_sequence = [50000] * ROBOT_COUNT
        self.stop_requested = False
        self.initial = None
        self.targets = None
        self.target_reached = [True, False, False]
        self.result_publisher = rospy.Publisher(
            "/multi_agv/formation_init/result", String,
            queue_size=1, latch=True)
        self.target_publisher = rospy.Publisher(
            "/multi_agv/formation_init/target_poses", PoseArray,
            queue_size=1, latch=True)

        self._reject_competing_publishers()
        for index in range(ROBOT_COUNT):
            self.publishers.append(rospy.Publisher(
                f"/agv{index + 1}/chassis_command",
                ChassisCommand, queue_size=2))
            rospy.Subscriber(
                f"/pose_provider/agv{index + 1}/base_pose_fused",
                PoseStamped,
                lambda message, robot=index: self._pose_callback(robot, message),
                queue_size=20, tcp_nodelay=True)
            rospy.Subscriber(
                f"/agv{index + 1}/chassis_feedback",
                ChassisFeedback,
                lambda message, robot=index: self._feedback_callback(robot, message),
                queue_size=50, tcp_nodelay=True)

    def _validate_parameters(self):
        numeric_positive = (
            self.rate_hz, self.subscriber_wait, self.maximum_pose_age,
            self.maximum_feedback_age, self.minimum_voltage,
            self.minimum_voltage_duration, self.side,
            self.final_side_tolerance, self.final_heading_tolerance,
            self.max_linear, self.max_angular, self.max_wheel,
            self.realign_threshold, self.near_target_distance,
            self.near_target_speed, self.docking_distance,
            self.docking_speed, self.position_tolerance,
            self.refinement_position_tolerance,
            self.final_heading_recovery_distance,
            self.heading_tolerance, self.maximum_seconds_per_robot,
            self.maximum_no_progress_seconds,
            self.maximum_initial_target_distance, self.minimum_separation,
            self.maximum_stationary_position_drift,
            self.maximum_stationary_heading_drift,
            self.maximum_actual_wheel_speed, self.stopped_wheel_tolerance)
        if not self.authorized or not self.confirm_area or not self.confirm_floor \
                or not self.confirm_auto:
            raise RuntimeError("all formation initialization authorizations are required")
        if any(not math.isfinite(value) or value <= 0.0
               for value in numeric_positive):
            raise RuntimeError("formation initialization parameters must be positive")
        if len(self.wheel_separation) != ROBOT_COUNT or any(
                not math.isfinite(value) or value <= 0.0
                for value in self.wheel_separation):
            raise RuntimeError("wheel_separation must contain three positive values")
        if self.max_wheel > 0.06 or self.max_linear > 0.03 or \
                self.max_angular > 0.30:
            raise RuntimeError("formation initialization motion bounds exceed software caps")
        if not self.position_tolerance < self.docking_distance < \
                self.near_target_distance or \
                self.docking_speed > self.near_target_speed or \
                self.near_target_speed > self.max_linear:
            raise RuntimeError("formation docking thresholds or speeds are inconsistent")
        if not self.position_tolerance < \
                self.final_heading_recovery_distance < self.minimum_separation:
            raise RuntimeError("final-heading recovery hysteresis is inconsistent")
        if self.refinement_position_tolerance >= self.position_tolerance or \
                2.0 * self.refinement_position_tolerance >= \
                self.final_side_tolerance:
            raise RuntimeError(
                "formation refinement tolerance cannot guarantee final sides")
        if self.minimum_separation < 0.20 or self.minimum_voltage < 10.0:
            raise RuntimeError("formation initialization safety bounds are too low")
        if not 1 <= self.required_subscribers <= 4 or \
                not 1 <= self.stable_samples_required <= 100 or \
                not 1 <= self.maximum_refinement_passes <= 5:
            raise RuntimeError("invalid subscriber or stable-sample requirement")

    def _reject_competing_publishers(self):
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        by_topic = dict(publishers)
        for index in range(ROBOT_COUNT):
            topic = f"/agv{index + 1}/chassis_command"
            if by_topic.get(topic):
                raise RuntimeError(
                    f"refusing formation initialization; {topic} already has "
                    f"publisher(s): {', '.join(by_topic[topic])}")

    def _pose_callback(self, index, message):
        if not message.header.frame_id.startswith("world@") or \
                message.header.stamp.is_zero():
            return
        x = message.pose.position.x
        y = message.pose.position.y
        heading = quaternion_yaw(message.pose.orientation)
        if not all(math.isfinite(value) for value in (x, y, heading)):
            return
        self.poses[index] = (x, y, heading, message.header.frame_id)
        self.pose_received[index] = time.monotonic()

    def _feedback_callback(self, index, message):
        if message.robot_id != index + 1:
            return
        self.feedback[index] = message
        self.feedback_received[index] = time.monotonic()

    def _support(self, pose):
        return (pose[0] + self.support_x * math.cos(pose[2]),
                pose[1] + self.support_x * math.sin(pose[2]))

    def _base_from_support(self, support, heading):
        return (support[0] - self.support_x * math.cos(heading),
                support[1] - self.support_x * math.sin(heading), heading)

    def _fresh_inputs(self):
        now = time.monotonic()
        if any(value is None for value in self.poses + self.feedback):
            raise RuntimeError("waiting for all three fused poses and chassis feedback")
        pose_ages = [now - stamp for stamp in self.pose_received]
        stale_poses = [
            f"agv{index + 1}={age:.3f}s"
            for index, age in enumerate(pose_ages)
            if age > self.maximum_pose_age
        ]
        if stale_poses:
            raise RuntimeError(
                "camera-fused pose stale: " + ", ".join(stale_poses) +
                f" (limit={self.maximum_pose_age:.3f}s)")
        feedback_ages = [now - stamp for stamp in self.feedback_received]
        stale_feedback = [
            f"agv{index + 1}={age:.3f}s"
            for index, age in enumerate(feedback_ages)
            if age > self.maximum_feedback_age
        ]
        if stale_feedback:
            raise RuntimeError(
                "chassis feedback stale: " + ", ".join(stale_feedback) +
                f" (limit={self.maximum_feedback_age:.3f}s)")
        frames = {pose[3] for pose in self.poses}
        if len(frames) != 1:
            raise RuntimeError("three fused poses do not share one calibration epoch")
        for index, feedback in enumerate(self.feedback):
            if not math.isfinite(feedback.battery_voltage):
                raise RuntimeError(f"agv{index + 1} battery voltage is invalid")
            if feedback.battery_voltage < self.minimum_voltage:
                if self.low_voltage_since[index] is None:
                    self.low_voltage_since[index] = now
                    rospy.logwarn(
                        "agv%d voltage %.3f V is below %.3f V; waiting %.2f s debounce",
                        index + 1, feedback.battery_voltage,
                        self.minimum_voltage, self.minimum_voltage_duration)
                elif now - self.low_voltage_since[index] >= \
                        self.minimum_voltage_duration:
                    raise RuntimeError(
                        f"agv{index + 1} battery remained below the bound "
                        f"for {self.minimum_voltage_duration:.2f} s")
            else:
                self.low_voltage_since[index] = None
            if feedback.control_loop_overrun:
                raise RuntimeError(f"agv{index + 1} reported control-loop overrun")
            actual = max(abs(feedback.wheel_linear_velocity_left_actual),
                         abs(feedback.wheel_linear_velocity_right_actual))
            if actual > self.maximum_actual_wheel_speed:
                raise RuntimeError(f"agv{index + 1} actual wheel speed exceeds bound")

    def _wait_ready(self):
        deadline = time.monotonic() + self.subscriber_wait
        last_reason = "inputs unavailable"
        stopped_samples = 0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                self._fresh_inputs()
                wheels_stopped = all(
                    max(abs(feedback.wheel_linear_velocity_left_actual),
                        abs(feedback.wheel_linear_velocity_right_actual)) <=
                    self.stopped_wheel_tolerance
                    for feedback in self.feedback)
                stopped_samples = stopped_samples + 1 if wheels_stopped else 0
                subscribers_ready = all(
                    publisher.get_num_connections() >= self.required_subscribers
                    for publisher in self.publishers)
                if subscribers_ready and stopped_samples >= 10:
                    self.command_sequence = [
                        max(50000, feedback.command_seq_applied + 1)
                        for feedback in self.feedback]
                    return
                if not subscribers_ready:
                    last_reason = "command subscribers are not connected"
                elif not wheels_stopped:
                    last_reason = "all six wheels must be stopped before initialization"
                else:
                    last_reason = "waiting for stable all-six-wheel stop confirmation"
            except RuntimeError as error:
                last_reason = str(error)
            time.sleep(0.05)
        raise RuntimeError(f"formation initialization readiness timeout: {last_reason}")

    def _compute_targets(self):
        robot1 = self.poses[0]
        support1 = self._support(robot1)
        heading = robot1[2]
        forward = (math.cos(heading), math.sin(heading))
        left = (-math.sin(heading), math.cos(heading))
        altitude = math.sqrt(3.0) * 0.5 * self.side
        support2 = (support1[0] - altitude * forward[0] +
                    0.5 * self.side * left[0],
                    support1[1] - altitude * forward[1] +
                    0.5 * self.side * left[1])
        support3 = (support1[0] - altitude * forward[0] -
                    0.5 * self.side * left[0],
                    support1[1] - altitude * forward[1] -
                    0.5 * self.side * left[1])
        return [robot1,
                self._base_from_support(support2, heading),
                self._base_from_support(support3, heading)]

    def _publish_targets(self):
        message = PoseArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.initial[0][3]
        for target in self.targets:
            pose = Pose()
            pose.position.x = target[0]
            pose.position.y = target[1]
            pose.orientation = yaw_quaternion(target[2])
            message.poses.append(pose)
        self.target_publisher.publish(message)

    def _command(self, index, linear, angular, method):
        half_track = 0.5 * self.wheel_separation[index]
        left = linear - half_track * angular
        right = linear + half_track * angular
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel:
            scale = self.max_wheel / peak
            left *= scale
            right *= scale
            linear = 0.5 * (left + right)
            angular = (right - left) / self.wheel_separation[index]
        self.command_sequence[index] += 1
        command = ChassisCommand()
        command.header.stamp = rospy.Time.now()
        command.robot_id = index + 1
        command.command_seq = self.command_sequence[index]
        command.control_mode = 1
        command.linear_velocity_reference = linear
        command.angular_velocity_reference = angular
        command.wheel_linear_velocity_left_raw = left
        command.wheel_linear_velocity_right_raw = right
        command.experiment_id = "three_car_camera_formation_init"
        command.method_id = method
        self.publishers[index].publish(command)

    def _publish_all(self, active=None, linear=0.0, angular=0.0,
                     method="FORMATION_INIT_STOP"):
        for index in range(ROBOT_COUNT):
            if index == active:
                self._command(index, linear, angular, method)
            else:
                self._command(index, 0.0, 0.0, "FORMATION_INIT_HOLD")

    def _check_separation(self):
        supports = [self._support(pose) for pose in self.poses]
        for first, second in ((0, 1), (0, 2), (1, 2)):
            value = distance(supports[first], supports[second])
            if value < self.minimum_separation:
                raise RuntimeError(
                    f"agv{first + 1}/agv{second + 1} separation {value:.3f} m "
                    f"is below {self.minimum_separation:.3f} m")

    def _check_stationary(self, active):
        for index in range(ROBOT_COUNT):
            if index == active:
                continue
            reference = (self.targets[index] if self.target_reached[index]
                         else self.initial[index])
            if distance(self.poses[index], reference) > \
                    self.maximum_stationary_position_drift or \
                    abs(wrap(self.poses[index][2] - reference[2])) > \
                    self.maximum_stationary_heading_drift:
                raise RuntimeError(f"stationary agv{index + 1} drifted from its hold pose")

    def _move_robot(self, index, position_tolerance=None,
                    phase="FORMATION_INIT"):
        target = self.targets[index]
        acceptance = (self.position_tolerance if position_tolerance is None
                      else position_tolerance)
        initial_distance = distance(self.poses[index], target)
        if initial_distance > self.maximum_initial_target_distance:
            raise RuntimeError(
                f"agv{index + 1} target is {initial_distance:.3f} m away; "
                f"manually place it within {self.maximum_initial_target_distance:.3f} m")
        rospy.logwarn(
            "Moving agv%d sequentially: target distance %.3f m, maximum %.3f m/s",
            index + 1, initial_distance, self.max_linear)
        deadline = time.monotonic() + self.maximum_seconds_per_robot
        best_distance = initial_distance
        last_progress = time.monotonic()
        translation_started = False
        final_heading_phase = False
        stable = 0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self.stop_requested:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"agv{index + 1} formation move timed out")
            self._fresh_inputs()
            self._check_separation()
            self._check_stationary(index)
            pose = self.poses[index]
            dx = target[0] - pose[0]
            dy = target[1] - pose[1]
            position_error = math.hypot(dx, dy)
            final_heading_error = wrap(target[2] - pose[2])
            if position_error < best_distance - 0.003:
                best_distance = position_error
                last_progress = time.monotonic()

            # Latch final-heading mode once position first enters tolerance.
            # Robot2 has a measured sub-mm/s translation bias while turning;
            # hysteresis prevents 1--2 mm threshold crossings from switching
            # between position and heading modes every control cycle.
            if not final_heading_phase and \
                    position_error <= acceptance:
                final_heading_phase = True
                stable = 0
                last_progress = time.monotonic()
                rospy.loginfo(
                    "agv%d latched final-heading phase at %.4f m",
                    index + 1, position_error)

            if final_heading_phase:
                if position_error > self.final_heading_recovery_distance:
                    final_heading_phase = False
                    stable = 0
                    best_distance = position_error
                    last_progress = time.monotonic()
                    self._publish_all(method=phase + "_POSITION_RECOVERY")
                    rospy.logwarn(
                        "agv%d left final-heading phase: position drift %.4f m",
                        index + 1, position_error)
                    rate.sleep()
                    continue
                if abs(final_heading_error) <= self.heading_tolerance:
                    if position_error <= acceptance:
                        stable += 1
                        self._publish_all(method=phase + "_SETTLE")
                        if stable >= self.stable_samples_required:
                            rospy.loginfo(
                                "agv%d aligned: position_error=%.4f m heading_error=%.2f deg",
                                index + 1, position_error,
                                math.degrees(final_heading_error))
                            return
                    else:
                        # Heading is complete but position is outside its
                        # tighter acceptance bound. Correct position once more
                        # from the now-correct vehicle orientation.
                        final_heading_phase = False
                        stable = 0
                        best_distance = position_error
                        last_progress = time.monotonic()
                        self._publish_all(
                            method=phase + "_POSITION_RECOVERY")
                else:
                    stable = 0
                    angular = clamp(1.8 * final_heading_error,
                                    self.max_angular)
                    self._publish_all(index, 0.0, angular,
                                      phase + "_FINAL_HEADING")
                rate.sleep()
                continue

            if translation_started and time.monotonic() - last_progress > \
                    self.maximum_no_progress_seconds:
                raise RuntimeError(
                    f"agv{index + 1} failed to make position progress; "
                    f"best error was {best_distance:.3f} m")
            stable = 0
            bearing = math.atan2(dy, dx)
            bearing_error = wrap(bearing - pose[2])
            if abs(bearing_error) <= 0.5 * math.pi:
                direction = 1.0
                travel_heading_error = bearing_error
            else:
                direction = -1.0
                travel_heading_error = wrap(
                    bearing_error - math.copysign(math.pi, bearing_error))
            if abs(travel_heading_error) > self.realign_threshold:
                angular = clamp(1.8 * travel_heading_error, self.max_angular)
                self._publish_all(index, 0.0, angular,
                                  phase + "_APPROACH_ALIGN")
            else:
                translation_started = True
                speed_limit = self.max_linear
                if position_error <= self.near_target_distance:
                    speed_limit = self.near_target_speed
                if position_error <= self.docking_distance:
                    speed_limit = self.docking_speed
                linear = direction * min(speed_limit, 0.55 * position_error)
                angular = clamp(2.2 * travel_heading_error,
                                0.8 * self.max_angular)
                self._publish_all(index, linear, angular,
                                  phase + "_APPROACH_FORWARD"
                                  if direction > 0.0 else
                                  phase + "_APPROACH_REVERSE")
            rate.sleep()
        raise RuntimeError("formation initialization interrupted")

    def _formation_errors(self):
        supports = [self._support(pose) for pose in self.poses]
        sides = [distance(supports[0], supports[1]),
                 distance(supports[0], supports[2]),
                 distance(supports[1], supports[2])]
        headings = [abs(wrap(pose[2] - self.poses[0][2]))
                    for pose in self.poses[1:]]
        return sides, headings

    def _final_check(self, raise_on_failure=True):
        sides, headings = self._formation_errors()
        rospy.loginfo(
            "Final support distances: d12=%.4f d13=%.4f d23=%.4f m",
            sides[0], sides[1], sides[2])
        if any(abs(value - self.side) > self.final_side_tolerance
               for value in sides):
            if raise_on_failure:
                raise RuntimeError(
                    "final support triangle exceeds side tolerance")
            return False
        if any(value > self.final_heading_tolerance for value in headings):
            if raise_on_failure:
                raise RuntimeError(
                    "final vehicle headings exceed parallel tolerance")
            return False
        return True

    def _refine_final_triangle(self):
        if self._final_check(raise_on_failure=False):
            return
        for refinement_pass in range(1, self.maximum_refinement_passes + 1):
            rospy.logwarn(
                "Final triangle is outside tolerance; refinement pass %d/%d "
                "targets agv2/agv3 within %.1f mm",
                refinement_pass, self.maximum_refinement_passes,
                1000.0 * self.refinement_position_tolerance)
            for index in (1, 2):
                self._move_robot(
                    index,
                    position_tolerance=self.refinement_position_tolerance,
                    phase="FORMATION_REFINE")
                self.target_reached[index] = True
            self._fresh_inputs()
            if self._final_check(raise_on_failure=False):
                rospy.loginfo(
                    "Final support triangle accepted after refinement pass %d",
                    refinement_pass)
                return
        self._final_check(raise_on_failure=True)

    def _stop_and_confirm(self):
        deadline = time.monotonic() + 2.0
        consecutive = 0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish_all()
            if all(feedback is not None and
                   max(abs(feedback.wheel_linear_velocity_left_actual),
                       abs(feedback.wheel_linear_velocity_right_actual)) <=
                   self.stopped_wheel_tolerance
                   for feedback in self.feedback):
                consecutive += 1
                if consecutive >= 10:
                    return True
            else:
                consecutive = 0
            rate.sleep()
        return False

    def run(self):
        self._wait_ready()
        self._fresh_inputs()
        self._check_separation()
        self.initial = list(self.poses)
        self.targets = self._compute_targets()
        self._publish_targets()
        for index in (1, 2):
            self._move_robot(index)
            self.target_reached[index] = True
        self._fresh_inputs()
        self._refine_final_triangle()
        if not self._stop_and_confirm():
            raise RuntimeError("all-six-wheel stop confirmation failed")
        rospy.set_param("/multi_agv/formation_init/result_code", 0)
        self.result_publisher.publish(String(data="FORMATION_CONFIRMED"))
        rospy.loginfo("Three-car camera formation initialization completed")
        rospy.sleep(0.25)

    def stop(self):
        self.stop_requested = True
        try:
            for _ in range(10):
                self._publish_all()
                time.sleep(0.03)
        except Exception:
            pass


def main():
    rospy.init_node("three_car_camera_formation_init", disable_signals=True)
    node = None
    interrupted = {"value": False}

    def request_stop(_signum, _frame):
        interrupted["value"] = True
        if node is not None:
            node.stop_requested = True

    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, request_stop)
    try:
        node = FormationInitializer()
        node.run()
        return 0
    except Exception as error:
        rospy.logerr("Formation initialization aborted: %s", error)
        rospy.set_param("/multi_agv/formation_init/result_code", 1)
        if node is not None:
            node.result_publisher.publish(String(data=f"ABORTED: {error}"))
            rospy.sleep(0.25)
        return 130 if interrupted["value"] else 1
    finally:
        if node is not None:
            node.stop()
        rospy.signal_shutdown("formation initialization finished")


if __name__ == "__main__":
    sys.exit(main())
