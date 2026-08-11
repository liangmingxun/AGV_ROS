#!/usr/bin/env python3

"""Bounded camera/IMU calibration of wheel feedback and turn response.

The program deliberately writes candidate parameters only.  It never changes
the running chassis configuration or a repository YAML file.
"""

import argparse
import json
import math
import os
import signal
import statistics
import sys
import time

import rosgraph
import rospy
from agv_msgs.msg import ChassisCommand, ChassisFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64


MAX_WHEEL_COMMAND = 0.05
WARNING_ACTUAL_SPEED = 0.08
HARD_ACTUAL_SPEED = 0.09
EMERGENCY_ACTUAL_SPEED = 0.12
STATE_TIMEOUT = 0.15
COMMAND_RATE = 100.0
STOP_SETTLE_SECONDS = 0.60
STOP_TIMEOUT_SECONDS = 3.0
STOPPED_WHEEL_SPEED = 0.01


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("invalid pose quaternion")
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def median_pose(samples):
    if not samples:
        raise ValueError("pose window is empty")
    return (
        statistics.median(x[1] for x in samples),
        statistics.median(x[2] for x in samples),
        math.atan2(
            statistics.median(math.sin(x[3]) for x in samples),
            statistics.median(math.cos(x[3]) for x in samples),
        ),
    )


def integrate(samples, start, end, index):
    selected = [x for x in samples if start <= x[0] <= end]
    if len(selected) < 2:
        raise ValueError("insufficient wheel samples for integration")
    return sum(
        0.5 * (a[index] + b[index]) * (b[0] - a[0])
        for a, b in zip(selected, selected[1:])
    )


def solve_three_by_three(matrix, vector):
    augmented = [list(row) + [rhs] for row, rhs in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-12:
            raise ValueError("calibration normal matrix is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * reference
                for value, reference in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][3] for row in range(3)]


def least_squares(rows, observations):
    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0 for _ in range(3)]
    for row, observation in zip(rows, observations):
        for i in range(3):
            rhs[i] += row[i] * observation
            for j in range(3):
                normal[i][j] += row[i] * row[j]
    return solve_three_by_three(normal, rhs)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Automatically generate review-only wheel and turn calibration "
            "candidates using camera, IMU and STM32 wheel feedback."
        )
    )
    parser.add_argument("--robot-id", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--straight-speed", type=float, default=0.03)
    parser.add_argument("--straight-duration", type=float, default=7.0)
    parser.add_argument("--arc-linear-speed", type=float, default=0.025)
    parser.add_argument("--arc-angular-speed", type=float, default=0.12)
    parser.add_argument("--arc-duration", type=float, default=6.0)
    parser.add_argument("--nominal-wheel-separation", type=float, default=0.114)
    parser.add_argument("--minimum-confidence", type=float, default=0.40)
    parser.add_argument("--minimum-calibration-voltage", type=float, default=11.20)
    parser.add_argument("--required-command-subscribers", type=int, default=2)
    parser.add_argument("--confirm-test-area-clear", action="store_true", required=True)
    parser.add_argument("--confirm-wheels-on-floor", action="store_true", required=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not 1 <= args.cycles <= 3:
        parser.error("cycles must be between 1 and 3")
    if not 0.01 <= args.straight_speed <= 0.04:
        parser.error("straight speed must be between 0.01 and 0.04 m/s")
    if not 2.0 <= args.straight_duration <= 8.0:
        parser.error("straight duration must be between 2 and 8 seconds")
    if not 0.01 <= args.arc_linear_speed <= 0.035:
        parser.error("arc linear speed must be between 0.01 and 0.035 m/s")
    if not 0.05 <= args.arc_angular_speed <= 0.18:
        parser.error("arc angular speed must be between 0.05 and 0.18 rad/s")
    if not 2.0 <= args.arc_duration <= 7.0:
        parser.error("arc duration must be between 2 and 7 seconds")
    if not 0.08 <= args.nominal_wheel_separation <= 0.16:
        parser.error("nominal wheel separation must be between 0.08 and 0.16 m")
    if not 0.2 <= args.minimum_confidence <= 0.9:
        parser.error("minimum confidence must be between 0.2 and 0.9")
    if not 10.5 <= args.minimum_calibration_voltage <= 12.6:
        parser.error("minimum calibration voltage must be between 10.5 and 12.6 V")
    maximum_arc_wheel = args.arc_linear_speed + 0.5 * (
        args.nominal_wheel_separation * args.arc_angular_speed
    )
    if max(args.straight_speed, maximum_arc_wheel) > MAX_WHEEL_COMMAND:
        parser.error("a requested wheel command exceeds the 0.05 m/s calibration bound")
    return args


class CalibrationRunner:
    def __init__(self, args):
        self.args = args
        self.robot = f"agv{args.robot_id}"
        self.command_topic = f"/{self.robot}/chassis_command"
        self.feedback_topic = f"/{self.robot}/chassis_feedback"
        self.pose_topic = f"/pose_provider/{self.robot}/base_pose_raw"
        self.confidence_topic = f"/camera/world/{self.robot}_confidence"
        self.feedback = []
        self.poses = []
        self.confidences = []
        self.alive = False
        self.alive_time = 0.0
        self.latest_feedback_time = 0.0
        self.latest_pose_time = 0.0
        self.latest_confidence_time = 0.0
        self.latest_feedback = None
        self.stop_requested = False
        self.hard_speed_count = 0
        self.last_speed_feedback_sequence = None
        self.command_sequence = 1
        self.segments = []
        private_root = f"/{self.robot}/chassis_controller"
        self.active_scale_left = float(
            rospy.get_param(private_root + "/wheel_feedback_scale_left", 1.0)
        )
        self.active_scale_right = float(
            rospy.get_param(private_root + "/wheel_feedback_scale_right", 1.0)
        )
        if not 0.5 <= self.active_scale_left <= 1.5:
            raise ValueError("active left feedback scale is outside [0.5, 1.5]")
        if not 0.5 <= self.active_scale_right <= 1.5:
            raise ValueError("active right feedback scale is outside [0.5, 1.5]")
        self.publisher = rospy.Publisher(
            self.command_topic, ChassisCommand, queue_size=2, latch=False
        )
        rospy.Subscriber(
            self.feedback_topic, ChassisFeedback, self.feedback_callback,
            queue_size=200,
        )
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=100)
        rospy.Subscriber(
            self.confidence_topic, Float64, self.confidence_callback, queue_size=100
        )
        rospy.Subscriber("/vision/aruco/alive", Bool, self.alive_callback, queue_size=10)

    def feedback_callback(self, message):
        now = rospy.Time.now().to_sec()
        self.latest_feedback_time = now
        self.latest_feedback = message
        self.command_sequence = max(
            self.command_sequence, int(message.command_seq_applied) + 1
        )
        self.feedback.append(
            (
                now,
                float(message.wheel_linear_velocity_left_actual),
                float(message.wheel_linear_velocity_right_actual),
                float(message.angular_velocity_actual),
                float(message.battery_voltage),
            )
        )
        if len(self.feedback) > 30000:
            del self.feedback[:5000]

    def pose_callback(self, message):
        now = rospy.Time.now().to_sec()
        try:
            pose_yaw = quaternion_yaw(message.pose.orientation)
        except ValueError:
            return
        self.latest_pose_time = now
        self.poses.append(
            (now, float(message.pose.position.x), float(message.pose.position.y), pose_yaw)
        )
        if len(self.poses) > 10000:
            del self.poses[:2000]

    def confidence_callback(self, message):
        now = rospy.Time.now().to_sec()
        self.latest_confidence_time = now
        self.confidences.append((now, float(message.data)))
        if len(self.confidences) > 10000:
            del self.confidences[:2000]

    def alive_callback(self, message):
        self.alive = bool(message.data)
        self.alive_time = rospy.Time.now().to_sec()

    def check_state(self, require_stationary=False):
        now = rospy.Time.now().to_sec()
        if self.stop_requested or rospy.is_shutdown():
            raise RuntimeError("calibration interrupted")
        if not self.alive or now - self.alive_time > STATE_TIMEOUT:
            raise RuntimeError("ArUco UDP link is stale")
        if self.latest_feedback is None or now - self.latest_feedback_time > STATE_TIMEOUT:
            raise RuntimeError("chassis feedback is stale")
        if now - self.latest_pose_time > STATE_TIMEOUT:
            raise RuntimeError("camera base pose is stale")
        if now - self.latest_confidence_time > STATE_TIMEOUT:
            raise RuntimeError("camera confidence is stale")
        confidence = self.confidences[-1][1]
        if confidence < self.args.minimum_confidence:
            raise RuntimeError(
                f"camera confidence {confidence:.3f} is below "
                f"{self.args.minimum_confidence:.3f}"
            )
        if self.latest_feedback.battery_voltage < self.args.minimum_calibration_voltage:
            raise RuntimeError(
                "battery voltage is below the calibration-quality gate "
                f"{self.args.minimum_calibration_voltage:.2f} V"
            )
        maximum_actual = max(
            abs(self.latest_feedback.wheel_linear_velocity_left_actual),
            abs(self.latest_feedback.wheel_linear_velocity_right_actual),
        )
        feedback_sequence = int(self.latest_feedback.feedback_seq)
        if feedback_sequence != self.last_speed_feedback_sequence:
            self.last_speed_feedback_sequence = feedback_sequence
            if maximum_actual > EMERGENCY_ACTUAL_SPEED:
                raise RuntimeError("actual wheel speed exceeded 0.12 m/s")
            if maximum_actual > HARD_ACTUAL_SPEED:
                self.hard_speed_count += 1
            else:
                self.hard_speed_count = 0
        if self.hard_speed_count >= 2:
            raise RuntimeError("actual wheel speed exceeded 0.09 m/s twice")
        if require_stationary and maximum_actual > STOPPED_WHEEL_SPEED:
            raise RuntimeError("wheels are not stationary")

    def make_command(self, linear, angular, method):
        half_track = 0.5 * self.args.nominal_wheel_separation
        left = linear - half_track * angular
        right = linear + half_track * angular
        if max(abs(left), abs(right)) > MAX_WHEEL_COMMAND + 1.0e-12:
            raise RuntimeError("calibration wheel command exceeded bound")
        message = ChassisCommand()
        message.header.stamp = rospy.Time.now()
        message.robot_id = self.args.robot_id
        message.command_seq = self.command_sequence
        message.control_mode = 1
        message.linear_velocity_reference = linear
        message.angular_velocity_reference = angular
        message.wheel_linear_velocity_left_raw = left
        message.wheel_linear_velocity_right_raw = right
        message.experiment_id = "camera_wheel_turn_calibration"
        message.method_id = method
        return message

    def publish_for(self, linear, angular, duration, method):
        message = self.make_command(linear, angular, method)
        deadline = time.monotonic() + duration
        rate = rospy.Rate(COMMAND_RATE)
        while time.monotonic() < deadline:
            self.check_state()
            message.header.stamp = rospy.Time.now()
            self.publisher.publish(message)
            rate.sleep()

    def stop_and_settle(self):
        self.command_sequence += 1
        message = self.make_command(0.0, 0.0, "calibration_stop")
        deadline = time.monotonic() + STOP_TIMEOUT_SECONDS
        stationary_since = None
        rate = rospy.Rate(COMMAND_RATE)
        while time.monotonic() < deadline:
            message.header.stamp = rospy.Time.now()
            self.publisher.publish(message)
            self.check_state()
            maximum_actual = max(
                abs(self.latest_feedback.wheel_linear_velocity_left_actual),
                abs(self.latest_feedback.wheel_linear_velocity_right_actual),
            )
            if maximum_actual <= STOPPED_WHEEL_SPEED:
                if stationary_since is None:
                    stationary_since = time.monotonic()
                if time.monotonic() - stationary_since >= STOP_SETTLE_SECONDS:
                    return rospy.Time.now().to_sec()
            else:
                stationary_since = None
            rate.sleep()
        raise RuntimeError("wheel stop was not confirmed within 3 seconds")

    def pose_window(self, end_time, width=0.40):
        return [x for x in self.poses if end_time - width <= x[0] <= end_time]

    def run_segment(self, name, linear, angular, duration):
        self.check_state(require_stationary=True)
        pre_time = rospy.Time.now().to_sec()
        pre_pose = median_pose(self.pose_window(pre_time))
        start = rospy.Time.now().to_sec()
        rospy.logwarn(
            "Calibration segment %s: v=%+.3f m/s w=%+.3f rad/s duration=%.2f s",
            name, linear, angular, duration,
        )
        self.publish_for(linear, angular, duration, "calibration_" + name)
        stop_time = self.stop_and_settle()
        post_pose = median_pose(self.pose_window(stop_time))
        left_corrected = integrate(self.feedback, start, stop_time, 1)
        right_corrected = integrate(self.feedback, start, stop_time, 2)
        imu_yaw = integrate(self.feedback, start, stop_time, 3)
        left_raw = left_corrected / self.active_scale_left
        right_raw = right_corrected / self.active_scale_right
        dx = post_pose[0] - pre_pose[0]
        dy = post_pose[1] - pre_pose[1]
        c = math.cos(pre_pose[2])
        s = math.sin(pre_pose[2])
        forward_chord = c * dx + s * dy
        lateral_chord = -s * dx + c * dy
        camera_yaw = normalize_angle(post_pose[2] - pre_pose[2])
        chord = math.hypot(dx, dy)
        if abs(camera_yaw) > 1.0e-4:
            ratio = abs(camera_yaw) / (2.0 * abs(math.sin(0.5 * camera_yaw)))
            camera_distance = math.copysign(chord * ratio, linear)
        else:
            camera_distance = math.copysign(chord, linear)
        confidence_values = [
            x[1] for x in self.confidences if start <= x[0] <= stop_time
        ]
        result = {
            "name": name,
            "linear_command_mps": linear,
            "angular_command_radps": angular,
            "command_duration_sec": duration,
            "wheel_left_raw_integral_m": left_raw,
            "wheel_right_raw_integral_m": right_raw,
            "camera_distance_m": camera_distance,
            "camera_forward_chord_m": forward_chord,
            "camera_lateral_chord_m": lateral_chord,
            "camera_yaw_rad": camera_yaw,
            "imu_yaw_integral_rad": imu_yaw,
            "confidence_min": min(confidence_values),
            "confidence_mean": statistics.mean(confidence_values),
            "battery_voltage_min": min(
                x[4] for x in self.feedback if start <= x[0] <= stop_time
            ),
            "battery_voltage_mean": statistics.mean(
                x[4] for x in self.feedback if start <= x[0] <= stop_time
            ),
        }
        self.segments.append(result)
        rospy.loginfo(
            "%s measured camera_distance=%+.4f m yaw=%+.4f rad "
            "wheel_integrals=(%+.4f,%+.4f) m",
            name, camera_distance, camera_yaw, left_raw, right_raw,
        )
        self.command_sequence += 1

    def wait_until_ready(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if (
                self.latest_feedback is not None
                and self.poses
                and self.confidences
                and self.alive
            ):
                self.check_state()
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("required calibration topics did not become ready")
        deadline = time.monotonic() + 5.0
        while self.publisher.get_num_connections() < self.args.required_command_subscribers:
            if time.monotonic() >= deadline:
                raise RuntimeError("required command subscribers did not connect")
            time.sleep(0.05)
        self.stop_and_settle()
        self.check_state(require_stationary=True)

    def fit(self):
        def fit_segments(segments):
            subset_rows = []
            subset_observations = []
            for item in segments:
                left = item["wheel_left_raw_integral_m"]
                right = item["wheel_right_raw_integral_m"]
                subset_rows.append([0.5 * left, 0.5 * right, 0.0])
                subset_observations.append(item["camera_distance_m"])
                subset_rows.append([-left, right, -item["camera_yaw_rad"]])
                subset_observations.append(0.0)
            return least_squares(subset_rows, subset_observations)

        def span(values):
            return max(values) - min(values)

        def median_absolute_deviation(values):
            centre = statistics.median(values)
            return statistics.median(abs(value - centre) for value in values)

        cycle_results = []
        for cycle in range(1, self.args.cycles + 1):
            cycle_segments = [
                segment for segment in self.segments
                if segment["name"].endswith(f"_{cycle}")
            ]
            left_scale, right_scale, track = fit_segments(cycle_segments)
            turns = {}
            for label, sign in (("positive", 1.0), ("negative", -1.0)):
                turn = next(
                    item for item in cycle_segments
                    if item["angular_command_radps"] * sign > 0.0
                )
                turns[label] = turn["camera_yaw_rad"] / (
                    turn["angular_command_radps"] *
                    turn["command_duration_sec"]
                )
            cycle_results.append({
                "cycle": cycle,
                "wheel_feedback_scale_left": left_scale,
                "wheel_feedback_scale_right": right_scale,
                "effective_wheel_separation_m": track,
                "positive_turn_gain": turns["positive"],
                "negative_turn_gain": turns["negative"],
            })

        medians = {
            key: statistics.median(item[key] for item in cycle_results)
            for key in (
                "wheel_feedback_scale_left",
                "wheel_feedback_scale_right",
                "effective_wheel_separation_m",
            )
        }
        geometry_inliers = [
            item for item in cycle_results
            if abs(item["wheel_feedback_scale_left"] -
                   medians["wheel_feedback_scale_left"]) <= 0.020
            and abs(item["wheel_feedback_scale_right"] -
                    medians["wheel_feedback_scale_right"]) <= 0.020
            and abs(item["effective_wheel_separation_m"] -
                    medians["effective_wheel_separation_m"]) <= 0.005
        ]
        # Keep producing diagnostic candidates even when fewer than two cycles
        # agree, but the component status below will reject those candidates.
        fit_source = geometry_inliers if geometry_inliers else cycle_results
        scale_left = statistics.median(
            item["wheel_feedback_scale_left"] for item in fit_source
        )
        scale_right = statistics.median(
            item["wheel_feedback_scale_right"] for item in fit_source
        )
        effective_track = statistics.median(
            item["effective_wheel_separation_m"] for item in fit_source
        )
        accepted_cycles = {item["cycle"] for item in fit_source}
        fitted_segments = [
            segment for segment in self.segments
            if int(segment["name"].rsplit("_", 1)[1]) in accepted_cycles
        ]
        translation_residuals = []
        yaw_residuals = []
        for segment in fitted_segments:
            left = segment["wheel_left_raw_integral_m"]
            right = segment["wheel_right_raw_integral_m"]
            predicted_distance = 0.5 * (scale_left * left + scale_right * right)
            predicted_yaw = (
                scale_right * right - scale_left * left
            ) / effective_track
            translation_residuals.append(
                predicted_distance - segment["camera_distance_m"]
            )
            yaw_residuals.append(predicted_yaw - segment["camera_yaw_rad"])

        directional = {}
        for label, sign in (("positive", 1.0), ("negative", -1.0)):
            gain_key = f"{label}_turn_gain"
            all_gains = [item[gain_key] for item in cycle_results]
            gain_median = statistics.median(all_gains)
            inliers = [
                item for item in cycle_results
                if abs(item[gain_key] - gain_median) <= 0.050
            ]
            gain_source = inliers if inliers else cycle_results
            gain = statistics.median(item[gain_key] for item in gain_source)
            directional[label] = {
                "measured_gain": gain,
                "cycle_gains": all_gains,
                "inlier_cycles": [item["cycle"] for item in inliers],
                "outlier_cycles": [
                    item["cycle"] for item in cycle_results if item not in inliers
                ],
                "inlier_span": span([item[gain_key] for item in gain_source]),
            }

        # The measured turn gain was produced with the nominal track used to
        # convert body yaw rate into wheel commands.  If the fitted effective
        # track is applied as well, compensate for that geometry change here;
        # otherwise track ratio and direction gain would be counted twice.
        track_ratio = self.args.nominal_wheel_separation / effective_track
        for value in directional.values():
            gain = value["measured_gain"]
            fixed_nominal_track_scale = (
                1.0 / gain if abs(gain) > 1.0e-9 else 1.0e6
            )
            joint_scale = track_ratio * fixed_nominal_track_scale
            value["fixed_nominal_track_scale"] = fixed_nominal_track_scale
            value["feedforward_scale_unclamped"] = joint_scale
            value["feedforward_scale_candidate"] = min(
                1.25, max(0.75, joint_scale)
            )

        translation_rmse = math.sqrt(
            statistics.mean(value * value for value in translation_residuals)
        )
        yaw_rmse = math.sqrt(
            statistics.mean(value * value for value in yaw_residuals)
        )
        chassis_reasons = []
        geometry_reasons = []
        direction_reasons = {"positive": [], "negative": []}
        if len(geometry_inliers) < 2:
            chassis_reasons.append("fewer than two robust geometry cycles")
            geometry_reasons.append("fewer than two robust geometry cycles")
        if not 0.8 <= scale_left <= 1.25:
            chassis_reasons.append("left feedback scale outside [0.8, 1.25]")
        if not 0.8 <= scale_right <= 1.25:
            chassis_reasons.append("right feedback scale outside [0.8, 1.25]")
        if not 0.08 <= effective_track <= 0.16:
            geometry_reasons.append(
                "effective wheel separation outside [0.08, 0.16] m"
            )
        if translation_rmse > 0.010:
            chassis_reasons.append("translation fit RMSE exceeds 10 mm")
        if yaw_rmse > 0.035:
            geometry_reasons.append("yaw fit RMSE exceeds 0.035 rad")
        for label, value in directional.items():
            if len(value["inlier_cycles"]) < 2:
                direction_reasons[label].append(
                    "fewer than two robust direction cycles"
                )
            if not 0.65 <= value["measured_gain"] <= 1.35:
                direction_reasons[label].append(
                    "turn gain outside [0.65, 1.35]"
                )
            if not 0.75 <= value["feedforward_scale_unclamped"] <= 1.25:
                direction_reasons[label].append(
                    "feedforward candidate requires saturation"
                )
        cycle_left_span = span([
            item["wheel_feedback_scale_left"] for item in fit_source
        ])
        cycle_right_span = span([
            item["wheel_feedback_scale_right"] for item in fit_source
        ])
        cycle_track_span = span([
            item["effective_wheel_separation_m"] for item in fit_source
        ])
        straight_ratios = []
        for segment in fitted_segments:
            if abs(segment["angular_command_radps"]) > 1.0e-9:
                continue
            average_wheel = 0.5 * (
                segment["wheel_left_raw_integral_m"] +
                segment["wheel_right_raw_integral_m"]
            )
            straight_ratios.append(segment["camera_distance_m"] / average_wheel)
        straight_ratio_median = statistics.median(straight_ratios)
        straight_ratio_mad = median_absolute_deviation(straight_ratios)
        straight_ratio_inliers = [
            value for value in straight_ratios
            if abs(value - straight_ratio_median) <= 0.025
        ]
        straight_ratio_span = span(straight_ratio_inliers)
        if cycle_left_span > 0.020:
            chassis_reasons.append("left feedback scale cycle span exceeds 0.020")
        if cycle_right_span > 0.020:
            chassis_reasons.append("right feedback scale cycle span exceeds 0.020")
        if cycle_track_span > 0.005:
            geometry_reasons.append(
                "effective wheel separation cycle span exceeds 5 mm"
            )
        if straight_ratio_mad > 0.010:
            chassis_reasons.append("straight segment scale MAD exceeds 0.010")
        if len(straight_ratio_inliers) < max(4, len(straight_ratios) - 1):
            chassis_reasons.append("more than one straight segment is an outlier")
        for label, value in directional.items():
            if value["inlier_span"] > 0.050:
                direction_reasons[label].append(
                    "inlier turn gain span exceeds 0.050"
                )
        component_status = {
            "wheel_feedback": (
                "REVIEW_REQUIRED" if not chassis_reasons else "REJECTED"
            ),
            "effective_wheel_separation": (
                "REVIEW_REQUIRED" if not geometry_reasons else "REJECTED"
            ),
            "positive_turn_feedforward": (
                "REVIEW_REQUIRED" if not direction_reasons["positive"]
                else "REJECTED"
            ),
            "negative_turn_feedforward": (
                "REVIEW_REQUIRED" if not direction_reasons["negative"]
                else "REJECTED"
            ),
        }
        quality_reasons = (
            [f"wheel_feedback: {reason}" for reason in chassis_reasons] +
            [f"effective_wheel_separation: {reason}"
             for reason in geometry_reasons] +
            [f"positive_turn_feedforward: {reason}"
             for reason in direction_reasons["positive"]] +
            [f"negative_turn_feedforward: {reason}"
             for reason in direction_reasons["negative"]]
        )
        return {
            "status": "REVIEW_REQUIRED" if not quality_reasons else "REJECTED",
            "robot_id": self.args.robot_id,
            "active_feedback_scale_left": self.active_scale_left,
            "active_feedback_scale_right": self.active_scale_right,
            "wheel_feedback_scale_left_candidate": scale_left,
            "wheel_feedback_scale_right_candidate": scale_right,
            "effective_wheel_separation_candidate_m": effective_track,
            "angular_feedforward_scale_positive_candidate": directional["positive"]["feedforward_scale_candidate"],
            "angular_feedforward_scale_negative_candidate": directional["negative"]["feedforward_scale_candidate"],
            "positive_turn_measured_gain": directional["positive"]["measured_gain"],
            "negative_turn_measured_gain": directional["negative"]["measured_gain"],
            "positive_turn_fixed_nominal_track_scale": directional["positive"]["fixed_nominal_track_scale"],
            "negative_turn_fixed_nominal_track_scale": directional["negative"]["fixed_nominal_track_scale"],
            "translation_fit_rmse_m": translation_rmse,
            "yaw_fit_rmse_rad": yaw_rmse,
            "component_status": component_status,
            "component_quality_reasons": {
                "wheel_feedback": chassis_reasons,
                "effective_wheel_separation": geometry_reasons,
                "positive_turn_feedforward": direction_reasons["positive"],
                "negative_turn_feedforward": direction_reasons["negative"],
            },
            "cycle_fit_candidates": cycle_results,
            "geometry_inlier_cycles": [
                item["cycle"] for item in geometry_inliers
            ],
            "geometry_outlier_cycles": [
                item["cycle"] for item in cycle_results
                if item not in geometry_inliers
            ],
            "cycle_left_scale_span": cycle_left_span,
            "cycle_right_scale_span": cycle_right_span,
            "cycle_effective_track_span_m": cycle_track_span,
            "straight_segment_scale_span": straight_ratio_span,
            "straight_segment_scale_median": straight_ratio_median,
            "straight_segment_scale_mad": straight_ratio_mad,
            "straight_segment_inlier_count": len(straight_ratio_inliers),
            "positive_turn_gain_cycle_span": directional["positive"]["inlier_span"],
            "negative_turn_gain_cycle_span": directional["negative"]["inlier_span"],
            "positive_turn_inlier_cycles": directional["positive"]["inlier_cycles"],
            "negative_turn_inlier_cycles": directional["negative"]["inlier_cycles"],
            "positive_turn_outlier_cycles": directional["positive"]["outlier_cycles"],
            "negative_turn_outlier_cycles": directional["negative"]["outlier_cycles"],
            "minimum_calibration_voltage": self.args.minimum_calibration_voltage,
            "quality_reasons": quality_reasons,
            "segments": self.segments,
        }

    def write_results(self, result):
        os.makedirs(self.args.output_dir, exist_ok=True)
        json_path = os.path.join(self.args.output_dir, "calibration_report.json")
        yaml_path = os.path.join(self.args.output_dir, "calibration_candidate.yaml")
        with open(json_path, "w", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        with open(yaml_path, "w", encoding="utf-8") as stream:
            stream.write("# REVIEW ONLY: this file is never applied automatically.\n")
            stream.write(f"status: {result['status']}\n")
            stream.write(f"robot_id: agv{self.args.robot_id}\n")
            stream.write("chassis_candidate:\n")
            stream.write(
                "  wheel_feedback_scale_left: "
                f"{result['wheel_feedback_scale_left_candidate']:.9f}\n"
            )
            stream.write(
                "  wheel_feedback_scale_right: "
                f"{result['wheel_feedback_scale_right_candidate']:.9f}\n"
            )
            stream.write("controller_candidate:\n")
            stream.write(
                "  effective_wheel_separation_m: "
                f"{result['effective_wheel_separation_candidate_m']:.9f}\n"
            )
            stream.write(
                "  angular_feedforward_scale_positive: "
                f"{result['angular_feedforward_scale_positive_candidate']:.9f}\n"
            )
            stream.write(
                "  angular_feedforward_scale_negative: "
                f"{result['angular_feedforward_scale_negative_candidate']:.9f}\n"
            )
            stream.write("quality:\n")
            stream.write("  component_status:\n")
            for name, status in result["component_status"].items():
                stream.write(f"    {name}: {status}\n")
            stream.write(
                f"  translation_fit_rmse_m: {result['translation_fit_rmse_m']:.9f}\n"
            )
            stream.write(f"  yaw_fit_rmse_rad: {result['yaw_fit_rmse_rad']:.9f}\n")
            stream.write("  automatically_applied: false\n")
            stream.write("  reasons:\n")
            for reason in result["quality_reasons"]:
                stream.write(f"    - {reason}\n")
        return json_path, yaml_path

    def run(self):
        self.wait_until_ready()
        rospy.logwarn(
            "Starting bounded %d-cycle camera wheel/turn calibration for %s; "
            "candidate parameters will not be applied automatically",
            self.args.cycles, self.robot,
        )
        for cycle in range(1, self.args.cycles + 1):
            self.run_segment(
                f"forward_{cycle}", self.args.straight_speed, 0.0,
                self.args.straight_duration,
            )
            self.run_segment(
                f"reverse_{cycle}", -self.args.straight_speed, 0.0,
                self.args.straight_duration,
            )
            self.run_segment(
                f"positive_arc_{cycle}", self.args.arc_linear_speed,
                self.args.arc_angular_speed, self.args.arc_duration,
            )
            self.run_segment(
                f"negative_arc_{cycle}", self.args.arc_linear_speed,
                -self.args.arc_angular_speed, self.args.arc_duration,
            )
        result = self.fit()
        json_path, yaml_path = self.write_results(result)
        rospy.loginfo("Calibration result status=%s", result["status"])
        rospy.loginfo("Report: %s", json_path)
        rospy.loginfo("Candidate: %s", yaml_path)
        return 0 if result["status"] == "REVIEW_REQUIRED" else 6

    def emergency_stop(self):
        try:
            self.command_sequence += 1
            message = self.make_command(0.0, 0.0, "calibration_emergency_stop")
            for _ in range(10):
                message.header.stamp = rospy.Time.now()
                self.publisher.publish(message)
                time.sleep(0.03)
        except Exception as error:  # best-effort stop during teardown
            rospy.logerr("Failed to publish calibration stop: %s", error)


def main():
    args = parse_args()
    rospy.init_node("camera_wheel_turn_calibration", anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    publishers, _, _ = master.getSystemState()
    existing = dict(publishers).get(f"/agv{args.robot_id}/chassis_command", [])
    if existing:
        rospy.logerr("Competing chassis command publisher(s): %s", ", ".join(existing))
        return 2
    runner = CalibrationRunner(args)

    def request_stop(_signum, _frame):
        runner.stop_requested = True

    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, request_stop)
    try:
        return runner.run()
    except Exception as error:
        rospy.logerr("Calibration aborted: %s", error)
        return 5
    finally:
        runner.emergency_stop()


if __name__ == "__main__":
    sys.exit(main())
