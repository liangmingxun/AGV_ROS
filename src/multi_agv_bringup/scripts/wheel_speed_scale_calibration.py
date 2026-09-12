#!/usr/bin/env python3
"""Run a bounded multi-speed wheel-scale calibration with camera ground truth.

This tool is intentionally separate from the formal M1/M2a execution path.  It
uses the currently configured feedback scales only to choose conservative
firmware-domain excitation values.  It never writes a production parameter.
"""

import argparse
import csv
import json
import math
import os
import signal
import statistics
import sys
import time


DEFAULT_SPEEDS = (0.06, 0.08, 0.10, 0.12, 0.14, 0.16)
COMMAND_RATE_HZ = 100.0
STATE_TIMEOUT_SECONDS = 0.25
STOPPED_SPEED_MPS = 0.015
STOP_CONFIRM_SECONDS = 0.30
STOP_TIMEOUT_SECONDS = 4.0
ABSOLUTE_ACTUAL_ABORT_MPS = 0.20
SUSTAINED_OVERSPEED_SAMPLES = 5
MINIMUM_STEADY_SAMPLES = 100


def parse_speed_list(value):
    try:
        speeds = tuple(float(item.strip()) for item in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("speeds must be comma-separated numbers") from error
    if not speeds or any(not math.isfinite(item) or item <= 0.0 for item in speeds):
        raise argparse.ArgumentTypeError("all speeds must be finite and positive")
    if list(speeds) != sorted(set(speeds)):
        raise argparse.ArgumentTypeError("speeds must be unique and strictly increasing")
    if speeds[-1] > 0.16 + 1.0e-12:
        raise argparse.ArgumentTypeError("the calibration is bounded to 0.16 m/s")
    return speeds


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Run one robot through a camera-referenced wheel-speed scale ladder. "
            "Candidate scales are review-only and are never applied automatically."
        )
    )
    parser.add_argument("--robot-id", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument(
        "--speeds", type=parse_speed_list,
        default=DEFAULT_SPEEDS,
        help="strictly increasing physical target points, comma-separated",
    )
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--ramp-seconds", type=float, default=1.5)
    parser.add_argument("--steady-seconds", type=float, default=3.0)
    parser.add_argument("--analysis-guard-seconds", type=float, default=0.35)
    parser.add_argument("--minimum-confidence", type=float, default=0.30)
    parser.add_argument("--minimum-battery-voltage", type=float, default=10.5)
    parser.add_argument("--maximum-excursion", type=float, default=0.85)
    parser.add_argument("--return-position-tolerance", type=float, default=0.18)
    parser.add_argument("--required-command-subscribers", type=int, default=2)
    parser.add_argument("--command-seq", type=int, default=100000)
    args = parser.parse_args(argv)
    if not 2 <= args.repetitions <= 5:
        parser.error("repetitions must be between 2 and 5")
    if not 0.5 <= args.ramp_seconds <= 4.0:
        parser.error("ramp-seconds must be between 0.5 and 4.0")
    if not 2.0 <= args.steady_seconds <= 8.0:
        parser.error("steady-seconds must be between 2.0 and 8.0")
    if not 0.1 <= args.analysis_guard_seconds < 0.5 * args.steady_seconds:
        parser.error("analysis guard must leave a non-empty steady fit window")
    if not 0.1 <= args.minimum_confidence <= 0.95:
        parser.error("minimum confidence must be in [0.1, 0.95]")
    if not 9.0 <= args.minimum_battery_voltage <= 12.6:
        parser.error("minimum battery voltage must be in [9.0, 12.6]")
    if not 0.60 <= args.maximum_excursion <= 1.20:
        parser.error("maximum excursion must be in [0.60, 1.20] m")
    if not 0.05 <= args.return_position_tolerance <= 0.30:
        parser.error("return tolerance must be in [0.05, 0.30] m")
    if not 2 <= args.required_command_subscribers <= 4:
        parser.error("at least chassis_controller and rosbag subscribers are required")
    return args


def smoothstep5(value):
    bounded = min(1.0, max(0.0, value))
    return bounded ** 3 * (10.0 + bounded * (-15.0 + 6.0 * bounded))


def _least_squares_time(samples, value_index):
    if len(samples) < 2:
        raise ValueError("at least two samples are required")
    times = [row[0] for row in samples]
    values = [row[value_index] for row in samples]
    centre = statistics.mean(times)
    mean_value = statistics.mean(values)
    denominator = sum((stamp - centre) ** 2 for stamp in times)
    if denominator <= 0.0:
        raise ValueError("sample timestamps have no span")
    slope = sum(
        (stamp - centre) * (value - mean_value)
        for stamp, value in zip(times, values)
    ) / denominator
    return slope, mean_value, centre


def unwrap_angles(values):
    if not values:
        return []
    output = [values[0]]
    for value in values[1:]:
        delta = value - output[-1]
        while delta > math.pi:
            value -= 2.0 * math.pi
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            value += 2.0 * math.pi
            delta += 2.0 * math.pi
        output.append(value)
    return output


def camera_wheel_velocity(pose_samples, wheel_separation):
    """Return body, yaw, left and right velocities from a fixed fit window."""
    if len(pose_samples) < MINIMUM_STEADY_SAMPLES:
        raise ValueError("insufficient camera samples in the steady fit window")
    ordered = sorted(pose_samples)
    unwrapped = unwrap_angles([row[3] for row in ordered])
    fitted = [
        (row[0], row[1], row[2], yaw)
        for row, yaw in zip(ordered, unwrapped)
    ]
    vx, _, centre = _least_squares_time(fitted, 1)
    vy, _, _ = _least_squares_time(fitted, 2)
    omega, mean_yaw, _ = _least_squares_time(fitted, 3)
    # With a linear yaw fit, its value at mean time equals mean_yaw.
    heading = mean_yaw
    body = vx * math.cos(heading) + vy * math.sin(heading)
    left = body - 0.5 * wheel_separation * omega
    right = body + 0.5 * wheel_separation * omega
    return {
        "body_velocity_mps": body,
        "yaw_rate_radps": omega,
        "wheel_left_physical_mps": left,
        "wheel_right_physical_mps": right,
        "fit_centre_stamp": centre,
        "camera_sample_count": len(fitted),
    }


def linear_fit(xs, ys, through_origin=False):
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("linear fit needs paired samples")
    if through_origin:
        denominator = sum(value * value for value in xs)
        if denominator <= 0.0:
            raise ValueError("linear fit input has no magnitude")
        slope = sum(x * y for x, y in zip(xs, ys)) / denominator
        intercept = 0.0
    else:
        mean_x = statistics.mean(xs)
        mean_y = statistics.mean(ys)
        denominator = sum((value - mean_x) ** 2 for value in xs)
        if denominator <= 0.0:
            raise ValueError("linear fit input has no span")
        slope = sum(
            (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)
        ) / denominator
        intercept = mean_y - slope * mean_x
    residuals = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    rmse = math.sqrt(statistics.mean(value * value for value in residuals))
    mean_y = statistics.mean(ys)
    total = sum((value - mean_y) ** 2 for value in ys)
    r_squared = 1.0 - sum(value * value for value in residuals) / total if total > 0 else 1.0
    return {
        "slope": slope,
        "intercept_mps": intercept,
        "rmse_mps": rmse,
        "r_squared": r_squared,
        "maximum_absolute_residual_mps": max(abs(value) for value in residuals),
    }


def evaluate_fit(model, xs, ys):
    residuals = [
        y - (model["slope"] * x + model["intercept_mps"])
        for x, y in zip(xs, ys)
    ]
    return {
        "sample_count": len(residuals),
        "rmse_mps": math.sqrt(statistics.mean(value * value for value in residuals)),
        "maximum_absolute_residual_mps": max(abs(value) for value in residuals),
        "mean_residual_mps": statistics.mean(residuals),
    }


def fit_candidates(segments, repetitions):
    """Fit feedback and command mappings, holding out the last repetition."""
    train = [row for row in segments if row["repetition"] < repetitions]
    validation = [row for row in segments if row["repetition"] == repetitions]
    if not train or not validation:
        raise ValueError("training and held-out validation segments are required")
    wheels = {}
    quality_reasons = []
    for side in ("left", "right"):
        y_key = "camera_wheel_{}_physical_mps".format(side)
        sources = {
            "feedback": "stm32_wheel_{}_measured_nominal_mps".format(side),
            "command_response": "firmware_wheel_{}_target_nominal_mps".format(side),
        }
        side_result = {}
        for name, x_key in sources.items():
            train_x = [row[x_key] for row in train]
            train_y = [row[y_key] for row in train]
            validation_x = [row[x_key] for row in validation]
            validation_y = [row[y_key] for row in validation]
            proportional = linear_fit(train_x, train_y, through_origin=True)
            affine = linear_fit(train_x, train_y, through_origin=False)
            proportional_validation = evaluate_fit(
                proportional, validation_x, validation_y)
            affine_validation = evaluate_fit(affine, validation_x, validation_y)
            recommendation = "proportional"
            if (
                abs(affine["intercept_mps"]) <= 0.003
                and affine_validation["rmse_mps"]
                < 0.8 * proportional_validation["rmse_mps"]
            ):
                recommendation = "affine_review"
            side_result[name] = {
                "proportional": proportional,
                "affine": affine,
                "proportional_validation": proportional_validation,
                "affine_validation": affine_validation,
                "recommended_model": recommendation,
            }
            directional_slopes = {}
            for direction in ("forward", "reverse"):
                selected = [row for row in train if row["direction"] == direction]
                directional_slopes[direction] = linear_fit(
                    [row[x_key] for row in selected],
                    [row[y_key] for row in selected],
                    through_origin=True,
                )["slope"]
            side_result[name]["directional_slopes"] = directional_slopes
            direction_difference = abs(
                directional_slopes["forward"] - directional_slopes["reverse"]
            ) / max(abs(directional_slopes["forward"]), 1.0e-12)
            side_result[name]["directional_slope_difference_fraction"] = (
                direction_difference
            )
            if proportional["r_squared"] < 0.995:
                quality_reasons.append(
                    "{} {} training R^2 is below 0.995".format(side, name)
                )
            if proportional_validation["rmse_mps"] > 0.003:
                quality_reasons.append(
                    "{} {} validation RMSE exceeds 0.003 m/s".format(side, name)
                )
            if proportional_validation["maximum_absolute_residual_mps"] > 0.006:
                quality_reasons.append(
                    "{} {} validation maximum residual exceeds 0.006 m/s".format(
                        side, name
                    )
                )
            if direction_difference > 0.05:
                quality_reasons.append(
                    "{} {} forward/reverse slope difference exceeds 5%".format(
                        side, name
                    )
                )
        feedback_slope = side_result["feedback"]["proportional"]["slope"]
        response_slope = side_result["command_response"]["proportional"]["slope"]
        side_result["wheel_feedback_scale_candidate"] = feedback_slope
        side_result["physical_to_firmware_command_scale_candidate"] = (
            1.0 / response_slope if response_slope > 0.0 else math.nan
        )
        wheels[side] = side_result
    return {
        "status": "REVIEW_REQUIRED" if not quality_reasons else "REJECTED",
        "training_repetitions": list(range(1, repetitions)),
        "validation_repetitions": [repetitions],
        "wheels": wheels,
        "quality_reasons": quality_reasons,
        "automatically_applied": False,
    }


def annotate_segment_residuals(segments, fit):
    for row in segments:
        for side in ("left", "right"):
            y = row["camera_wheel_{}_physical_mps".format(side)]
            for name, x_key in (
                ("feedback", "stm32_wheel_{}_measured_nominal_mps".format(side)),
                ("command_response", "firmware_wheel_{}_target_nominal_mps".format(side)),
            ):
                model = fit["wheels"][side][name]["proportional"]
                row["{}_{}_residual_mps".format(side, name)] = (
                    y - model["slope"] * row[x_key]
                )


def generate_fit_plots(segments, fit, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"left": "#1f77b4", "right": "#d62728"}
    source_fields = (
        ("feedback", "STM32 measured nominal speed (m/s)"),
        ("command_response", "Firmware target nominal speed (m/s)"),
    )
    figure, axes = plt.subplots(2, 2, figsize=(11.0, 8.0), constrained_layout=True)
    for column, (name, x_label) in enumerate(source_fields):
        for row_index, side in enumerate(("left", "right")):
            axis = axes[row_index][column]
            x_key = (
                "stm32_wheel_{}_measured_nominal_mps".format(side)
                if name == "feedback" else
                "firmware_wheel_{}_target_nominal_mps".format(side)
            )
            y_key = "camera_wheel_{}_physical_mps".format(side)
            train = [row for row in segments if row["repetition"] in fit["training_repetitions"]]
            validation = [row for row in segments if row["repetition"] in fit["validation_repetitions"]]
            axis.scatter([row[x_key] for row in train], [row[y_key] for row in train],
                         s=24, color=colors[side], alpha=0.75, label="train")
            axis.scatter([row[x_key] for row in validation],
                         [row[y_key] for row in validation], s=42,
                         facecolors="none", edgecolors="#111111", label="validation")
            limit = max(abs(row[x_key]) for row in segments) * 1.08
            x_line = (-limit, limit)
            model = fit["wheels"][side][name]["proportional"]
            axis.plot(x_line, [model["slope"] * value for value in x_line],
                      color="#444444", linewidth=1.5,
                      label="fit k={:.5f}".format(model["slope"]))
            axis.set_title("{} wheel: {} mapping".format(side.capitalize(), name))
            axis.set_xlabel(x_label)
            axis.set_ylabel("Camera-derived physical speed (m/s)")
            axis.grid(True, alpha=0.25)
            axis.legend(loc="best")
    for suffix, dpi in (("png", 300), ("pdf", None)):
        figure.savefig(os.path.join(output_dir, "scale_fit." + suffix), dpi=dpi)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(10.0, 6.5), sharex=True,
                               constrained_layout=True)
    for axis, side in zip(axes, ("left", "right")):
        for name, marker in (("feedback", "o"), ("command_response", "x")):
            key = "{}_{}_residual_mps".format(side, name)
            axis.scatter(
                [abs(row["physical_target_point_mps"]) for row in segments],
                [row[key] for row in segments], s=28, marker=marker,
                label=name,
            )
        axis.axhline(0.0, color="#222222", linewidth=1.0)
        axis.set_ylabel("{} residual (m/s)".format(side.capitalize()))
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
    axes[-1].set_xlabel("Physical target point magnitude (m/s)")
    for suffix, dpi in (("png", 300), ("pdf", None)):
        figure.savefig(os.path.join(output_dir, "scale_residuals." + suffix), dpi=dpi)
    plt.close(figure)


class CalibrationRunner:
    def __init__(self, args):
        import rosgraph
        import rospy
        from agv_msgs.msg import ChassisCommand, ChassisFeedback
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, Float64, UInt64

        self.rospy = rospy
        self.ChassisCommand = ChassisCommand
        self.args = args
        self.robot = "agv{}".format(args.robot_id)
        self.command_topic = "/{}/chassis_command".format(self.robot)
        self.feedback_topic = "/{}/chassis_feedback".format(self.robot)
        self.pose_topic = "/pose_provider/{}/base_pose_raw".format(self.robot)
        self.confidence_topic = "/camera/world/{}_confidence".format(self.robot)
        self.feedback = []
        self.poses = []
        self.samples = []
        self.segments = []
        self.latest_feedback = None
        self.latest_feedback_received = 0.0
        self.latest_pose_received = 0.0
        self.latest_confidence_received = 0.0
        self.latest_confidence = math.nan
        self.alive = False
        self.alive_received = 0.0
        self.calibration_epoch = None
        self.calibration_epoch_changed = False
        self.phase = "initializing"
        self.point_speed = 0.0
        self.active_origin = None
        self.overspeed_count = 0
        self.last_safety_feedback_seq = None
        self.stop_requested = False
        self.command_sequence = args.command_seq

        rospy.init_node(
            "wheel_speed_scale_calibration", anonymous=True, disable_signals=True
        )
        master = rosgraph.Master(rospy.get_name())
        publishers, _, _ = master.getSystemState()
        existing = dict(publishers).get(self.command_topic, [])
        if existing:
            raise RuntimeError(
                "command topic already has publisher(s): {}".format(", ".join(existing))
            )

        root = "/{}/chassis_controller".format(self.robot)
        self.feedback_scale_left = float(
            rospy.get_param(root + "/wheel_feedback_scale_left")
        )
        self.feedback_scale_right = float(
            rospy.get_param(root + "/wheel_feedback_scale_right")
        )
        self.wheel_separation = float(rospy.get_param(root + "/wheel_separation"))
        formal_limit = float(rospy.get_param(root + "/formal_available_wheel_limit"))
        transport = str(rospy.get_param(root + "/transport_type"))
        if transport != "serial":
            raise RuntimeError("calibration requires serial chassis transport")
        if abs(formal_limit - 0.16) > 1.0e-9:
            raise RuntimeError(
                "chassis formal_available_wheel_limit must be 0.16 m/s"
            )
        for name, value in (
            ("left feedback scale", self.feedback_scale_left),
            ("right feedback scale", self.feedback_scale_right),
        ):
            if not math.isfinite(value) or not 0.8 <= value <= 1.25:
                raise RuntimeError("{} is outside [0.8, 1.25]".format(name))
        if not 0.08 <= self.wheel_separation <= 0.16:
            raise RuntimeError("wheel separation is outside [0.08, 0.16] m")

        self.publisher = rospy.Publisher(
            self.command_topic, ChassisCommand, queue_size=2, latch=False
        )
        rospy.Subscriber(
            self.feedback_topic, ChassisFeedback, self.feedback_callback,
            queue_size=300,
        )
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=300)
        rospy.Subscriber(
            self.confidence_topic, Float64, self.confidence_callback, queue_size=100
        )
        rospy.Subscriber("/vision/aruco/alive", Bool, self.alive_callback, queue_size=10)
        rospy.Subscriber(
            "/vision/aruco/calibration_epoch", UInt64,
            self.calibration_epoch_callback, queue_size=2,
        )

    @staticmethod
    def yaw_from_pose(pose):
        q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def feedback_callback(self, message):
        now = self.rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec() or now
        left_actual = float(message.wheel_linear_velocity_left_actual)
        right_actual = float(message.wheel_linear_velocity_right_actual)
        row = {
            "stamp": stamp,
            "receive_stamp": now,
            "phase": self.phase,
            "physical_target_point_mps": self.point_speed,
            "feedback_seq": int(message.feedback_seq),
            "command_seq_applied": int(message.command_seq_applied),
            "packet_seq": int(message.packet_seq),
            "serial_receive_stamp": message.serial_receive_stamp.to_sec(),
            "wheel_left_raw": float(message.wheel_linear_velocity_left_raw),
            "wheel_right_raw": float(message.wheel_linear_velocity_right_raw),
            "wheel_left_applied": float(message.wheel_linear_velocity_left_applied),
            "wheel_right_applied": float(message.wheel_linear_velocity_right_applied),
            "wheel_left_actual_physical": left_actual,
            "wheel_right_actual_physical": right_actual,
            "stm32_wheel_left_measured_nominal_mmps": (
                1000.0 * left_actual / self.feedback_scale_left
            ),
            "stm32_wheel_right_measured_nominal_mmps": (
                1000.0 * right_actual / self.feedback_scale_right
            ),
            # This equality describes the audited pre-remediation ROS code.
            "firmware_wheel_left_target_nominal_mmps": (
                1000.0 * float(message.wheel_linear_velocity_left_applied)
            ),
            "firmware_wheel_right_target_nominal_mmps": (
                1000.0 * float(message.wheel_linear_velocity_right_applied)
            ),
            "battery_voltage": float(message.battery_voltage),
            "camera_pose_stamp": self.poses[-1][0] if self.poses else math.nan,
            "camera_x": self.poses[-1][1] if self.poses else math.nan,
            "camera_y": self.poses[-1][2] if self.poses else math.nan,
            "camera_yaw": self.poses[-1][3] if self.poses else math.nan,
            "camera_confidence": self.latest_confidence,
        }
        self.feedback.append(row)
        self.samples.append(row)
        self.latest_feedback = message
        self.latest_feedback_received = now

    def pose_callback(self, message):
        now = self.rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec() or now
        self.poses.append((
            stamp,
            float(message.pose.position.x),
            float(message.pose.position.y),
            self.yaw_from_pose(message.pose),
        ))
        self.latest_pose_received = now

    def confidence_callback(self, message):
        self.latest_confidence = float(message.data)
        self.latest_confidence_received = self.rospy.Time.now().to_sec()

    def alive_callback(self, message):
        self.alive = bool(message.data)
        self.alive_received = self.rospy.Time.now().to_sec()

    def calibration_epoch_callback(self, message):
        value = int(message.data)
        if value <= 0:
            self.calibration_epoch_changed = True
        elif self.calibration_epoch is None:
            self.calibration_epoch = value
        elif value != self.calibration_epoch:
            self.calibration_epoch_changed = True

    def request_stop(self, _signum=None, _frame=None):
        self.stop_requested = True

    def current_pose(self):
        return self.poses[-1] if self.poses else None

    def check_state(self, expected_speed=None, require_stationary=False):
        now = self.rospy.Time.now().to_sec()
        if self.stop_requested or self.rospy.is_shutdown():
            raise RuntimeError("calibration interrupted")
        if self.calibration_epoch is None or self.calibration_epoch_changed:
            raise RuntimeError("camera calibration epoch is missing or changed")
        checks = (
            (self.alive and now - self.alive_received <= STATE_TIMEOUT_SECONDS,
             "ArUco UDP link is stale"),
            (self.latest_feedback is not None and
             now - self.latest_feedback_received <= STATE_TIMEOUT_SECONDS,
             "chassis feedback is stale"),
            (self.poses and now - self.latest_pose_received <= STATE_TIMEOUT_SECONDS,
             "camera base pose is stale"),
            (math.isfinite(self.latest_confidence) and
             now - self.latest_confidence_received <= STATE_TIMEOUT_SECONDS,
             "camera confidence is stale"),
        )
        for valid, reason in checks:
            if not valid:
                raise RuntimeError(reason)
        if self.latest_confidence < self.args.minimum_confidence:
            raise RuntimeError(
                "camera confidence {:.3f} is below {:.3f}".format(
                    self.latest_confidence, self.args.minimum_confidence
                )
            )
        if self.latest_feedback.battery_voltage < self.args.minimum_battery_voltage:
            raise RuntimeError(
                "battery voltage {:.3f} V is below {:.3f} V".format(
                    self.latest_feedback.battery_voltage,
                    self.args.minimum_battery_voltage,
                )
            )
        actual = max(
            abs(self.latest_feedback.wheel_linear_velocity_left_actual),
            abs(self.latest_feedback.wheel_linear_velocity_right_actual),
        )
        if actual > ABSOLUTE_ACTUAL_ABORT_MPS:
            raise RuntimeError("actual wheel speed exceeded 0.20 m/s")
        expected = max(0.0, expected_speed or 0.0)
        threshold = min(0.18, expected + 0.030)
        feedback_seq = int(self.latest_feedback.feedback_seq)
        if feedback_seq != self.last_safety_feedback_seq:
            self.last_safety_feedback_seq = feedback_seq
            if actual > threshold:
                self.overspeed_count += 1
            else:
                self.overspeed_count = 0
        if self.overspeed_count >= SUSTAINED_OVERSPEED_SAMPLES:
            raise RuntimeError(
                "actual wheel speed exceeded the bounded calibration envelope"
            )
        if require_stationary and actual > STOPPED_SPEED_MPS:
            raise RuntimeError("wheels are not stationary")
        if self.active_origin is not None:
            pose = self.current_pose()
            excursion = math.hypot(pose[1] - self.active_origin[1],
                                   pose[2] - self.active_origin[2])
            if excursion > self.args.maximum_excursion:
                raise RuntimeError("camera excursion exceeded the configured bound")

    def make_command(self, left, right, method):
        message = self.ChassisCommand()
        message.header.stamp = self.rospy.Time.now()
        message.robot_id = self.args.robot_id
        message.command_seq = self.command_sequence
        message.control_mode = 1
        message.linear_velocity_reference = 0.5 * (left + right)
        message.angular_velocity_reference = (right - left) / self.wheel_separation
        message.wheel_linear_velocity_left_raw = left
        message.wheel_linear_velocity_right_raw = right
        message.experiment_id = "wheel_speed_scale_calibration"
        message.method_id = method
        return message

    def publish_zero(self):
        message = self.make_command(0.0, 0.0, "wheel_scale_stop")
        message.header.stamp = self.rospy.Time.now()
        self.publisher.publish(message)

    def stop_and_confirm(self):
        self.phase = "stop"
        self.point_speed = 0.0
        self.command_sequence += 1
        deadline = time.monotonic() + STOP_TIMEOUT_SECONDS
        stationary_since = None
        rate = self.rospy.Rate(COMMAND_RATE_HZ)
        while time.monotonic() < deadline:
            self.publish_zero()
            self.check_state(expected_speed=0.0)
            actual = max(
                abs(self.latest_feedback.wheel_linear_velocity_left_actual),
                abs(self.latest_feedback.wheel_linear_velocity_right_actual),
            )
            if actual <= STOPPED_SPEED_MPS:
                if stationary_since is None:
                    stationary_since = time.monotonic()
                if time.monotonic() - stationary_since >= STOP_CONFIRM_SECONDS:
                    return
            else:
                stationary_since = None
            rate.sleep()
        raise RuntimeError("wheel stop was not confirmed within 4 seconds")

    def wait_until_ready(self):
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if (
                self.latest_feedback is not None and self.poses
                and math.isfinite(self.latest_confidence) and self.alive
                and self.calibration_epoch is not None
            ):
                self.check_state()
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("required camera/chassis topics did not become ready")
        deadline = time.monotonic() + 8.0
        while self.publisher.get_num_connections() < self.args.required_command_subscribers:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "command topic does not have chassis and rosbag subscribers"
                )
            time.sleep(0.05)
        self.stop_and_confirm()
        self.check_state(require_stationary=True)

    def command_pair(self, target_speed, direction):
        # REVIEW-ONLY excitation: these are not production command scales.
        left = direction * target_speed / self.feedback_scale_left
        right = direction * target_speed / self.feedback_scale_right
        if max(abs(left), abs(right)) > 0.16 + 1.0e-12:
            raise RuntimeError("candidate excitation exceeds the 0.16 command limit")
        return left, right

    def run_leg(self, repetition, target_speed, direction):
        left_target, right_target = self.command_pair(target_speed, direction)
        total = 2.0 * self.args.ramp_seconds + self.args.steady_seconds
        start = self.rospy.Time.now().to_sec()
        steady_start = start + self.args.ramp_seconds + self.args.analysis_guard_seconds
        steady_end = (
            start + self.args.ramp_seconds + self.args.steady_seconds
            - self.args.analysis_guard_seconds
        )
        label = "r{}_{}_{:.2f}".format(
            repetition, "forward" if direction > 0 else "reverse", target_speed
        )
        self.rospy.logwarn(
            "%s: physical target=%+.3f m/s, firmware excitation=[%+.3f,%+.3f] m/s",
            label, direction * target_speed, left_target, right_target,
        )
        self.command_sequence += 1
        deadline = time.monotonic() + total
        rate = self.rospy.Rate(COMMAND_RATE_HZ)
        while time.monotonic() < deadline:
            elapsed = self.rospy.Time.now().to_sec() - start
            if elapsed < self.args.ramp_seconds:
                scale = smoothstep5(elapsed / self.args.ramp_seconds)
                self.phase = "ramp_up"
            elif elapsed < self.args.ramp_seconds + self.args.steady_seconds:
                scale = 1.0
                self.phase = "steady"
            else:
                remaining = total - elapsed
                scale = smoothstep5(remaining / self.args.ramp_seconds)
                self.phase = "ramp_down"
            self.point_speed = direction * target_speed
            self.check_state(expected_speed=target_speed)
            message = self.make_command(
                scale * left_target, scale * right_target,
                "wheel_scale_{}".format(label),
            )
            message.header.stamp = self.rospy.Time.now()
            self.publisher.publish(message)
            rate.sleep()
        self.stop_and_confirm()

        pose_window = [row for row in self.poses if steady_start <= row[0] <= steady_end]
        feedback_window = [
            row for row in self.feedback if steady_start <= row["stamp"] <= steady_end
        ]
        camera = camera_wheel_velocity(pose_window, self.wheel_separation)
        if len(feedback_window) < MINIMUM_STEADY_SAMPLES:
            raise RuntimeError("insufficient chassis feedback in steady fit window")
        confidence = [row["camera_confidence"] for row in feedback_window]
        result = {
            "label": label,
            "repetition": repetition,
            "direction": "forward" if direction > 0 else "reverse",
            "physical_target_point_mps": direction * target_speed,
            "steady_start": steady_start,
            "steady_end": steady_end,
            "feedback_sample_count": len(feedback_window),
            "camera_sample_count": camera["camera_sample_count"],
            "camera_body_velocity_mps": camera["body_velocity_mps"],
            "camera_yaw_rate_radps": camera["yaw_rate_radps"],
            "camera_wheel_left_physical_mps": camera["wheel_left_physical_mps"],
            "camera_wheel_right_physical_mps": camera["wheel_right_physical_mps"],
            "firmware_wheel_left_target_nominal_mps": statistics.mean(
                row["firmware_wheel_left_target_nominal_mmps"] / 1000.0
                for row in feedback_window
            ),
            "firmware_wheel_right_target_nominal_mps": statistics.mean(
                row["firmware_wheel_right_target_nominal_mmps"] / 1000.0
                for row in feedback_window
            ),
            "stm32_wheel_left_measured_nominal_mps": statistics.mean(
                row["stm32_wheel_left_measured_nominal_mmps"] / 1000.0
                for row in feedback_window
            ),
            "stm32_wheel_right_measured_nominal_mps": statistics.mean(
                row["stm32_wheel_right_measured_nominal_mmps"] / 1000.0
                for row in feedback_window
            ),
            "ros_wheel_left_actual_physical_mps": statistics.mean(
                row["wheel_left_actual_physical"] for row in feedback_window
            ),
            "ros_wheel_right_actual_physical_mps": statistics.mean(
                row["wheel_right_actual_physical"] for row in feedback_window
            ),
            "minimum_camera_confidence": min(confidence),
            "minimum_battery_voltage": min(
                row["battery_voltage"] for row in feedback_window
            ),
            "active_feedback_scale_left": self.feedback_scale_left,
            "active_feedback_scale_right": self.feedback_scale_right,
        }
        self.segments.append(result)
        self.rospy.loginfo(
            "%s camera wheels=[%+.4f,%+.4f] m/s, STM32 measured=[%+.4f,%+.4f] m/s",
            label,
            result["camera_wheel_left_physical_mps"],
            result["camera_wheel_right_physical_mps"],
            result["stm32_wheel_left_measured_nominal_mps"],
            result["stm32_wheel_right_measured_nominal_mps"],
        )

    def verify_return(self, origin):
        pose = self.current_pose()
        error = math.hypot(pose[1] - origin[1], pose[2] - origin[2])
        if error > self.args.return_position_tolerance:
            raise RuntimeError(
                "forward/reverse pair return error {:.3f} m exceeds {:.3f} m".format(
                    error, self.args.return_position_tolerance
                )
            )

    def write_raw_samples(self):
        os.makedirs(self.args.output_dir, exist_ok=True)
        path = os.path.join(self.args.output_dir, "calibration_raw.csv")
        if not self.samples:
            return path
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.samples[0]))
            writer.writeheader()
            writer.writerows(self.samples)
        return path

    def write_results(self, fit):
        metrics_path = os.path.join(self.args.output_dir, "speed_point_metrics.csv")
        with open(metrics_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.segments[0]))
            writer.writeheader()
            writer.writerows(self.segments)

        report = {
            "schema_version": 1,
            "status": fit["status"],
            "robot_id": self.robot,
            "operator": self.args.operator,
            "speed_points_mps": list(self.args.speeds),
            "repetitions": self.args.repetitions,
            "directions": ["forward", "reverse"],
            "active_feedback_scale": {
                "left": self.feedback_scale_left,
                "right": self.feedback_scale_right,
            },
            "excitation_policy": (
                "firmware nominal target initialized as physical target divided "
                "by active feedback scale; calibration-only, not production mapping"
            ),
            "fit": fit,
            "segments": self.segments,
        }
        report_path = os.path.join(self.args.output_dir, "scale_fit_report.json")
        with open(report_path, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True, ensure_ascii=False)
            stream.write("\n")

        candidate_path = os.path.join(self.args.output_dir, "scale_candidate.yaml")
        with open(candidate_path, "w", encoding="utf-8") as stream:
            stream.write("# REVIEW ONLY. Never applied automatically.\n")
            stream.write("status: {}\n".format(fit["status"]))
            stream.write("robot_id: {}\n".format(self.robot))
            stream.write("candidate:\n")
            for side in ("left", "right"):
                result = fit["wheels"][side]
                stream.write(
                    "  wheel_feedback_scale_{}: {:.9f}\n".format(
                        side, result["wheel_feedback_scale_candidate"]
                    )
                )
                stream.write(
                    "  physical_to_firmware_command_scale_{}: {:.9f}\n".format(
                        side,
                        result["physical_to_firmware_command_scale_candidate"],
                    )
                )
            stream.write("automatically_applied: false\n")
            if fit["quality_reasons"]:
                stream.write("quality_reasons:\n")
                for reason in fit["quality_reasons"]:
                    stream.write("  - {}\n".format(reason))
            else:
                stream.write("quality_reasons: []\n")
        return metrics_path, report_path, candidate_path

    def emergency_stop(self):
        try:
            self.phase = "emergency_stop"
            self.point_speed = 0.0
            self.command_sequence += 1
            for _ in range(20):
                self.publish_zero()
                time.sleep(0.02)
        except Exception:
            pass

    def run(self):
        os.makedirs(self.args.output_dir, exist_ok=True)
        try:
            self.wait_until_ready()
            self.rospy.logwarn(
                "Starting %s scale ladder: speeds=%s, repetitions=%d; no parameter will be applied",
                self.robot, ",".join("{:.2f}".format(x) for x in self.args.speeds),
                self.args.repetitions,
            )
            for repetition in range(1, self.args.repetitions + 1):
                for speed in self.args.speeds:
                    origin = self.current_pose()
                    self.active_origin = origin
                    self.run_leg(repetition, speed, 1.0)
                    self.run_leg(repetition, speed, -1.0)
                    self.verify_return(origin)
                    self.active_origin = None
            fit = fit_candidates(self.segments, self.args.repetitions)
            annotate_segment_residuals(self.segments, fit)
            raw_path = self.write_raw_samples()
            metrics_path, report_path, candidate_path = self.write_results(fit)
            try:
                generate_fit_plots(self.segments, fit, self.args.output_dir)
            except Exception as error:
                with open(
                    os.path.join(self.args.output_dir, "plot_failure.txt"),
                    "w", encoding="utf-8",
                ) as stream:
                    stream.write(str(error) + "\n")
                self.rospy.logwarn("Fit plots could not be generated: %s", error)
            self.rospy.loginfo("Calibration acquisition complete; status=%s", fit["status"])
            self.rospy.loginfo("Raw samples: %s", raw_path)
            self.rospy.loginfo("Point metrics: %s", metrics_path)
            self.rospy.loginfo("Fit report: %s", report_path)
            self.rospy.loginfo("Review-only candidate: %s", candidate_path)
            return 0
        except Exception as error:
            self.rospy.logerr("Wheel speed scale calibration failed: %s", error)
            failure = {
                "status": "ACQUISITION_FAILED",
                "reason": str(error),
                "robot_id": self.robot,
                "completed_segments": self.segments,
                "automatically_applied": False,
            }
            os.makedirs(self.args.output_dir, exist_ok=True)
            with open(
                os.path.join(self.args.output_dir, "failure_report.json"),
                "w", encoding="utf-8",
            ) as stream:
                json.dump(failure, stream, indent=2, ensure_ascii=False)
                stream.write("\n")
            self.write_raw_samples()
            return 5
        finally:
            self.emergency_stop()


def main(argv=None):
    args = parse_args(argv)
    runner = CalibrationRunner(args)
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, runner.request_stop)
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
