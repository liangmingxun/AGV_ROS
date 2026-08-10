"""Lossless field extraction plus causal alignment for frozen ROS bags."""

import bisect
import json
import math
from collections import defaultdict
from pathlib import Path

import rosbag

from .io_utils import atomic_dump_yaml, stamp_to_sec, write_csv


RAW_SCHEMAS = {
    "chassis_command.csv": [
        "topic", "bag_stamp", "header_stamp", "robot_id", "command_seq",
        "control_mode", "linear_velocity_reference",
        "angular_velocity_reference", "wheel_left_raw", "wheel_right_raw",
        "experiment_id", "method_id",
    ],
    "chassis_feedback.csv": [
        "topic", "bag_stamp", "header_stamp", "serial_receive_stamp",
        "robot_id", "feedback_seq", "command_seq_applied", "packet_seq",
        "wheel_left_raw", "wheel_right_raw", "wheel_left_applied",
        "wheel_right_applied", "wheel_left_actual", "wheel_right_actual",
        "linear_velocity_actual", "angular_velocity_actual",
        "battery_voltage", "control_loop_overrun",
        "speed_limit_active_left", "speed_limit_active_right",
        "accel_limit_active_left", "accel_limit_active_right",
        "decel_limit_active_left", "decel_limit_active_right",
    ],
    "capability_report.csv": [
        "topic", "bag_stamp", "header_stamp", "robot_id", "capability_seq",
        "max_wheel_velocity_left", "max_wheel_velocity_right",
        "max_wheel_acceleration_left", "max_wheel_acceleration_right",
        "max_wheel_deceleration_left", "max_wheel_deceleration_right",
        "derating_ratio", "derating_mode", "derating_active",
        "battery_voltage",
    ],
    "cooperative_state.csv": [
        "topic", "bag_stamp", "header_stamp",
        "robot_localization_source_1", "robot_localization_source_2",
        "robot_localization_source_3",
        "robot_pose_valid_1", "robot_pose_valid_2", "robot_pose_valid_3",
        "robot_pose_stamp_1", "robot_pose_stamp_2", "robot_pose_stamp_3",
        "robot_pose_x_1", "robot_pose_x_2", "robot_pose_x_3",
        "robot_pose_y_1", "robot_pose_y_2", "robot_pose_y_3",
        "robot_pose_yaw_1", "robot_pose_yaw_2", "robot_pose_yaw_3",
        "support_pose_valid_1", "support_pose_valid_2",
        "support_pose_valid_3",
        "support_pose_x_1", "support_pose_x_2", "support_pose_x_3",
        "support_pose_y_1", "support_pose_y_2", "support_pose_y_3",
        "support_pose_yaw_1", "support_pose_yaw_2", "support_pose_yaw_3",
        "path_state_valid_1",
        "path_state_valid_2", "path_state_valid_3",
        "s_actual_1", "s_actual_2", "s_actual_3",
        "s_dot_actual_1", "s_dot_actual_2", "s_dot_actual_3",
        "load_localization_source", "load_pose_valid", "load_pose_stamp",
        "load_pose_x", "load_pose_y", "load_pose_yaw",
        "load_path_state_valid", "load_s_actual", "load_s_dot_actual",
    ],
    "path_reference.csv": [
        "topic", "bag_stamp", "header_stamp", "path_id", "path_version",
        "load_s_reference", "load_velocity_reference",
        "load_acceleration_reference", "load_x_reference",
        "load_y_reference", "load_yaw_reference",
        "support_x_reference_1", "support_x_reference_2",
        "support_x_reference_3", "support_y_reference_1",
        "support_y_reference_2", "support_y_reference_3",
        "support_yaw_reference_1", "support_yaw_reference_2",
        "support_yaw_reference_3",
    ],
    "controller_state.csv": [
        "topic", "bag_stamp", "header_stamp", "experiment_id", "method_id",
        "path_progress_actual_1", "path_progress_actual_2",
        "path_progress_actual_3", "path_velocity_actual_1",
        "path_velocity_actual_2", "path_velocity_actual_3",
        "common_velocity_lower_bound", "common_velocity_upper_bound",
        "common_velocity_reference", "channel_input_raw_1",
        "channel_input_raw_2", "channel_input_raw_3",
        "channel_input_limited_1", "channel_input_limited_2",
        "channel_input_limited_3",
    ],
    "experiment_state.csv": [
        "topic", "bag_stamp", "header_stamp", "experiment_id", "method_id",
        "block_id", "run_id", "phase", "run_active", "evaluation_active",
        "manual_abort", "abort_reason",
    ],
    "derating_command.csv": [
        "topic", "bag_stamp", "header_stamp", "robot_id", "command_seq",
        "mode", "active", "target_speed_ratio_left",
        "target_speed_ratio_right", "target_accel_ratio_left",
        "target_accel_ratio_right", "target_decel_ratio_left",
        "target_decel_ratio_right", "ramp_down_time", "ramp_up_time",
        "experiment_id",
    ],
    "imu.csv": [
        "topic", "bag_stamp", "header_stamp", "frame_id",
        "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
        "linear_acceleration_x", "linear_acceleration_y",
        "linear_acceleration_z", "orientation_x", "orientation_y",
        "orientation_z", "orientation_w",
    ],
    "odometry.csv": [
        "topic", "bag_stamp", "header_stamp", "frame_id", "child_frame_id",
        "position_x", "position_y", "position_z", "orientation_x",
        "orientation_y", "orientation_z", "orientation_w", "linear_x",
        "linear_y", "angular_z",
    ],
    "formal_algorithm_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "m2b_algorithm_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "camera_pose.csv": [
        "topic", "bag_stamp", "header_stamp", "header_seq",
        "calibration_epoch_token", "frame_id", "position_x", "position_y",
        "position_z", "orientation_x", "orientation_y", "orientation_z",
        "orientation_w",
    ],
    "camera_confidence.csv": [
        "topic", "bag_stamp", "header_stamp", "confidence",
    ],
    "camera_calibration_epoch.csv": [
        "topic", "bag_stamp", "header_stamp", "epoch_unix_ns",
    ],
}


def _header_row(topic, bag_stamp, message):
    header = getattr(message, "header", None)
    return {
        "topic": topic,
        "bag_stamp": stamp_to_sec(bag_stamp),
        "header_stamp": stamp_to_sec(
            getattr(header, "stamp", bag_stamp)),
    }


def _array_fields(row, name, values):
    for index, value in enumerate(values, start=1):
        row["{}_{}".format(name, index)] = value


def _extract(topic, bag_stamp, message):
    row = _header_row(topic, bag_stamp, message)
    message_type = getattr(message, "_type", "")
    if message_type == "agv_msgs/ChassisCommand":
        row.update({
            "robot_id": message.robot_id,
            "command_seq": message.command_seq,
            "control_mode": message.control_mode,
            "linear_velocity_reference": message.linear_velocity_reference,
            "angular_velocity_reference": message.angular_velocity_reference,
            "wheel_left_raw": message.wheel_linear_velocity_left_raw,
            "wheel_right_raw": message.wheel_linear_velocity_right_raw,
            "experiment_id": message.experiment_id,
            "method_id": message.method_id,
        })
        return "chassis_command.csv", row
    if message_type == "agv_msgs/ChassisFeedback":
        row.update({
            "serial_receive_stamp": stamp_to_sec(
                message.serial_receive_stamp),
            "robot_id": message.robot_id,
            "feedback_seq": message.feedback_seq,
            "command_seq_applied": message.command_seq_applied,
            "packet_seq": message.packet_seq,
            "wheel_left_raw": message.wheel_linear_velocity_left_raw,
            "wheel_right_raw": message.wheel_linear_velocity_right_raw,
            "wheel_left_applied": message.wheel_linear_velocity_left_applied,
            "wheel_right_applied": message.wheel_linear_velocity_right_applied,
            "wheel_left_actual": message.wheel_linear_velocity_left_actual,
            "wheel_right_actual": message.wheel_linear_velocity_right_actual,
            "linear_velocity_actual": message.linear_velocity_actual,
            "angular_velocity_actual": message.angular_velocity_actual,
            "battery_voltage": message.battery_voltage,
            "control_loop_overrun": message.control_loop_overrun,
            "speed_limit_active_left": message.speed_limit_active_left,
            "speed_limit_active_right": message.speed_limit_active_right,
            "accel_limit_active_left": message.accel_limit_active_left,
            "accel_limit_active_right": message.accel_limit_active_right,
            "decel_limit_active_left": message.decel_limit_active_left,
            "decel_limit_active_right": message.decel_limit_active_right,
        })
        return "chassis_feedback.csv", row
    if message_type == "agv_msgs/CapabilityReport":
        row.update({
            "robot_id": message.robot_id,
            "capability_seq": message.capability_seq,
            "max_wheel_velocity_left":
                message.max_wheel_linear_velocity_left,
            "max_wheel_velocity_right":
                message.max_wheel_linear_velocity_right,
            "max_wheel_acceleration_left":
                message.max_wheel_linear_acceleration_left,
            "max_wheel_acceleration_right":
                message.max_wheel_linear_acceleration_right,
            "max_wheel_deceleration_left":
                message.max_wheel_linear_deceleration_left,
            "max_wheel_deceleration_right":
                message.max_wheel_linear_deceleration_right,
            "derating_ratio": message.derating_ratio,
            "derating_mode": message.derating_mode,
            "derating_active": message.derating_active,
            "battery_voltage": message.battery_voltage,
        })
        return "capability_report.csv", row
    if message_type == "agv_msgs/CooperativeState":
        _array_fields(row, "robot_localization_source",
                      message.robot_localization_source)
        _array_fields(row, "robot_pose_valid", message.robot_pose_valid)
        _array_fields(
            row, "robot_pose_stamp",
            [stamp_to_sec(value) for value in message.robot_pose_stamp])
        _array_fields(row, "robot_pose_x",
                      [pose.x for pose in message.robot_pose])
        _array_fields(row, "robot_pose_y",
                      [pose.y for pose in message.robot_pose])
        _array_fields(row, "robot_pose_yaw",
                      [pose.theta for pose in message.robot_pose])
        _array_fields(row, "support_pose_valid", message.support_pose_valid)
        _array_fields(row, "support_pose_x",
                      [pose.x for pose in message.support_pose])
        _array_fields(row, "support_pose_y",
                      [pose.y for pose in message.support_pose])
        _array_fields(row, "support_pose_yaw",
                      [pose.theta for pose in message.support_pose])
        _array_fields(row, "path_state_valid", message.path_state_valid)
        _array_fields(row, "s_actual", message.s_actual)
        _array_fields(row, "s_dot_actual", message.s_dot_actual)
        row.update({
            "load_localization_source": message.load_localization_source,
            "load_pose_valid": message.load_pose_valid,
            "load_pose_stamp": stamp_to_sec(message.load_pose_stamp),
            "load_pose_x": message.load_pose.x,
            "load_pose_y": message.load_pose.y,
            "load_pose_yaw": message.load_pose.theta,
            "load_path_state_valid": message.load_path_state_valid,
            "load_s_actual": message.load_s_actual,
            "load_s_dot_actual": message.load_s_dot_actual,
        })
        return "cooperative_state.csv", row
    if message_type == "agv_msgs/PathReference":
        row.update({
            "path_id": message.path_id,
            "path_version": message.path_version,
            "load_s_reference": message.load_path_progress_reference,
            "load_velocity_reference": message.load_path_velocity_reference,
            "load_acceleration_reference":
                message.load_path_acceleration_reference,
            "load_x_reference": message.load_pose_reference.x,
            "load_y_reference": message.load_pose_reference.y,
            "load_yaw_reference": message.load_pose_reference.theta,
        })
        _array_fields(
            row, "support_x_reference",
            [pose.x for pose in message.support_pose_reference])
        _array_fields(
            row, "support_y_reference",
            [pose.y for pose in message.support_pose_reference])
        _array_fields(
            row, "support_yaw_reference",
            [pose.theta for pose in message.support_pose_reference])
        return "path_reference.csv", row
    if message_type == "agv_msgs/ControllerState":
        row.update({
            "experiment_id": message.experiment_id,
            "method_id": message.method_id,
            "common_velocity_lower_bound":
                message.common_velocity_lower_bound,
            "common_velocity_upper_bound":
                message.common_velocity_upper_bound,
            "common_velocity_reference":
                message.common_load_velocity_reference,
        })
        _array_fields(
            row, "path_progress_actual", message.path_progress_actual)
        _array_fields(
            row, "path_velocity_actual", message.path_velocity_actual)
        _array_fields(row, "channel_input_raw", message.channel_input_raw)
        _array_fields(
            row, "channel_input_limited", message.channel_input_limited)
        return "controller_state.csv", row
    if message_type == "agv_msgs/ExperimentState":
        row.update({
            "experiment_id": message.experiment_id,
            "method_id": message.method_id,
            "block_id": message.block_id,
            "run_id": message.run_id,
            "phase": message.phase,
            "run_active": message.run_active,
            "evaluation_active": message.evaluation_active,
            "manual_abort": message.manual_abort,
            "abort_reason": message.abort_reason,
        })
        return "experiment_state.csv", row
    if message_type == "agv_msgs/DeratingCommand":
        row.update({
            "robot_id": message.robot_id,
            "command_seq": message.command_seq,
            "mode": message.mode,
            "active": message.active,
            "target_speed_ratio_left": message.target_speed_ratio_left,
            "target_speed_ratio_right": message.target_speed_ratio_right,
            "target_accel_ratio_left": message.target_accel_ratio_left,
            "target_accel_ratio_right": message.target_accel_ratio_right,
            "target_decel_ratio_left": message.target_decel_ratio_left,
            "target_decel_ratio_right": message.target_decel_ratio_right,
            "ramp_down_time": message.ramp_down_time,
            "ramp_up_time": message.ramp_up_time,
            "experiment_id": message.experiment_id,
        })
        return "derating_command.csv", row
    if message_type == "sensor_msgs/Imu":
        row.update({
            "frame_id": message.header.frame_id,
            "angular_velocity_x": message.angular_velocity.x,
            "angular_velocity_y": message.angular_velocity.y,
            "angular_velocity_z": message.angular_velocity.z,
            "linear_acceleration_x": message.linear_acceleration.x,
            "linear_acceleration_y": message.linear_acceleration.y,
            "linear_acceleration_z": message.linear_acceleration.z,
            "orientation_x": message.orientation.x,
            "orientation_y": message.orientation.y,
            "orientation_z": message.orientation.z,
            "orientation_w": message.orientation.w,
        })
        return "imu.csv", row
    if message_type == "nav_msgs/Odometry":
        row.update({
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "position_x": message.pose.pose.position.x,
            "position_y": message.pose.pose.position.y,
            "position_z": message.pose.pose.position.z,
            "orientation_x": message.pose.pose.orientation.x,
            "orientation_y": message.pose.pose.orientation.y,
            "orientation_z": message.pose.pose.orientation.z,
            "orientation_w": message.pose.pose.orientation.w,
            "linear_x": message.twist.twist.linear.x,
            "linear_y": message.twist.twist.linear.y,
            "angular_z": message.twist.twist.angular.z,
        })
        return "odometry.csv", row
    if message_type == "geometry_msgs/PoseStamped":
        frame_id = message.header.frame_id
        epoch_token = ""
        if frame_id.startswith("world@") and len(frame_id) == 14:
            try:
                parsed_token = int(frame_id[6:], 16)
                if parsed_token > 0:
                    epoch_token = parsed_token
            except ValueError:
                pass
        row.update({
            "header_seq": message.header.seq,
            "calibration_epoch_token": epoch_token,
            "frame_id": frame_id,
            "position_x": message.pose.position.x,
            "position_y": message.pose.position.y,
            "position_z": message.pose.position.z,
            "orientation_x": message.pose.orientation.x,
            "orientation_y": message.pose.orientation.y,
            "orientation_z": message.pose.orientation.z,
            "orientation_w": message.pose.orientation.w,
        })
        return "camera_pose.csv", row
    if message_type == "std_msgs/Float64":
        row.update({
            "header_stamp": stamp_to_sec(bag_stamp),
            "confidence": message.data,
        })
        return "camera_confidence.csv", row
    if (message_type == "std_msgs/UInt64" and
            topic == "/vision/aruco/calibration_epoch"):
        row.update({
            "header_stamp": stamp_to_sec(bag_stamp),
            "epoch_unix_ns": message.data,
        })
        return "camera_calibration_epoch.csv", row
    if message_type == "std_msgs/Float64MultiArray":
        label = ""
        if message.layout.dim:
            label = message.layout.dim[0].label
        row.update({
            "header_stamp": stamp_to_sec(bag_stamp),
            "layout_label": label,
            "data_json": json.dumps(list(message.data),
                                    separators=(",", ":")),
        })
        if topic == "/multi_agv/m2b_algorithm_state":
            return "m2b_algorithm_state.csv", row
        return "formal_algorithm_state.csv", row
    return None, None


def _series(rows):
    ordered = sorted(rows, key=lambda row: float(row["header_stamp"]))
    return ([float(row["header_stamp"]) for row in ordered], ordered)


def _latest(series, stamp, maximum_age):
    stamps, rows = series
    if not stamps:
        return None
    index = bisect.bisect_right(stamps, stamp) - 1
    if index < 0:
        return None
    row = rows[index]
    return row if stamp - float(row["header_stamp"]) <= maximum_age else None


def _valid_cooperative(row):
    if not row:
        return False
    fields = ["load_pose_valid", "load_path_state_valid"]
    for index in range(1, 4):
        fields.extend([
            "robot_pose_valid_{}".format(index),
            "support_pose_valid_{}".format(index),
            "path_state_valid_{}".format(index),
        ])
    return all(str(row.get(field, "")).lower() in ("true", "1")
               for field in fields)


def _aligned_rows(raw, maximum_age):
    anchors = raw["cooperative_state.csv"]
    feedback = defaultdict(list)
    capability = defaultdict(list)
    for row in raw["chassis_feedback.csv"]:
        feedback[int(row["robot_id"])].append(row)
    for row in raw["capability_report.csv"]:
        capability[int(row["robot_id"])].append(row)
    feedback = {
        robot: _series(feedback[robot]) for robot in range(1, 4)}
    capability = {
        robot: _series(capability[robot]) for robot in range(1, 4)}
    paths = _series(raw["path_reference.csv"])
    controllers = _series(raw["controller_state.csv"])
    formal_debug = _series(raw["formal_algorithm_state.csv"])
    m2b_debug = _series(raw["m2b_algorithm_state.csv"])
    experiment_states = _series(raw["experiment_state.csv"])
    has_experiment_state_stream = bool(experiment_states[0])

    output = []
    for state in anchors:
        stamp = float(state["header_stamp"])
        path = _latest(paths, stamp, maximum_age)
        controller = _latest(controllers, stamp, maximum_age)
        debug = _latest(formal_debug, stamp, maximum_age)
        m2b = _latest(m2b_debug, stamp, maximum_age)
        experiment_state = _latest(
            experiment_states, stamp, maximum_age)
        row = {
            "stamp": stamp,
            "localization_valid": _valid_cooperative(state),
            "load_s_actual": state.get("load_s_actual", math.nan),
            "load_s_dot_actual": state.get("load_s_dot_actual", math.nan),
            "load_pose_x": state.get("load_pose_x", math.nan),
            "load_pose_y": state.get("load_pose_y", math.nan),
            "load_pose_yaw": state.get("load_pose_yaw", math.nan),
            "load_x_reference": (
                path.get("load_x_reference", math.nan)
                if path else math.nan),
            "load_y_reference": (
                path.get("load_y_reference", math.nan)
                if path else math.nan),
            "load_yaw_reference": (
                path.get("load_yaw_reference", math.nan)
                if path else math.nan),
            "load_s_reference": (
                path.get("load_s_reference", math.nan)
                if path else math.nan),
            "load_velocity_reference": (
                path.get("load_velocity_reference", math.nan)
                if path else math.nan),
            "common_velocity_reference": (
                controller.get("common_velocity_reference", math.nan)
                if controller else math.nan),
            "mapped_common_velocity_lower": (
                controller.get("common_velocity_lower_bound", math.nan)
                if controller else math.nan),
            "mapped_common_velocity_upper": (
                controller.get("common_velocity_upper_bound", math.nan)
                if controller else math.nan),
            "evaluation_active": (
                experiment_state.get("evaluation_active", False)
                if experiment_state else not has_experiment_state_stream),
            "experiment_state_available": experiment_state is not None,
            "manual_abort": (
                experiment_state.get("manual_abort", False)
                if experiment_state else False),
        }
        debug_values = []
        if (debug and debug.get("layout_label") == (
                "formal_algorithm_state_v1:header9+3x27")):
            try:
                debug_values = json.loads(debug["data_json"])
            except (TypeError, ValueError):
                debug_values = []
        m2b_values = []
        if (m2b and m2b.get("layout_label") ==
                "m2b_algorithm_state_v1:header6+3x20"):
            try:
                m2b_values = json.loads(m2b["data_json"])
            except (TypeError, ValueError):
                m2b_values = []
        for robot in range(1, 4):
            for pose_name in ("robot_pose", "support_pose"):
                for axis in ("x", "y", "yaw"):
                    field = "{}_{}_{}".format(pose_name, axis, robot)
                    row["agv{}_{}_{}".format(
                        robot, pose_name, axis)] = state.get(field, math.nan)
            row["agv{}_s_actual".format(robot)] = state.get(
                "s_actual_{}".format(robot), math.nan)
            row["agv{}_s_dot_actual".format(robot)] = state.get(
                "s_dot_actual_{}".format(robot), math.nan)
            for axis in ("x", "y", "yaw"):
                row["agv{}_support_reference_{}".format(robot, axis)] = (
                    path.get(
                        "support_{}_reference_{}".format(axis, robot),
                        math.nan) if path else math.nan)
            current_feedback = _latest(
                feedback[robot], stamp, maximum_age)
            current_capability = _latest(
                capability[robot], stamp, maximum_age)
            for side in ("left", "right"):
                prefix = "agv{}_wheel_{}".format(robot, side)
                source_side = side
                for stage in ("raw", "applied", "actual"):
                    row[prefix + "_" + stage] = (
                        current_feedback.get(
                            "wheel_{}_{}".format(source_side, stage), math.nan)
                        if current_feedback else math.nan)
                row[prefix + "_reported_limit"] = (
                    current_capability.get(
                        "max_wheel_velocity_{}".format(side), math.nan)
                    if current_capability else math.nan)
                row[prefix + "_speed_limit_active"] = (
                    current_feedback.get(
                        "speed_limit_active_{}".format(side), False)
                    if current_feedback else False)
                row[prefix + "_accel_limit_active"] = (
                    current_feedback.get(
                        "accel_limit_active_{}".format(side), False)
                    if current_feedback else False)
                row[prefix + "_decel_limit_active"] = (
                    current_feedback.get(
                        "decel_limit_active_{}".format(side), False)
                    if current_feedback else False)
            row["agv{}_reported_velocity_limit".format(robot)] = (
                min(float(current_capability["max_wheel_velocity_left"]),
                    float(current_capability["max_wheel_velocity_right"]))
                if current_capability else math.nan)
            debug_offset = 9 + (robot - 1) * 27
            row["agv{}_mapped_velocity_lower".format(robot)] = (
                debug_values[debug_offset]
                if len(debug_values) > debug_offset + 1 else math.nan)
            row["agv{}_mapped_velocity_upper".format(robot)] = (
                debug_values[debug_offset + 1]
                if len(debug_values) > debug_offset + 1 else math.nan)
            if len(debug_values) >= debug_offset + 27:
                names = (
                    "boundary_lower", "boundary_upper", "robust_margin",
                    "risk_factor", "risk_signal", "z", "upsilon", "phi",
                    "zeta_z", "zeta_v", "zeta_phi",
                    "reference_acceleration", "debug_s_actual",
                    "debug_s_dot_actual", "position_error",
                    "velocity_error", "psi", "gamma",
                    "gamma_inverse_upsilon", "composite_error",
                    "channel_input_raw", "channel_input_limited",
                    "theta_hat_1", "theta_hat_2", "disturbance_estimate",
                    "channel_limit_active", "sustained_wheel_saturation")
                for offset, name in enumerate(names):
                    row["agv{}_{}".format(robot, name)] = (
                        debug_values[debug_offset + offset])
            m2b_offset = 6 + (robot - 1) * 20
            if len(m2b_values) >= m2b_offset + 20:
                names = (
                    "z", "upsilon", "phi", "reference_acceleration",
                    "zeta_z", "zeta_v", "zeta_phi", "debug_s_actual",
                    "debug_s_dot_actual", "position_error",
                    "velocity_error", "psi", "gamma",
                    "gamma_inverse_upsilon", "composite_error",
                    "channel_input_raw", "channel_input_limited",
                    "reported_capability", "delta_inverse", "mapping_zeta")
                for offset, name in enumerate(names):
                    row["m2b_agv{}_{}".format(robot, name)] = (
                        m2b_values[m2b_offset + offset])
        if len(m2b_values) == 66:
            positions = [m2b_values[6 + i * 20] for i in range(3)]
            velocities = [m2b_values[7 + i * 20] for i in range(3)]
            row["m2b_delta_z"] = max(positions) - min(positions)
            row["m2b_delta_w"] = max(velocities) - min(velocities)
        output.append(row)
    return output


def convert_bag(bag_path, output_dir, maximum_alignment_age=0.2,
                parquet=False):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    raw = {filename: [] for filename in RAW_SCHEMAS}
    inventory = {}

    with rosbag.Bag(str(bag_path), "r") as bag:
        type_map = {
            topic: connection.msg_type
            for topic, connection in bag.get_type_and_topic_info().topics.items()
        }
        for topic, message, bag_stamp in bag.read_messages():
            current = inventory.setdefault(topic, {
                "type": type_map.get(topic, getattr(message, "_type", "")),
                "messages": 0,
                "first_bag_stamp": stamp_to_sec(bag_stamp),
                "last_bag_stamp": stamp_to_sec(bag_stamp),
            })
            current["messages"] += 1
            current["last_bag_stamp"] = stamp_to_sec(bag_stamp)
            filename, row = _extract(topic, bag_stamp, message)
            if filename:
                raw[filename].append(row)

    for filename, rows in raw.items():
        write_csv(output_dir / filename, rows, RAW_SCHEMAS[filename])
    aligned = _aligned_rows(raw, maximum_alignment_age)
    write_csv(output_dir / "aligned_samples.csv", aligned)
    atomic_dump_yaml(output_dir / "topic_inventory.yaml", {
        "schema_version": 1,
        "bag": str(Path(bag_path).resolve()),
        "causal_alignment": True,
        "maximum_alignment_age": maximum_alignment_age,
        "topics": inventory,
    })

    parquet_written = False
    if parquet:
        try:
            import pandas
            pandas.DataFrame(aligned).to_parquet(
                output_dir / "aligned_samples.parquet", index=False)
            parquet_written = True
        except (ImportError, ValueError) as error:
            raise RuntimeError(
                "Parquet output requires pandas and pyarrow or fastparquet"
            ) from error
    return {
        "output_dir": str(output_dir),
        "topics": len(inventory),
        "aligned_samples": len(aligned),
        "parquet_written": parquet_written,
    }
