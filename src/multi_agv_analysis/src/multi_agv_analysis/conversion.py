"""Lossless field extraction plus causal alignment for frozen ROS bags."""

import bisect
import json
import math
from collections import defaultdict
from pathlib import Path

import rosbag

from .io_utils import atomic_dump_yaml, finite_float, stamp_to_sec, write_csv


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
        "wheel_right_applied", "wheel_left_firmware_target_nominal",
        "wheel_right_firmware_target_nominal",
        "wheel_left_firmware_feedback_nominal",
        "wheel_right_firmware_feedback_nominal",
        "wheel_left_actual", "wheel_right_actual",
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
        "topic", "bag_stamp", "header_stamp", "frame_id",
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
        "topic", "bag_stamp", "header_stamp", "frame_id", "path_id", "path_version",
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
        "path_progress_execute_reference_1",
        "path_progress_execute_reference_2",
        "path_progress_execute_reference_3",
        "path_velocity_execute_reference_1",
        "path_velocity_execute_reference_2",
        "path_velocity_execute_reference_3",
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
    "fused_motion.csv": [
        "topic", "bag_stamp", "header_stamp", "frame_id", "child_frame_id",
        "position_x", "position_y", "position_z", "orientation_x",
        "orientation_y", "orientation_z", "orientation_w", "linear_x",
        "linear_y", "angular_z",
    ],
    "formal_algorithm_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "state_chain_timing.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "formal_execution_limiter_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "m2b_algorithm_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "risk_disturbance_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "transient_yaw_disturbance_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "yaw_effectiveness_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "yaw_effectiveness_hold_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "classic_additive_disturbance_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "classic_additive_physical_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "v5_exploration_quality_state.csv": [
        "topic", "bag_stamp", "header_stamp", "layout_label", "data_json",
    ],
    "yaw_drive_disturbance_state.csv": [
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
            # getattr keeps conversion compatible with bags recorded before
            # the Stage-D diagnostics were added to ChassisFeedback.
            "wheel_left_firmware_target_nominal": getattr(
                message, "wheel_firmware_target_left_nominal_mps", math.nan),
            "wheel_right_firmware_target_nominal": getattr(
                message, "wheel_firmware_target_right_nominal_mps", math.nan),
            "wheel_left_firmware_feedback_nominal": getattr(
                message, "wheel_firmware_feedback_left_nominal_mps", math.nan),
            "wheel_right_firmware_feedback_nominal": getattr(
                message, "wheel_firmware_feedback_right_nominal_mps", math.nan),
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
        row["frame_id"] = message.header.frame_id
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
        row["frame_id"] = message.header.frame_id
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
        _array_fields(
            row, "path_progress_execute_reference",
            message.path_progress_execute_reference)
        _array_fields(
            row, "path_velocity_execute_reference",
            message.path_velocity_execute_reference)
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
        if topic in {"/pose_provider/agv{}/base_motion_fused".format(n) for n in range(1,4)}:
            return "fused_motion.csv", row
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
        if topic == "/multi_agv/formal_execution_limiter_state":
            return "formal_execution_limiter_state.csv", row
        if topic == "/multi_agv/formal_algorithm_state":
            return "formal_algorithm_state.csv", row
        if topic == "/multi_agv/risk_disturbance_state":
            return "risk_disturbance_state.csv", row
        if topic == "/multi_agv/yaw_drive_disturbance_state":
            return "yaw_drive_disturbance_state.csv", row
        if topic == "/multi_agv/transient_yaw_disturbance_state":
            return "transient_yaw_disturbance_state.csv", row
        if topic == "/multi_agv/yaw_effectiveness_state":
            return "yaw_effectiveness_state.csv", row
        if topic == "/multi_agv/yaw_effectiveness_hold_state":
            return "yaw_effectiveness_hold_state.csv", row
        if topic == "/multi_agv/classic_additive_disturbance_state":
            return "classic_additive_disturbance_state.csv", row
        if topic == "/multi_agv/classic_additive_physical_state":
            return "classic_additive_physical_state.csv", row
        if topic == "/multi_agv/v5_exploration_quality_state":
            return "v5_exploration_quality_state.csv", row
        if topic == "/multi_agv/state_projection_timing":
            values = list(message.data)
            if values and math.isfinite(values[0]):
                row["header_stamp"] = values[0]
            return "state_chain_timing.csv", row
        if topic in {
                "/pose_provider/agv{}/{}".format(robot, suffix)
                for robot in range(1, 4)
                for suffix in ("fusion_timing", "estimator_timing")}:
            return "state_chain_timing.csv", row
        # Sharing a ROS message type does not make arbitrary diagnostics an
        # algorithm state. Unknown arrays remain in the original bag.
        return None, None
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


def _yaw_from_quaternion(row):
    x = finite_float(row.get("orientation_x"))
    y = finite_float(row.get("orientation_y"))
    z = finite_float(row.get("orientation_z"))
    w = finite_float(row.get("orientation_w"))
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return math.nan
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-12:
        return math.nan
    x, y, z, w = (value/norm for value in (x, y, z, w))
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def _equivalent_load_pose(state, path):
    """Rigidly map the reference load through the measured support fit."""
    if not path:
        return (math.nan, math.nan, math.nan)
    actual = [(finite_float(state.get("support_pose_x_{}".format(index))),
               finite_float(state.get("support_pose_y_{}".format(index))))
              for index in range(1, 4)]
    reference = [(finite_float(path.get(
                    "support_x_reference_{}".format(index))),
                  finite_float(path.get(
                    "support_y_reference_{}".format(index))))
                 for index in range(1, 4)]
    load_reference = (finite_float(path.get("load_x_reference")),
                      finite_float(path.get("load_y_reference")),
                      finite_float(path.get("load_yaw_reference")))
    if not all(math.isfinite(value) for point in actual + reference
               for value in point) or not all(
                   math.isfinite(value) for value in load_reference):
        return (math.nan, math.nan, math.nan)
    actual_center = tuple(sum(point[axis] for point in actual) / 3.0
                          for axis in range(2))
    reference_center = tuple(sum(point[axis] for point in reference) / 3.0
                             for axis in range(2))
    # Invalid/terminal PathReference messages may contain three default zero
    # support poses. Their rigid transform is undefined, not a measured pose.
    if any(sum((point[axis] - center[axis]) ** 2
               for point in points for axis in range(2)) <= 1e-12
           for points, center in ((reference, reference_center),
                                  (actual, actual_center))):
        return (math.nan, math.nan, math.nan)
    dot = sum((reference[index][0] - reference_center[0]) *
              (actual[index][0] - actual_center[0]) +
              (reference[index][1] - reference_center[1]) *
              (actual[index][1] - actual_center[1]) for index in range(3))
    cross = sum((reference[index][0] - reference_center[0]) *
                (actual[index][1] - actual_center[1]) -
                (reference[index][1] - reference_center[1]) *
                (actual[index][0] - actual_center[0]) for index in range(3))
    if math.hypot(dot, cross) <= 1e-12:
        return (math.nan, math.nan, math.nan)
    yaw = math.atan2(cross, dot)
    dx = load_reference[0] - reference_center[0]
    dy = load_reference[1] - reference_center[1]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (actual_center[0] + cosine * dx - sine * dy,
            actual_center[1] + sine * dx + cosine * dy,
            load_reference[2] + yaw)


def _measured_payload_in_reference(measured, state, path, transform):
    """Require an exact frame match or an explicit, frame-bound SE(2) transform."""
    source = str(measured.get("frame_id", "")) if measured else ""
    target = str(path.get("frame_id", "")) if path else ""
    state_frame = str(state.get("frame_id", ""))
    target = target or state_frame
    invalid = (math.nan, math.nan, math.nan)
    if not measured or not source or not target or (state_frame and state_frame != target):
        return invalid, "missing_or_inconsistent_frame"
    pose = (finite_float(measured.get("position_x")),
            finite_float(measured.get("position_y")), _yaw_from_quaternion(measured))
    if not all(math.isfinite(value) for value in pose):
        return invalid, "invalid_pose"
    if source == target:
        return pose, "same_frame"
    if not isinstance(transform, dict) or transform.get("source_frame") != source or transform.get("target_frame") != target:
        return invalid, "frame_mismatch"
    tx, ty, yaw = (finite_float(transform.get(key)) for key in ("x", "y", "yaw"))
    if not all(math.isfinite(value) for value in (tx, ty, yaw)):
        return invalid, "invalid_transform"
    c, s = math.cos(yaw), math.sin(yaw)
    return (tx+c*pose[0]-s*pose[1], ty+s*pose[0]+c*pose[1],
            math.atan2(math.sin(pose[2]+yaw), math.cos(pose[2]+yaw))), "explicit_transform"


def _aligned_rows(raw, maximum_age,
                  measured_payload_pose_topic=
                  "/pose_provider/load/pose_filtered", measured_payload_transform=None):
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
    formal_debug = _series([
        value for value in raw["formal_algorithm_state.csv"]
        if value.get("topic", "/multi_agv/formal_algorithm_state") ==
        "/multi_agv/formal_algorithm_state"])
    execution_limiter = _series(
        raw["formal_execution_limiter_state.csv"])
    m2b_debug = _series(raw["m2b_algorithm_state.csv"])
    risk_debug = _series(raw.get("risk_disturbance_state.csv", []))
    yaw_debug = _series(raw.get("yaw_drive_disturbance_state.csv", []))
    transient_debug = _series(raw.get("transient_yaw_disturbance_state.csv", []))
    effectiveness_debug = _series(raw.get("yaw_effectiveness_state.csv", []))
    hold_effectiveness_debug = _series(raw.get("yaw_effectiveness_hold_state.csv", []))
    classic_additive_debug = _series(raw.get("classic_additive_disturbance_state.csv", []))
    classic_additive_physical_debug = _series(
        raw.get("classic_additive_physical_state.csv", []))
    experiment_states = _series(raw["experiment_state.csv"])
    measured_load_poses = _series([
        value for value in raw["camera_pose.csv"]
        if value.get("topic") == measured_payload_pose_topic])
    robot2_derating = _series([
        value for value in raw["derating_command.csv"]
        if int(value.get("robot_id", 0)) == 2])
    has_experiment_state_stream = bool(experiment_states[0])
    projections = _series([
        value for value in raw["state_chain_timing.csv"]
        if value.get("topic") == "/multi_agv/state_projection_timing"])

    output = []
    for state in anchors:
        stamp = float(state["header_stamp"])
        path = _latest(paths, stamp, maximum_age)
        controller = _latest(controllers, stamp, maximum_age)
        debug = _latest(formal_debug, stamp, maximum_age)
        limiter = _latest(execution_limiter, stamp, maximum_age)
        m2b = _latest(m2b_debug, stamp, maximum_age)
        risk = _latest(risk_debug, stamp, maximum_age)
        yaw = _latest(yaw_debug, stamp, maximum_age)
        transient = _latest(transient_debug, stamp, maximum_age)
        classic_additive = _latest(classic_additive_debug, stamp, maximum_age)
        classic_additive_physical = _latest(
            classic_additive_physical_debug, stamp, maximum_age)
        experiment_state = _latest(
            experiment_states, stamp, maximum_age)
        measured_load = _latest(measured_load_poses, stamp, maximum_age)
        derating = _latest(robot2_derating, stamp, maximum_age)
        projection = _latest(projections, stamp, maximum_age)
        projection_values = json.loads(projection["data_json"]) if projection else []
        # Never associate a previous estimator tick with a new state.
        if len(projection_values) != 18 or abs(projection_values[0] - stamp) > 1e-6:
            projection_values = []
        controller_method = (
            str(controller.get("method_id", "")) if controller else "")
        engineering_baseline = (
            controller_method == "CAMERA_IMU_WHEEL_FUSED_CLOSED_LOOP")
        equivalent_load = _equivalent_load_pose(state, path)
        measured_pose, measured_status = _measured_payload_in_reference(
            measured_load, state, path, measured_payload_transform)
        row = {
            "stamp": stamp,
            "localization_valid": _valid_cooperative(state),
            "load_s_actual": state.get("load_s_actual", math.nan),
            "load_s_dot_actual": state.get("load_s_dot_actual", math.nan),
            "load_pose_x": state.get("load_pose_x", math.nan),
            "load_pose_y": state.get("load_pose_y", math.nan),
            "load_pose_yaw": state.get("load_pose_yaw", math.nan),
            "equivalent_load_pose_x": equivalent_load[0],
            "equivalent_load_pose_y": equivalent_load[1],
            "equivalent_load_pose_yaw": equivalent_load[2],
            "reference_frame": (path.get("frame_id", "") if path else "") or state.get("frame_id", ""),
            "measured_load_source_frame": measured_load.get("frame_id", "") if measured_load else "",
            "measured_load_alignment_status": measured_status,
            "measured_load_pose_x": measured_pose[0],
            "measured_load_pose_y": measured_pose[1],
            "measured_load_pose_yaw": measured_pose[2],
            "measured_load_pose_valid": all(math.isfinite(value) for value in measured_pose),
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
            "public_reference_velocity": (
                controller.get("common_velocity_reference", math.nan)
                if controller else math.nan),
            "common_boundary_lower": (
                math.nan if engineering_baseline else
                controller.get("common_velocity_lower_bound", math.nan)
                if controller else math.nan),
            "common_boundary_upper": (
                math.nan if engineering_baseline else
                controller.get("common_velocity_upper_bound", math.nan)
                if controller else math.nan),
            "evaluation_active": (
                experiment_state.get("evaluation_active", False)
                if experiment_state else not has_experiment_state_stream),
            "experiment_state_available": experiment_state is not None,
            "manual_abort": (
                experiment_state.get("manual_abort", False)
                if experiment_state else False),
            "agv2_derating_command_available": derating is not None,
            "agv2_derating_active": (
                derating.get("active", False) if derating else False),
            "agv2_derating_speed_ratio": (
                min(float(derating["target_speed_ratio_left"]),
                    float(derating["target_speed_ratio_right"]))
                if derating else math.nan),
            "agv2_derating_acceleration_ratio": (
                min(float(derating["target_accel_ratio_left"]),
                    float(derating["target_accel_ratio_right"]))
                if derating else math.nan),
            "agv2_derating_deceleration_ratio": (
                min(float(derating["target_decel_ratio_left"]),
                    float(derating["target_decel_ratio_right"]))
                if derating else math.nan),
            "agv2_derating_command_seq": (
                derating.get("command_seq", math.nan)
                if derating else math.nan),
        }
        debug_values = []
        debug_version = 0
        if (debug and debug.get("layout_label") in (
                "formal_algorithm_state_v1:header9+3x27",
                "formal_algorithm_state_v2:header10+3x29",
                "formal_algorithm_state_v3:header10+3x29+reference4")):
            try:
                debug_values = json.loads(debug["data_json"])
                debug_version = (3 if debug.get("layout_label") ==
                                 "formal_algorithm_state_v3:header10+3x29+reference4"
                                 else 2 if debug.get("layout_label") ==
                                 "formal_algorithm_state_v2:header10+3x29"
                                 else 1)
            except (TypeError, ValueError):
                debug_values = []
                debug_version = 0
        limiter_values = []
        if (limiter and limiter.get("layout_label") ==
                "formal_execution_limiter_v1:header4+3x7"):
            try:
                limiter_values = json.loads(limiter["data_json"])
            except (TypeError, ValueError):
                limiter_values = []
        row["fleet_scale"] = (
            limiter_values[2] if len(limiter_values) == 25 else
            1.0 if engineering_baseline else math.nan)
        row["wheel_pre_limit_peak"] = (
            limiter_values[3] if len(limiter_values) == 25 else math.nan)
        row["mapped_path_capability_available"] = (
            bool(debug_values[9])
            if debug_version >= 2 and len(debug_values) in (97, 101) else False)
        expected_debug_size = 101 if debug_version == 3 else 97 if debug_version == 2 else 90
        row["algorithm_state_available"] = (
            debug_version != 0 and len(debug_values) == expected_debug_size)
        row["algorithm_valid"] = (
            bool(debug_values[0])
            if row["algorithm_state_available"] else False)
        row["candidate_common_velocity"] = (
            sum(debug_values[97:100]) / 3.0
            if debug_version == 3 and len(debug_values) == 101 else math.nan)
        row["upper_effective_common_velocity"] = (
            debug_values[100]
            if debug_version == 3 and len(debug_values) == 101 else math.nan)
        risk_values = []
        if (risk and risk.get("layout_label") ==
                "risk_disturbance_state_v1:header11+3x10"):
            try:
                risk_values = json.loads(risk["data_json"])
            except (TypeError, ValueError):
                risk_values = []
        row["risk_disturbance_state_available"] = len(risk_values) == 41
        row["risk_disturbance_enabled"] = (
            bool(risk_values[0]) if len(risk_values) == 41 else False)
        row["risk_disturbance_peak_fraction"] = (
            risk_values[6] if len(risk_values) == 41 else math.nan)
        row["hard_common_upper"] = (
            risk_values[8] if len(risk_values) == 41 else math.nan)
        row["baseline_common_upper"] = (
            risk_values[9] if len(risk_values) == 41 else math.nan)
        row["risk_contraction"] = (
            risk_values[10] if len(risk_values) == 41 else math.nan)
        yaw_values = []
        if (yaw and yaw.get("layout_label") ==
                "yaw_drive_disturbance_v2:header15"):
            try:
                yaw_values = json.loads(yaw["data_json"])
            except (TypeError, ValueError):
                yaw_values = []
        yaw_names = (
            "enabled", "active", "source_stamp", "robot_id",
            "path_progress", "window", "phase", "amplitude",
            "signed_disturbance", "left_nominal", "right_nominal",
            "left_disturbed", "right_disturbed",
            "mean_longitudinal_delta", "yaw_differential_delta")
        row["yaw_drive_disturbance_state_available"] = len(yaw_values) == 15
        for offset, name in enumerate(yaw_names):
            row["yaw_drive_disturbance_{}".format(name)] = (
                yaw_values[offset] if len(yaw_values) == 15 else math.nan)
        transient_values = []
        if transient and transient.get("layout_label") == "transient_yaw_v4:header19":
            try:
                transient_values = json.loads(transient["data_json"])
            except (ValueError, TypeError):
                pass
        row["transient_yaw_state_available"] = len(transient_values) == 19
        for offset, name in enumerate((
                "source_stamp", "wall_time", "trigger_wall_time", "triggered",
                "active", "finished", "elapsed", "progress", "amplitude",
                "duration", "envelope", "disturbance", "left_raw", "right_raw",
                "left_disturbed", "right_disturbed", "mean_longitudinal_delta",
                "yaw_differential_delta", "causal_wheel_margin")):
            row["transient_yaw_" + name] = (
                transient_values[offset] if len(transient_values) == 19 else math.nan)
        m2b_values = []
        effectiveness = _latest(effectiveness_debug, stamp, maximum_age)
        effectiveness_values = []
        if effectiveness and effectiveness.get("layout_label") == "yaw_effectiveness_v5:header19+3x3":
            try:
                effectiveness_values = json.loads(effectiveness["data_json"])
            except (ValueError, TypeError):
                pass
        row["yaw_effectiveness_state_available"] = len(effectiveness_values) == 28
        for offset, name in enumerate(("source_stamp", "wall_time", "trigger_wall_time", "triggered",
                "active", "finished", "elapsed", "progress", "gamma_min", "duration", "envelope", "gamma",
                "left_raw", "right_raw", "left_degraded", "right_degraded", "mean_longitudinal_delta",
                "yaw_differential_delta", "causal_wheel_margin")):
            row["yaw_effectiveness_"+name] = effectiveness_values[offset] if len(effectiveness_values)==28 else math.nan
        for i in range(3):
            for j, name in enumerate(("longitudinal_error", "lateral_error", "heading_error")):
                row["agv{}_tracker_{}".format(i+1,name)] = effectiveness_values[19+3*i+j] if len(effectiveness_values)==28 else math.nan
        effectiveness = _latest(hold_effectiveness_debug, stamp, maximum_age)
        effectiveness_values = []
        if effectiveness and effectiveness.get("layout_label") == "yaw_hold_v5b:header19+3x3":
            try:
                effectiveness_values = json.loads(effectiveness["data_json"])
            except (ValueError, TypeError):
                pass
        row["yaw_effectiveness_hold_state_available"] = len(effectiveness_values) == 28
        for offset, name in enumerate(("source_stamp", "wall_time", "trigger_wall_time", "triggered",
                "active", "finished", "elapsed", "progress", "gamma_min", "duration", "envelope", "gamma",
                "left_raw", "right_raw", "left_degraded", "right_degraded", "mean_longitudinal_delta",
                "yaw_differential_delta", "causal_wheel_margin")):
            row["yaw_effectiveness_hold_"+name] = effectiveness_values[offset] if len(effectiveness_values)==28 else math.nan
        for i in range(3):
            for j, name in enumerate(("longitudinal_error", "lateral_error", "heading_error")):
                row["agv{}_hold_tracker_{}".format(i+1,name)] = effectiveness_values[19+3*i+j] if len(effectiveness_values)==28 else math.nan
        classic_values = []
        if (classic_additive and classic_additive.get("layout_label") ==
                "classic_additive_candidate_A:header27+3x3"):
            try:
                classic_values = json.loads(classic_additive["data_json"])
            except (ValueError, TypeError):
                pass
        row["classic_additive_state_available"] = len(classic_values) == 36
        for offset, name in enumerate((
                "source_stamp", "wall_time", "trigger_wall_time", "triggered",
                "active", "finished", "elapsed", "progress",
                "linear_amplitude", "angular_amplitude", "linear_frequency",
                "angular_frequency", "angular_phase", "duration", "ramp_in",
                "ramp_out", "wheel_separation", "envelope", "d_v", "d_omega",
                "left_raw", "right_raw", "left_disturbed", "right_disturbed",
                "mean_delta", "angular_delta", "robot_id")):
            row["classic_additive_"+name] = (
                classic_values[offset] if len(classic_values)==36 else math.nan)
        for i in range(3):
            for j, name in enumerate(("longitudinal_error", "lateral_error", "heading_error")):
                row["agv{}_classic_tracker_{}".format(i+1, name)] = (
                    classic_values[27+3*i+j] if len(classic_values)==36 else math.nan)
        physical_values = []
        if (classic_additive_physical and
                classic_additive_physical.get("layout_label") ==
                "classic_additive_physical_v1:header21+robot2_feedback2"):
            try:
                physical_values = json.loads(
                    classic_additive_physical["data_json"])
            except (ValueError, TypeError):
                pass
        row["classic_additive_physical_state_available"] = (
            len(physical_values) == 23)
        for offset, name in enumerate((
                "source_stamp", "wall_time", "trigger_wall_time", "triggered",
                "active", "finished", "elapsed", "progress", "scale",
                "linear_amplitude_actual", "angular_amplitude_actual",
                "envelope", "d_v", "d_omega", "wheel_pre_left",
                "wheel_pre_right", "wheel_post_disturbance_left",
                "wheel_post_disturbance_right", "wheel_post_limit_left",
                "wheel_post_limit_right", "robot_id", "wheel_actual_left",
                "wheel_actual_right")):
            row["classic_additive_physical_"+name] = (
                physical_values[offset]
                if len(physical_values) == 23 else math.nan)
        if (m2b and m2b.get("layout_label") ==
                "m2b_algorithm_state_v1:header6+3x20"):
            try:
                m2b_values = json.loads(m2b["data_json"])
            except (TypeError, ValueError):
                m2b_values = []
        for robot in range(1, 4):
            risk_offset = 11 + (robot - 1) * 10
            for offset, suffix in enumerate((
                    "disturbance_window", "disturbance_base",
                    "disturbance_velocity", "disturbance_sinusoid",
                    "disturbance_total", "mapped_capability_diagnostic",
                    "hard_inner_upper", "baseline_inner_upper",
                    "risk_inner_upper", "effective_inner_upper")):
                row["agv{}_{}".format(robot, suffix)] = (
                    risk_values[risk_offset + offset]
                    if len(risk_values) == 41 else math.nan)
            for pose_name in ("robot_pose", "support_pose"):
                for axis in ("x", "y", "yaw"):
                    field = "{}_{}_{}".format(pose_name, axis, robot)
                    row["agv{}_{}_{}".format(
                        robot, pose_name, axis)] = state.get(field, math.nan)
            row["agv{}_s_actual".format(robot)] = state.get(
                "s_actual_{}".format(robot), math.nan)
            # Match algorithmPositionActual: preserve geometric evidence and
            # separately expose the controller's recorded origin alignment.
            origin = (limiter_values[4 + (robot - 1) * 7 + 6]
                      if len(limiter_values) == 25 else
                      0.0 if engineering_baseline else math.nan)
            row["agv{}_s_initial_offset".format(robot)] = origin
            row["agv{}_s_tracking_actual".format(robot)] = (
                finite_float(row["agv{}_s_actual".format(robot)]) -
                finite_float(origin))
            prefix = "agv{}".format(robot)
            row[prefix + "_s_origin_aligned_actual"] = row[prefix + "_s_tracking_actual"]
            # Follow the recorded controller coordinate, never infer a newer
            # controller definition for a historical bag.
            recorded_actual = finite_float(controller.get(
                "path_progress_actual_{}".format(robot))) if controller else math.nan
            if math.isfinite(recorded_actual):
                row[prefix + "_s_tracking_actual"] = recorded_actual
            row[prefix + "_progress_tracking_definition"] = (
                "controller_recorded_actual_minus_public_reference"
                if math.isfinite(recorded_actual) else
                "recorded_origin_aligned_actual_minus_public_reference")
            source_stamp = finite_float(state.get("robot_pose_stamp_{}".format(robot)))
            measured_s = finite_float(row[prefix + "_s_actual"])
            projected_flag = False
            held_flag = False
            if projection_values:
                index = 3 + (robot - 1) * 5
                source_stamp = projection_values[index]
                projected_flag = bool(projection_values[index + 2])
                measured_s = projection_values[index + 3]
                held_flag = bool(projection_values[2])
            measured_reference = (_latest(paths, source_stamp, maximum_age)
                                  if math.isfinite(source_stamp) and source_stamp > 0 else None)
            row[prefix + "_pose_source_stamp"] = source_stamp
            row[prefix + "_pose_source_age"] = stamp - source_stamp if source_stamp > 0 else math.nan
            row[prefix + "_motion_projected"] = projected_flag
            row[prefix + "_snapshot_held"] = held_flag
            row[prefix + "_measured_s_tracking_actual"] = measured_s - finite_float(origin)
            row[prefix + "_measurement_time_progress_error"] = (
                measured_s - finite_float(origin) - finite_float(
                    measured_reference.get("load_s_reference"))
                if measured_reference else math.nan)
            row["agv{}_s_dot_actual".format(robot)] = state.get(
                "s_dot_actual_{}".format(robot), math.nan)
            row["agv{}_s_execute_reference".format(robot)] = (
                controller.get(
                    "path_progress_execute_reference_{}".format(robot),
                    math.nan) if controller else math.nan)
            row["agv{}_s_dot_execute_reference".format(robot)] = (
                controller.get(
                    "path_velocity_execute_reference_{}".format(robot),
                    math.nan) if controller else math.nan)
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
                for diagnostic in (
                        "firmware_target_nominal",
                        "firmware_feedback_nominal"):
                    row[prefix + "_" + diagnostic] = (
                        current_feedback.get(
                            "wheel_{}_{}".format(side, diagnostic),
                            math.nan)
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
            limiter_offset = 4 + (robot - 1) * 7
            if len(limiter_values) == 25:
                row["agv{}_wheel_left_pre_limit".format(robot)] = (
                    limiter_values[limiter_offset])
                row["agv{}_wheel_right_pre_limit".format(robot)] = (
                    limiter_values[limiter_offset + 1])
                row["agv{}_wheel_left_fleet_scaled".format(robot)] = (
                    limiter_values[limiter_offset + 2])
                row["agv{}_wheel_right_fleet_scaled".format(robot)] = (
                    limiter_values[limiter_offset + 3])
            else:
                # The engineering baseline has no formal fleet scaler.  Its
                # ChassisFeedback raw field is therefore the true planar
                # execution demand and is preserved as both pre-limit and
                # fleet-scaled demand.  Other methods must never infer these
                # stages when their dedicated limiter debug stream is absent.
                baseline_left = row.get(
                    "agv{}_wheel_left_raw".format(robot), math.nan)
                baseline_right = row.get(
                    "agv{}_wheel_right_raw".format(robot), math.nan)
                row["agv{}_wheel_left_pre_limit".format(robot)] = (
                    baseline_left if engineering_baseline else math.nan)
                row["agv{}_wheel_right_pre_limit".format(robot)] = (
                    baseline_right if engineering_baseline else math.nan)
                row["agv{}_wheel_left_fleet_scaled".format(robot)] = (
                    baseline_left if engineering_baseline else math.nan)
                row["agv{}_wheel_right_fleet_scaled".format(robot)] = (
                    baseline_right if engineering_baseline else math.nan)
            row["agv{}_reported_velocity_limit".format(robot)] = (
                min(float(current_capability["max_wheel_velocity_left"]),
                    float(current_capability["max_wheel_velocity_right"]))
                if current_capability else math.nan)
            row["agv{}_capability_derating_active".format(robot)] = (
                current_capability.get("derating_active", False)
                if current_capability else False)
            row["agv{}_capability_derating_ratio".format(robot)] = (
                current_capability.get("derating_ratio", math.nan)
                if current_capability else math.nan)
            debug_header = 10 if debug_version >= 2 else 9
            debug_fields = 29 if debug_version >= 2 else 27
            debug_offset = debug_header + (robot - 1) * debug_fields
            if len(debug_values) >= debug_offset + debug_fields:
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
                if debug_version >= 2 and bool(debug_values[9]):
                    row["agv{}_mapped_path_velocity_lower".format(robot)] = (
                        debug_values[debug_offset + 27])
                    row["agv{}_mapped_path_velocity_upper".format(robot)] = (
                        debug_values[debug_offset + 28])
                else:
                    row["agv{}_mapped_path_velocity_lower".format(robot)] = (
                        math.nan)
                    row["agv{}_mapped_path_velocity_upper".format(robot)] = (
                        math.nan)
            else:
                row["agv{}_mapped_path_velocity_lower".format(robot)] = (
                    math.nan)
                row["agv{}_mapped_path_velocity_upper".format(robot)] = (
                    math.nan)
            if engineering_baseline:
                row["agv{}_boundary_lower".format(robot)] = math.nan
                row["agv{}_boundary_upper".format(robot)] = math.nan
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
                parquet=False,
                measured_payload_pose_topic=
                "/pose_provider/load/pose_filtered", measured_payload_transform=None):
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
    aligned = _aligned_rows(
        raw, maximum_alignment_age, measured_payload_pose_topic, measured_payload_transform)
    write_csv(output_dir / "aligned_samples.csv", aligned)
    atomic_dump_yaml(output_dir / "topic_inventory.yaml", {
        "schema_version": 1,
        "bag": str(Path(bag_path).resolve()),
        "causal_alignment": True,
        "maximum_alignment_age": maximum_alignment_age,
        "measured_payload_pose_topic": measured_payload_pose_topic,
        "measured_payload_transform": measured_payload_transform,
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
