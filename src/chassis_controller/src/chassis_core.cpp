#include "chassis_controller/chassis_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chassis_controller {
namespace {

WheelLimits scaled(const WheelLimits& nominal, const DeratingRatios& ratios) {
  return {nominal.max_velocity_left * ratios.speed_left,
          nominal.max_velocity_right * ratios.speed_right,
          nominal.max_acceleration_left * ratios.acceleration_left,
          nominal.max_acceleration_right * ratios.acceleration_right,
          nominal.max_deceleration_left * ratios.deceleration_left,
          nominal.max_deceleration_right * ratios.deceleration_right};
}

double minimumRatio(const DeratingRatios& ratios) {
  return std::min(
      std::min(ratios.speed_left, ratios.speed_right),
      std::min(std::min(ratios.acceleration_left, ratios.acceleration_right),
               std::min(ratios.deceleration_left, ratios.deceleration_right)));
}

bool validPositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool validConfig(const ChassisConfig& config) {
  const auto& limits = config.nominal_limits;
  return config.robot_index >= 1 && config.robot_index <= 3 &&
         validPositive(config.wheel_separation) &&
         validPositive(config.control_loop_overrun_seconds) &&
         validPositive(config.command_timeout_seconds) &&
         validPositive(config.sensor_feedback_timeout_seconds) &&
         std::isfinite(
             config.odometry_stationary_wheel_velocity_tolerance) &&
         config.odometry_stationary_wheel_velocity_tolerance >= 0.0 &&
         config.odometry_stationary_wheel_velocity_tolerance <= 0.02 &&
         validPositive(limits.max_velocity_left) &&
         validPositive(limits.max_velocity_right) &&
         validPositive(limits.max_acceleration_left) &&
         validPositive(limits.max_acceleration_right) &&
         validPositive(limits.max_deceleration_left) &&
         validPositive(limits.max_deceleration_right);
}

bool sameRatios(const DeratingRatios& a, const DeratingRatios& b) {
  return a.speed_left == b.speed_left && a.speed_right == b.speed_right &&
         a.acceleration_left == b.acceleration_left &&
         a.acceleration_right == b.acceleration_right &&
         a.deceleration_left == b.deceleration_left &&
         a.deceleration_right == b.deceleration_right;
}

bool sameDeratingCommand(const DeratingInput& a, const DeratingInput& b) {
  return a.robot_id == b.robot_id && a.sequence == b.sequence &&
         a.active == b.active && a.mode == b.mode &&
         sameRatios(a.ratios, b.ratios) &&
         a.ramp_down_seconds == b.ramp_down_seconds &&
         a.ramp_up_seconds == b.ramp_up_seconds;
}

}  // namespace

ChassisCore::ChassisCore(const ChassisConfig& config)
    : config_(config),
      odometry_(config.odometry_stationary_wheel_velocity_tolerance),
      capability_{0, config.nominal_limits} {
  if (!validConfig(config_)) {
    throw std::invalid_argument("invalid chassis configuration");
  }
}

bool ChassisCore::acceptCommand(const CommandInput& command) {
  if (command.robot_id != config_.robot_index ||
      !std::isfinite(command.raw.left) || !std::isfinite(command.raw.right)) {
    return false;
  }
  if (has_command_ && command.sequence < command_.sequence) {
    return false;
  }
  if (has_command_ && command.sequence == command_.sequence) {
    const bool identical =
        command.robot_id == command_.robot_id &&
        command.raw.left == command_.raw.left &&
        command.raw.right == command_.raw.right;
    if (identical) {
      command_age_seconds_ = 0.0;
      feedback_.command_watchdog_active = false;
      if (std::abs(command.raw.left) <= 1.0e-12 &&
          std::abs(command.raw.right) <= 1.0e-12 &&
          has_sensor_ &&
          sensor_age_seconds_ <= config_.sensor_feedback_timeout_seconds) {
        sensor_watchdog_latched_ = false;
      }
    }
    return identical;
  }
  command_ = command;
  has_command_ = true;
  command_age_seconds_ = 0.0;
  feedback_.command_watchdog_active = false;
  if (std::abs(command.raw.left) <= 1.0e-12 &&
      std::abs(command.raw.right) <= 1.0e-12 &&
      has_sensor_ &&
      sensor_age_seconds_ <= config_.sensor_feedback_timeout_seconds) {
    sensor_watchdog_latched_ = false;
  }
  return true;
}

bool ChassisCore::acceptDerating(const DeratingInput& command) {
  if (command.robot_id != config_.robot_index) return false;
  if (has_derating_command_) {
    if (command.sequence < last_derating_command_.sequence) return false;
    if (command.sequence == last_derating_command_.sequence) {
      return sameDeratingCommand(command, last_derating_command_);
    }
  }
  const auto ratios = command.active ? command.ratios : DeratingRatios::nominal();
  const double ramp = command.active ? command.ramp_down_seconds
                                     : command.ramp_up_seconds;
  if (!derating_.accept(command.sequence, ratios, ramp)) return false;
  derating_mode_ = command.mode;
  last_derating_command_ = command;
  has_derating_command_ = true;
  return true;
}

WheelCommand ChassisCore::step(double dt_seconds) {
  if (has_command_ && std::isfinite(dt_seconds) && dt_seconds > 0.0) {
    command_age_seconds_ += dt_seconds;
  }
  if (has_sensor_ && std::isfinite(dt_seconds) && dt_seconds > 0.0) {
    sensor_age_seconds_ += dt_seconds;
  }
  const bool nonzero_command =
      std::abs(command_.raw.left) > 1.0e-12 ||
      std::abs(command_.raw.right) > 1.0e-12;
  feedback_.command_watchdog_active =
      has_command_ && nonzero_command &&
      command_age_seconds_ > config_.command_timeout_seconds;
  if (has_sensor_ &&
      sensor_age_seconds_ > config_.sensor_feedback_timeout_seconds) {
    sensor_watchdog_latched_ = true;
  }
  feedback_.sensor_watchdog_active = sensor_watchdog_latched_;
  const WheelCommand effective_command =
      (feedback_.command_watchdog_active ||
       feedback_.sensor_watchdog_active) ? WheelCommand{} : command_.raw;

  const auto ratios = derating_.update(dt_seconds);
  capability_.ratios = ratios;
  capability_.limits = scaled(config_.nominal_limits, ratios);
  capability_.derating_ratio = minimumRatio(ratios);
  capability_.derating_mode = derating_mode_;
  capability_.derating_active = capability_.derating_ratio < 1.0;
  capability_.local_revision = derating_.revision();
  ++capability_.sequence;

  const auto limited =
      limiter_.update(effective_command, capability_.limits, dt_seconds);
  if (limited.valid) {
    feedback_.raw = effective_command;
    feedback_.applied = limited.applied;
    feedback_.command_seq_applied = command_.sequence;
    feedback_.speed_limited_left = limited.speed_limited_left;
    feedback_.speed_limited_right = limited.speed_limited_right;
    feedback_.accel_limited_left = limited.accel_limited_left;
    feedback_.accel_limited_right = limited.accel_limited_right;
    feedback_.decel_limited_left = limited.decel_limited_left;
    feedback_.decel_limited_right = limited.decel_limited_right;
  }

  feedback_.control_loop_overrun = !std::isfinite(dt_seconds) ||
      dt_seconds <= 0.0 || dt_seconds > config_.control_loop_overrun_seconds;
  if (std::isfinite(dt_seconds) && dt_seconds > 0.0) {
    if (!feedback_.sensor_watchdog_active) {
      odometry_.update(feedback_.actual.left, feedback_.actual.right,
                       sensor_.imu_yaw_rate, dt_seconds);
    }
  }
  feedback_.odometry = odometry_.state();
  feedback_.linear_velocity = odometry_.linearVelocity();
  feedback_.angular_velocity = odometry_.yawRate();
  ++feedback_.sequence;
  return feedback_.applied;
}

void ChassisCore::updateSensors(const SensorInput& sensor) {
  if (sensor.packet_fresh) {
    sensor_age_seconds_ = 0.0;
  }
  has_sensor_ = true;
  sensor_ = sensor;
  feedback_.actual.left = sensor.wheel_left_mm_per_second / 1000.0;
  feedback_.actual.right = sensor.wheel_right_mm_per_second / 1000.0;
  feedback_.battery_voltage = sensor.battery_voltage;
  feedback_.packet_sequence = sensor.packet_sequence;
}

void ChassisCore::resetOdometry(const Pose2DState& pose) {
  odometry_.reset(pose);
  feedback_.odometry = odometry_.state();
  feedback_.linear_velocity = 0.0;
  feedback_.angular_velocity = 0.0;
}

}  // namespace chassis_controller
