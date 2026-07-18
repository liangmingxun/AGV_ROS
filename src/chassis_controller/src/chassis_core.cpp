#include "chassis_controller/chassis_core.hpp"

#include <algorithm>
#include <cmath>

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

}  // namespace

ChassisCore::ChassisCore(const ChassisConfig& config)
    : config_(config), capability_{0, config.nominal_limits} {}

bool ChassisCore::acceptCommand(const CommandInput& command) {
  if (command.robot_id != config_.robot_index ||
      (has_command_ && command.sequence <= command_.sequence) ||
      !std::isfinite(command.raw.left) || !std::isfinite(command.raw.right)) {
    return false;
  }
  command_ = command;
  has_command_ = true;
  return true;
}

bool ChassisCore::acceptDerating(const DeratingInput& command) {
  if (command.robot_id != config_.robot_index) return false;
  const auto ratios = command.active ? command.ratios : DeratingRatios::nominal();
  const double ramp = command.active ? command.ramp_down_seconds
                                     : command.ramp_up_seconds;
  if (!derating_.accept(command.sequence, ratios, ramp)) return false;
  derating_mode_ = command.mode;
  return true;
}

WheelCommand ChassisCore::step(double dt_seconds) {
  const auto ratios = derating_.update(dt_seconds);
  capability_.ratios = ratios;
  capability_.limits = scaled(config_.nominal_limits, ratios);
  capability_.derating_ratio = minimumRatio(ratios);
  capability_.derating_mode = derating_mode_;
  capability_.derating_active = capability_.derating_ratio < 1.0;
  capability_.local_revision = derating_.revision();
  ++capability_.sequence;

  const auto limited = limiter_.update(command_.raw, capability_.limits, dt_seconds);
  if (limited.valid) {
    feedback_.raw = command_.raw;
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
    odometry_.update(feedback_.actual.left, feedback_.actual.right,
                     sensor_.imu_yaw_rate, dt_seconds);
  }
  feedback_.odometry = odometry_.state();
  feedback_.linear_velocity = odometry_.linearVelocity();
  feedback_.angular_velocity = odometry_.yawRate();
  ++feedback_.sequence;
  return feedback_.applied;
}

void ChassisCore::updateSensors(const SensorInput& sensor) {
  sensor_ = sensor;
  feedback_.actual.left = sensor.wheel_left_mm_per_second / 1000.0;
  feedback_.actual.right = sensor.wheel_right_mm_per_second / 1000.0;
  feedback_.battery_voltage = sensor.battery_voltage;
  feedback_.packet_sequence = sensor.packet_sequence;
}

}  // namespace chassis_controller
