#pragma once

#include <cstdint>

#include "chassis_controller/derating_profile.hpp"
#include "chassis_controller/odometry_integrator.hpp"
#include "chassis_controller/wheel_command_limiter.hpp"

namespace chassis_controller {

struct ChassisConfig {
  std::uint8_t robot_index{0};
  double wheel_separation{0.114};
  WheelLimits nominal_limits{};
  double control_loop_overrun_seconds{0.008};
  double command_timeout_seconds{0.20};
  double sensor_feedback_timeout_seconds{0.15};
  double odometry_stationary_wheel_velocity_tolerance{0.005};
};

struct CommandInput {
  std::uint8_t robot_id{0};
  std::uint32_t sequence{0};
  WheelCommand raw{};
};

struct DeratingInput {
  std::uint8_t robot_id{0};
  std::uint32_t sequence{0};
  bool active{false};
  std::uint8_t mode{0};
  DeratingRatios ratios{};
  double ramp_down_seconds{0.0};
  double ramp_up_seconds{0.0};
};

struct SensorInput {
  double wheel_left_mm_per_second{0.0};
  double wheel_right_mm_per_second{0.0};
  double imu_yaw_rate{0.0};
  double battery_voltage{0.0};
  std::uint32_t packet_sequence{0};
  bool packet_fresh{true};
};

struct FeedbackState {
  std::uint32_t sequence{0};
  std::uint32_t command_seq_applied{0};
  std::uint32_t packet_sequence{0};
  WheelCommand raw{};
  WheelCommand applied{};
  WheelCommand actual{};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double battery_voltage{0.0};
  Pose2DState odometry{};
  bool control_loop_overrun{false};
  bool command_watchdog_active{false};
  bool sensor_watchdog_active{false};
  bool speed_limited_left{false};
  bool speed_limited_right{false};
  bool accel_limited_left{false};
  bool accel_limited_right{false};
  bool decel_limited_left{false};
  bool decel_limited_right{false};
};

struct CapabilityState {
  std::uint32_t sequence{0};
  WheelLimits limits{};
  DeratingRatios ratios{};
  double derating_ratio{1.0};
  std::uint8_t derating_mode{0};
  bool derating_active{false};
  std::uint64_t local_revision{0};
};

class ChassisCore {
 public:
  explicit ChassisCore(const ChassisConfig& config);

  bool acceptCommand(const CommandInput& command);
  bool acceptDerating(const DeratingInput& command);
  WheelCommand step(double dt_seconds);
  void updateSensors(const SensorInput& sensor);
  void resetOdometry(const Pose2DState& pose = {});
  const FeedbackState& feedback() const noexcept { return feedback_; }
  const CapabilityState& capability() const noexcept { return capability_; }

 private:
  ChassisConfig config_;
  WheelCommandLimiter limiter_;
  DeratingProfile derating_;
  OdometryIntegrator odometry_;
  CommandInput command_{};
  SensorInput sensor_{};
  FeedbackState feedback_{};
  CapabilityState capability_{};
  std::uint8_t derating_mode_{0};
  bool has_command_{false};
  double command_age_seconds_{0.0};
  bool has_sensor_{false};
  double sensor_age_seconds_{0.0};
  bool sensor_watchdog_latched_{false};
  DeratingInput last_derating_command_{};
  bool has_derating_command_{false};
};

}  // namespace chassis_controller
