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
};

}  // namespace chassis_controller
