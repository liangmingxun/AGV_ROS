#pragma once

namespace chassis_controller {

struct WheelCommand {
  double left{0.0};
  double right{0.0};
};

struct WheelLimits {
  double max_velocity_left{0.0};
  double max_velocity_right{0.0};
  double max_acceleration_left{0.0};
  double max_acceleration_right{0.0};
  double max_deceleration_left{0.0};
  double max_deceleration_right{0.0};
};

struct WheelLimitResult {
  WheelCommand target{};
  WheelCommand applied{};
  bool speed_limited_left{false};
  bool speed_limited_right{false};
  bool accel_limited_left{false};
  bool accel_limited_right{false};
  bool decel_limited_left{false};
  bool decel_limited_right{false};
  bool valid{false};
};

class WheelCommandLimiter {
 public:
  explicit WheelCommandLimiter(WheelCommand initial = {});

  WheelLimitResult update(const WheelCommand& raw,
                          const WheelLimits& limits,
                          double dt_seconds);
  const WheelCommand& applied() const noexcept { return applied_; }
  void reset(const WheelCommand& applied = {});

 private:
  WheelCommand applied_{};
};

}  // namespace chassis_controller
