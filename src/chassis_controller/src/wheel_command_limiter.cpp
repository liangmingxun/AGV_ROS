#include "chassis_controller/wheel_command_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace chassis_controller {
namespace {

bool finitePositiveOrZero(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double clampMagnitude(double value, double maximum) {
  return std::max(-maximum, std::min(value, maximum));
}

struct SingleWheelResult {
  double target{0.0};
  double applied{0.0};
  bool speed_limited{false};
  bool accel_limited{false};
  bool decel_limited{false};
};

SingleWheelResult limitWheel(double raw, double previous, double max_velocity,
                             double max_acceleration, double max_deceleration,
                             double dt) {
  SingleWheelResult result;
  result.target = clampMagnitude(raw, max_velocity);
  result.speed_limited = result.target != raw;

  const bool reversing = previous != 0.0 && result.target != 0.0 &&
                         std::signbit(previous) != std::signbit(result.target);
  if (reversing) {
    const double change = max_deceleration * dt;
    result.applied = std::copysign(std::max(0.0, std::abs(previous) - change),
                                   previous);
    result.decel_limited = change < std::abs(previous);
  } else {
    const double delta = result.target - previous;
    const bool increasing_magnitude = std::abs(result.target) > std::abs(previous);
    const double max_change =
        (increasing_magnitude ? max_acceleration : max_deceleration) * dt;
    if (std::abs(delta) > max_change) {
      result.applied = previous + std::copysign(max_change, delta);
      result.accel_limited = increasing_magnitude;
      result.decel_limited = !increasing_magnitude;
    } else {
      result.applied = result.target;
    }
  }

  const double final_value = clampMagnitude(result.applied, max_velocity);
  result.speed_limited = result.speed_limited || final_value != result.applied;
  result.applied = final_value;
  return result;
}

}  // namespace

WheelCommandLimiter::WheelCommandLimiter(WheelCommand initial) : applied_(initial) {}

WheelLimitResult WheelCommandLimiter::update(const WheelCommand& raw,
                                              const WheelLimits& limits,
                                              double dt_seconds) {
  WheelLimitResult result;
  result.applied = applied_;
  const bool valid = std::isfinite(raw.left) && std::isfinite(raw.right) &&
                     std::isfinite(dt_seconds) && dt_seconds > 0.0 &&
                     finitePositiveOrZero(limits.max_velocity_left) &&
                     finitePositiveOrZero(limits.max_velocity_right) &&
                     finitePositiveOrZero(limits.max_acceleration_left) &&
                     finitePositiveOrZero(limits.max_acceleration_right) &&
                     finitePositiveOrZero(limits.max_deceleration_left) &&
                     finitePositiveOrZero(limits.max_deceleration_right);
  if (!valid) return result;

  const auto left = limitWheel(raw.left, applied_.left, limits.max_velocity_left,
                               limits.max_acceleration_left,
                               limits.max_deceleration_left, dt_seconds);
  const auto right = limitWheel(raw.right, applied_.right, limits.max_velocity_right,
                                limits.max_acceleration_right,
                                limits.max_deceleration_right, dt_seconds);
  result.target = {left.target, right.target};
  result.applied = {left.applied, right.applied};
  result.speed_limited_left = left.speed_limited;
  result.speed_limited_right = right.speed_limited;
  result.accel_limited_left = left.accel_limited;
  result.accel_limited_right = right.accel_limited;
  result.decel_limited_left = left.decel_limited;
  result.decel_limited_right = right.decel_limited;
  result.valid = true;
  applied_ = result.applied;
  return result;
}

void WheelCommandLimiter::reset(const WheelCommand& applied) { applied_ = applied; }

}  // namespace chassis_controller
