#include "multi_agv_control/capability_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multi_agv_control {
namespace {

bool finiteNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double nonnegativeRatio(double numerator, double denominator) {
  if (numerator <= 0.0) {
    return 0.0;
  }
  if (denominator <= 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  return numerator / denominator;
}

}  // namespace

CapabilityMapper::CapabilityMapper(const CapabilityReserve& reserve)
    : reserve_(reserve) {
  if (!finiteNonnegative(reserve_.body_linear_velocity) ||
      !finiteNonnegative(reserve_.body_angular_velocity) ||
      !finiteNonnegative(reserve_.wheel_linear_velocity) ||
      !finiteNonnegative(reserve_.path_acceleration) ||
      !finiteNonnegative(reserve_.path_deceleration) ||
      !std::isfinite(reserve_.heading_rate_epsilon) ||
      reserve_.heading_rate_epsilon <= 0.0) {
    throw std::invalid_argument("invalid capability reserve");
  }
}

PathCapability CapabilityMapper::map(
    const WheelCapability& wheel, const CapabilityGeometry& geometry) const {
  PathCapability result;
  result.source_stamp = wheel.source_stamp;
  result.source_sequence = wheel.source_sequence;
  if (!finiteNonnegative(wheel.maximum_velocity_left) ||
      !finiteNonnegative(wheel.maximum_velocity_right) ||
      !finiteNonnegative(wheel.maximum_acceleration_left) ||
      !finiteNonnegative(wheel.maximum_acceleration_right) ||
      !finiteNonnegative(wheel.maximum_deceleration_left) ||
      !finiteNonnegative(wheel.maximum_deceleration_right) ||
      !std::isfinite(geometry.speed_scale) || geometry.speed_scale <= 0.0 ||
      !std::isfinite(geometry.heading_rate) ||
      !std::isfinite(geometry.wheel_separation) ||
      geometry.wheel_separation <= 0.0) {
    return result;
  }

  const double left_coefficient = std::abs(
      geometry.speed_scale - 0.5 * geometry.wheel_separation *
                                 geometry.heading_rate);
  const double right_coefficient = std::abs(
      geometry.speed_scale + 0.5 * geometry.wheel_separation *
                                 geometry.heading_rate);
  const double common_wheel_velocity =
      std::min(wheel.maximum_velocity_left, wheel.maximum_velocity_right);
  const double body_linear_limit = nonnegativeRatio(
      common_wheel_velocity - reserve_.body_linear_velocity,
      geometry.speed_scale);
  const double body_angular_limit = nonnegativeRatio(
      2.0 * common_wheel_velocity / geometry.wheel_separation -
          reserve_.body_angular_velocity,
      std::abs(geometry.heading_rate) + reserve_.heading_rate_epsilon);
  const double wheel_left_limit = nonnegativeRatio(
      wheel.maximum_velocity_left - reserve_.wheel_linear_velocity,
      left_coefficient);
  const double wheel_right_limit = nonnegativeRatio(
      wheel.maximum_velocity_right - reserve_.wheel_linear_velocity,
      right_coefficient);
  result.actuator_upper_velocity = std::max(
      0.0, std::min({body_linear_limit, body_angular_limit,
                     wheel_left_limit, wheel_right_limit}));
  result.lower_velocity = -result.actuator_upper_velocity;
  result.upper_velocity = result.actuator_upper_velocity;

  // Acceleration/deceleration are mapped once through the wheel Jacobian.
  // They are deliberately not deducted from the velocity bound above.
  const double mapped_acceleration = std::min(
      nonnegativeRatio(wheel.maximum_acceleration_left, left_coefficient),
      nonnegativeRatio(wheel.maximum_acceleration_right, right_coefficient));
  const double mapped_deceleration = std::min(
      nonnegativeRatio(wheel.maximum_deceleration_left, left_coefficient),
      nonnegativeRatio(wheel.maximum_deceleration_right, right_coefficient));
  result.available_acceleration =
      std::max(0.0, mapped_acceleration - reserve_.path_acceleration);
  result.available_deceleration =
      std::max(0.0, mapped_deceleration - reserve_.path_deceleration);
  result.valid = std::isfinite(result.actuator_upper_velocity) &&
                 std::isfinite(result.available_acceleration) &&
                 std::isfinite(result.available_deceleration);
  return result;
}

FleetCapability CapabilityMapper::mapFleet(
    const std::array<WheelCapability, 3>& wheels,
    const std::array<CapabilityGeometry, 3>& geometries) const {
  FleetCapability fleet;
  fleet.public_upper_velocity = std::numeric_limits<double>::infinity();
  fleet.valid = true;
  for (std::size_t index = 0U; index < fleet.robots.size(); ++index) {
    fleet.robots[index] = map(wheels[index], geometries[index]);
    fleet.valid = fleet.valid && fleet.robots[index].valid;
    fleet.public_upper_velocity = std::min(
        fleet.public_upper_velocity,
        fleet.robots[index].actuator_upper_velocity);
  }
  if (!fleet.valid) {
    fleet.public_upper_velocity = 0.0;
  }
  return fleet;
}

}  // namespace multi_agv_control
