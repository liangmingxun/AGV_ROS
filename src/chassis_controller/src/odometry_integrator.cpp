#include "chassis_controller/odometry_integrator.hpp"

#include <cmath>
#include <stdexcept>

namespace chassis_controller {

OdometryIntegrator::OdometryIntegrator(
    double stationary_wheel_velocity_tolerance)
    : stationary_wheel_velocity_tolerance_(
          stationary_wheel_velocity_tolerance) {
  if (!std::isfinite(stationary_wheel_velocity_tolerance_) ||
      stationary_wheel_velocity_tolerance_ < 0.0) {
    throw std::invalid_argument(
        "invalid stationary wheel velocity tolerance");
  }
}

bool OdometryIntegrator::update(double actual_left, double actual_right,
                                double imu_yaw_rate, double dt_seconds) {
  if (!std::isfinite(actual_left) || !std::isfinite(actual_right) ||
      !std::isfinite(imu_yaw_rate) || !std::isfinite(dt_seconds) ||
      dt_seconds <= 0.0) {
    return false;
  }
  linear_velocity_ = 0.5 * (actual_left + actual_right);
  const bool wheels_stationary =
      std::abs(actual_left) <= stationary_wheel_velocity_tolerance_ &&
      std::abs(actual_right) <= stationary_wheel_velocity_tolerance_;
  // A stationary differential chassis cannot change yaw by itself. Suppress
  // residual gyro bias while both measured drive-wheel speeds are zero; an
  // in-place turn still has non-zero and opposite wheel speeds.
  yaw_rate_ = wheels_stationary ? 0.0 : imu_yaw_rate;
  const double midpoint_yaw = state_.yaw + 0.5 * yaw_rate_ * dt_seconds;
  state_.x += linear_velocity_ * dt_seconds * std::cos(midpoint_yaw);
  state_.y += linear_velocity_ * dt_seconds * std::sin(midpoint_yaw);
  state_.yaw = normalizeYaw(state_.yaw + yaw_rate_ * dt_seconds);
  return true;
}

void OdometryIntegrator::reset(const Pose2DState& pose) {
  state_ = pose;
  state_.yaw = normalizeYaw(state_.yaw);
  linear_velocity_ = 0.0;
  yaw_rate_ = 0.0;
}

double OdometryIntegrator::normalizeYaw(double yaw) {
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2.0 * pi;
  yaw = std::fmod(yaw + pi, two_pi);
  if (yaw < 0.0) yaw += two_pi;
  return yaw - pi;
}

}  // namespace chassis_controller
