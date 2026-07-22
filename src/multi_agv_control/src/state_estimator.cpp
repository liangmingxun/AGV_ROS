#include "multi_agv_control/state_estimator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace multi_agv_control {
namespace {

bool finiteVector(const Eigen::Vector2d& value) {
  return std::isfinite(value.x()) && std::isfinite(value.y());
}

Eigen::Vector2d rotate(const Eigen::Vector2d& value, double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * value.x() - s * value.y(),
          s * value.x() + c * value.y()};
}

}  // namespace

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

PlanarPose composePose(const PlanarPose& parent, const PlanarPose& child) {
  return {parent.position + rotate(child.position, parent.yaw),
          normalizeAngle(parent.yaw + child.yaw)};
}

RigidPoseEstimate fitRigidLoadPose(
    const std::array<Eigen::Vector2d, 3>& measured_supports,
    const std::array<SupportOffset, 3>& load_offsets) {
  RigidPoseEstimate result;
  Eigen::Vector2d measured_center = Eigen::Vector2d::Zero();
  Eigen::Vector2d offset_center = Eigen::Vector2d::Zero();
  std::array<Eigen::Vector2d, 3> offsets;
  for (std::size_t index = 0U; index < measured_supports.size(); ++index) {
    if (!finiteVector(measured_supports[index]) ||
        !std::isfinite(load_offsets[index].tangent) ||
        !std::isfinite(load_offsets[index].normal)) {
      return result;
    }
    offsets[index] = {load_offsets[index].tangent, load_offsets[index].normal};
    measured_center += measured_supports[index];
    offset_center += offsets[index];
  }
  measured_center /= 3.0;
  offset_center /= 3.0;

  double dot = 0.0;
  double cross = 0.0;
  double offset_variance = 0.0;
  for (std::size_t index = 0U; index < measured_supports.size(); ++index) {
    const Eigen::Vector2d q = offsets[index] - offset_center;
    const Eigen::Vector2d p = measured_supports[index] - measured_center;
    dot += q.dot(p);
    cross += q.x() * p.y() - q.y() * p.x();
    offset_variance += q.squaredNorm();
  }
  if (offset_variance <= 1e-12 || std::hypot(dot, cross) <= 1e-12) {
    return result;
  }

  result.pose.yaw = std::atan2(cross, dot);
  result.pose.position = measured_center - rotate(offset_center, result.pose.yaw);
  double squared_residual = 0.0;
  for (std::size_t index = 0U; index < measured_supports.size(); ++index) {
    const Eigen::Vector2d predicted =
        result.pose.position + rotate(offsets[index], result.pose.yaw);
    squared_residual += (predicted - measured_supports[index]).squaredNorm();
  }
  result.rms_residual = std::sqrt(squared_residual / 3.0);
  result.valid = std::isfinite(result.rms_residual) &&
                 finiteVector(result.pose.position) &&
                 std::isfinite(result.pose.yaw);
  return result;
}

StateEstimator::StateEstimator(PathProjector projector,
                               const StateEstimatorConfig& config)
    : projector_(std::move(projector)), config_(config) {
  if (!std::isfinite(config_.filter_alpha) || config_.filter_alpha <= 0.0 ||
      config_.filter_alpha >= 1.0 ||
      !std::isfinite(config_.projection_half_window) ||
      config_.projection_half_window <= 0.0 ||
      !std::isfinite(config_.initial_progress) || config_.initial_progress < 0.0 ||
      config_.initial_progress > projector_.pathLength() ||
      !std::isfinite(config_.minimum_measurement_interval) ||
      config_.minimum_measurement_interval <= 0.0 ||
      !std::isfinite(config_.maximum_measurement_interval) ||
      config_.maximum_measurement_interval <= config_.minimum_measurement_interval ||
      !std::isfinite(config_.maximum_absolute_speed) ||
      config_.maximum_absolute_speed <= 0.0) {
    throw std::invalid_argument("invalid state estimator configuration");
  }
  reset(config_.initial_progress);
}

StateEstimate StateEstimator::invalidEstimate(
    double measurement_stamp, const ProjectionResult& projection) const {
  StateEstimate result;
  result.measurement_stamp = measurement_stamp;
  result.projection = projection;
  return result;
}

StateEstimate StateEstimator::update(const Eigen::Vector2d& measured_position,
                                     double measurement_stamp) {
  if (!std::isfinite(measurement_stamp)) {
    return invalidEstimate(measurement_stamp, {});
  }
  const double seed = initialized_ ? last_estimate_.progress
                                   : config_.initial_progress;
  const auto projection = projector_.project(
      measured_position, seed, config_.projection_half_window);
  if (!projection.valid) {
    return invalidEstimate(measurement_stamp, projection);
  }

  if (!initialized_) {
    last_estimate_.valid = true;
    last_estimate_.speed_initialized = false;
    last_estimate_.measurement_stamp = measurement_stamp;
    last_estimate_.progress = projection.s;
    last_estimate_.speed = 0.0;
    last_estimate_.projection = projection;
    initialized_ = true;
    return last_estimate_;
  }

  const double dt = measurement_stamp - last_estimate_.measurement_stamp;
  if (!std::isfinite(dt) || dt < config_.minimum_measurement_interval) {
    return invalidEstimate(measurement_stamp, projection);
  }
  const double raw_speed = (projection.s - last_estimate_.progress) / dt;
  if (dt > config_.maximum_measurement_interval ||
      !std::isfinite(raw_speed) ||
      std::abs(raw_speed) > config_.maximum_absolute_speed) {
    last_estimate_.valid = true;
    last_estimate_.speed_initialized = false;
    last_estimate_.measurement_stamp = measurement_stamp;
    last_estimate_.progress = projection.s;
    last_estimate_.speed = 0.0;
    last_estimate_.projection = projection;
    return invalidEstimate(measurement_stamp, projection);
  }

  const double previous_speed = last_estimate_.speed_initialized
                                    ? last_estimate_.speed : 0.0;
  last_estimate_.valid = true;
  last_estimate_.speed_initialized = true;
  last_estimate_.measurement_stamp = measurement_stamp;
  last_estimate_.progress = projection.s;
  last_estimate_.speed = config_.filter_alpha * previous_speed +
                         (1.0 - config_.filter_alpha) * raw_speed;
  last_estimate_.projection = projection;
  return last_estimate_;
}

void StateEstimator::reset(double initial_progress) {
  if (!std::isfinite(initial_progress) || initial_progress < 0.0 ||
      initial_progress > projector_.pathLength()) {
    throw std::invalid_argument("initial progress lies outside the path");
  }
  config_.initial_progress = initial_progress;
  initialized_ = false;
  last_estimate_ = StateEstimate{};
  last_estimate_.progress = initial_progress;
}

}  // namespace multi_agv_control
