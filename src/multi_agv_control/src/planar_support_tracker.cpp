#include "multi_agv_control/planar_support_tracker.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace multi_agv_control {
namespace {

bool finitePose(const PlanarPose& pose) {
  return std::isfinite(pose.position.x()) &&
         std::isfinite(pose.position.y()) && std::isfinite(pose.yaw);
}

}  // namespace

PlanarSupportTracker::PlanarSupportTracker(
    SupportGeometry geometry, const PlanarTrackerConfig& config)
    : geometry_(std::move(geometry)), config_(config) {
  if (!std::isfinite(config_.longitudinal_gain) ||
      config_.longitudinal_gain < 0.0 ||
      !std::isfinite(config_.lateral_gain) || config_.lateral_gain < 0.0 ||
      !std::isfinite(config_.heading_gain) || config_.heading_gain < 0.0) {
    throw std::invalid_argument("tracker gains must be finite and nonnegative");
  }
  for (double separation : config_.wheel_separation) {
    if (!std::isfinite(separation) || separation <= 0.0) {
      throw std::invalid_argument("wheel separation must be finite and positive");
    }
  }
  for (const auto& offset : config_.base_to_support) {
    if (!std::isfinite(offset.x()) || !std::isfinite(offset.y())) {
      throw std::invalid_argument(
          "base-to-support offsets must be finite");
    }
  }
  if (config_.chassis_reference_samples < 2U) {
    throw std::invalid_argument(
        "chassis reference requires at least two samples");
  }
  if (geometry_.supportCount() != config_.wheel_separation.size()) {
    throw std::invalid_argument("tracker requires exactly three support paths");
  }
  buildChassisReference();
}

double PlanarSupportTracker::chassisHeadingDerivative(
    std::size_t support_index, double load_progress,
    double chassis_heading) const {
  const auto longitudinal_offset =
      config_.base_to_support[support_index].x();
  const auto support = geometry_.sample(support_index, load_progress);
  if (std::abs(longitudinal_offset) <= 1e-12) {
    return support.heading_rate;
  }
  const Eigen::Vector2d chassis_normal{
      -std::sin(chassis_heading), std::cos(chassis_heading)};
  return chassis_normal.dot(support.first_derivative) /
         longitudinal_offset;
}

void PlanarSupportTracker::buildChassisReference() {
  const double path_length = geometry_.path().length();
  chassis_reference_step_ =
      path_length /
      static_cast<double>(config_.chassis_reference_samples - 1U);
  for (std::size_t support_index = 0U;
       support_index < chassis_heading_.size(); ++support_index) {
    auto& heading = chassis_heading_[support_index];
    heading.assign(config_.chassis_reference_samples, 0.0);
    const double longitudinal_offset =
        config_.base_to_support[support_index].x();
    if (std::abs(longitudinal_offset) <= 1e-12) {
      for (std::size_t index = 0U; index < heading.size(); ++index) {
        heading[index] =
            geometry_.sample(support_index,
                             static_cast<double>(index) *
                                 chassis_reference_step_).heading;
      }
    } else {
      const bool integrate_forward = longitudinal_offset > 0.0;
      const std::size_t boundary =
          integrate_forward ? 0U : heading.size() - 1U;
      const double boundary_s =
          static_cast<double>(boundary) * chassis_reference_step_;
      heading[boundary] =
          geometry_.sample(support_index, boundary_s).heading;
      const auto integrate = [&](double s, double theta, double step) {
        const double k1 =
            chassisHeadingDerivative(support_index, s, theta);
        const double k2 = chassisHeadingDerivative(
            support_index, s + 0.5 * step, theta + 0.5 * step * k1);
        const double k3 = chassisHeadingDerivative(
            support_index, s + 0.5 * step, theta + 0.5 * step * k2);
        const double k4 = chassisHeadingDerivative(
            support_index, s + step, theta + step * k3);
        return theta +
               step * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
      };
      if (integrate_forward) {
        for (std::size_t index = 0U; index + 1U < heading.size(); ++index) {
          const double s =
              static_cast<double>(index) * chassis_reference_step_;
          heading[index + 1U] =
              integrate(s, heading[index], chassis_reference_step_);
        }
      } else {
        for (std::size_t index = heading.size() - 1U; index > 0U; --index) {
          const double s =
              static_cast<double>(index) * chassis_reference_step_;
          heading[index - 1U] =
              integrate(s, heading[index], -chassis_reference_step_);
        }
      }
    }
    for (std::size_t index = 0U; index < heading.size(); ++index) {
      const double s =
          static_cast<double>(index) * chassis_reference_step_;
      const auto support = geometry_.sample(support_index, s);
      const double heading_rate =
          chassisHeadingDerivative(support_index, s, heading[index]);
      const Eigen::Vector2d tangent{
          std::cos(heading[index]), std::sin(heading[index])};
      const double speed_scale =
          tangent.dot(support.first_derivative) +
          config_.base_to_support[support_index].y() * heading_rate;
      if (!std::isfinite(heading[index]) ||
          !std::isfinite(heading_rate) ||
          !std::isfinite(speed_scale) || speed_scale <= 0.0) {
        throw std::invalid_argument(
            "base-to-support offset produces invalid chassis reference");
      }
    }
  }
}

PlanarSupportTracker::ChassisReference
PlanarSupportTracker::chassisReference(
    std::size_t support_index, double load_progress) const {
  const double path_length = geometry_.path().length();
  const double progress = std::max(0.0, std::min(path_length, load_progress));
  const auto offset = config_.base_to_support[support_index];
  if (offset.squaredNorm() <= 1e-24) {
    const auto support = geometry_.sample(support_index, progress);
    ChassisReference result;
    result.support_pose = {support.position, support.heading};
    result.chassis_pose = result.support_pose;
    result.speed_scale = support.speed_scale;
    result.heading_rate = support.heading_rate;
    return result;
  }
  const double table_position = progress / chassis_reference_step_;
  const std::size_t left = std::min(
      static_cast<std::size_t>(table_position),
      config_.chassis_reference_samples - 2U);
  const std::size_t right = left + 1U;
  const double ratio =
      std::max(0.0, std::min(1.0, table_position -
                                     static_cast<double>(left)));
  const double heading_left = chassis_heading_[support_index][left];
  const double heading_delta = normalizeAngle(
      chassis_heading_[support_index][right] - heading_left);
  const double chassis_heading = heading_left + ratio * heading_delta;
  const auto support = geometry_.sample(support_index, progress);
  const double c = std::cos(chassis_heading);
  const double s = std::sin(chassis_heading);
  const Eigen::Vector2d offset_world{
      c * offset.x() - s * offset.y(),
      s * offset.x() + c * offset.y()};
  const double heading_rate = chassisHeadingDerivative(
      support_index, progress, chassis_heading);
  const Eigen::Vector2d chassis_tangent{c, s};

  ChassisReference result;
  result.support_pose = {support.position, support.heading};
  result.chassis_pose.position = support.position - offset_world;
  result.chassis_pose.yaw = normalizeAngle(chassis_heading);
  result.speed_scale =
      chassis_tangent.dot(support.first_derivative) +
      offset.y() * heading_rate;
  result.heading_rate = heading_rate;
  return result;
}

PlanarTrackingResult PlanarSupportTracker::track(
    const PlanarTrackingInput& input) const {
  PlanarTrackingResult result;
  if (input.support_index >= geometry_.supportCount() ||
      !std::isfinite(input.load_progress_reference) ||
      !std::isfinite(input.channel_velocity_command) ||
      !finitePose(input.robot_pose_actual) ||
      !finitePose(input.support_pose_actual)) {
    return result;
  }
  const auto reference =
      chassisReference(input.support_index, input.load_progress_reference);
  result.support_pose_reference = reference.support_pose;
  result.chassis_pose_reference = reference.chassis_pose;
  const Eigen::Vector2d world_error =
      reference.chassis_pose.position - input.robot_pose_actual.position;
  const double c = std::cos(input.robot_pose_actual.yaw);
  const double s = std::sin(input.robot_pose_actual.yaw);
  result.longitudinal_error =
      c * world_error.x() + s * world_error.y();
  result.lateral_error =
      -s * world_error.x() + c * world_error.y();
  result.heading_error = normalizeAngle(
      reference.chassis_pose.yaw - input.robot_pose_actual.yaw);
  result.linear_velocity_feedforward =
      reference.speed_scale * input.channel_velocity_command;
  result.angular_velocity_feedforward =
      reference.heading_rate * input.channel_velocity_command;
  result.linear_velocity_raw =
      result.linear_velocity_feedforward * std::cos(result.heading_error) +
      config_.longitudinal_gain * result.longitudinal_error;
  result.angular_velocity_raw =
      result.angular_velocity_feedforward +
      config_.lateral_gain * result.lateral_error +
      config_.heading_gain * std::sin(result.heading_error);
  const double half_track =
      0.5 * config_.wheel_separation[input.support_index];
  result.wheel_linear_velocity_left_raw =
      result.linear_velocity_raw - half_track * result.angular_velocity_raw;
  result.wheel_linear_velocity_right_raw =
      result.linear_velocity_raw + half_track * result.angular_velocity_raw;
  result.valid = std::isfinite(result.wheel_linear_velocity_left_raw) &&
                 std::isfinite(result.wheel_linear_velocity_right_raw);
  return result;
}

std::array<PlanarTrackingResult, 3> PlanarSupportTracker::trackFleet(
    double common_load_progress, double common_channel_velocity,
    const std::array<PlanarPose, 3>& robot_poses,
    const std::array<PlanarPose, 3>& support_poses) const {
  std::array<PlanarTrackingResult, 3> results;
  for (std::size_t index = 0U; index < results.size(); ++index) {
    results[index] = track({index, common_load_progress,
                            common_channel_velocity, robot_poses[index],
                            support_poses[index]});
  }
  return results;
}

}  // namespace multi_agv_control
