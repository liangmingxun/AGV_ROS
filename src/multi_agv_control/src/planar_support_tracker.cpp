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
  if (geometry_.supportCount() != config_.wheel_separation.size()) {
    throw std::invalid_argument("tracker requires exactly three support paths");
  }
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
  const auto reference = geometry_.sample(
      input.support_index, input.load_progress_reference);
  result.support_pose_reference = {reference.position, reference.heading};
  const Eigen::Vector2d world_error =
      reference.position - input.support_pose_actual.position;
  const double c = std::cos(input.robot_pose_actual.yaw);
  const double s = std::sin(input.robot_pose_actual.yaw);
  result.longitudinal_error =
      c * world_error.x() + s * world_error.y();
  result.lateral_error =
      -s * world_error.x() + c * world_error.y();
  result.heading_error = normalizeAngle(
      reference.heading - input.robot_pose_actual.yaw);
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
