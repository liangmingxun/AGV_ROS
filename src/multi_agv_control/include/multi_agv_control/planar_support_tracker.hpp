#pragma once

#include <array>
#include <cstddef>

#include "multi_agv_control/state_estimator.hpp"
#include "multi_agv_control/support_geometry.hpp"

namespace multi_agv_control {

struct PlanarTrackerConfig {
  double longitudinal_gain{1.0};
  double lateral_gain{2.0};
  double heading_gain{2.0};
  std::array<double, 3> wheel_separation{{0.114, 0.114, 0.114}};
};

struct PlanarTrackingInput {
  std::size_t support_index{0U};
  double load_progress_reference{0.0};
  double channel_velocity_command{0.0};
  PlanarPose robot_pose_actual;
  PlanarPose support_pose_actual;
};

struct PlanarTrackingResult {
  bool valid{false};
  PlanarPose support_pose_reference;
  double longitudinal_error{0.0};
  double lateral_error{0.0};
  double heading_error{0.0};
  double linear_velocity_feedforward{0.0};
  double angular_velocity_feedforward{0.0};
  double linear_velocity_raw{0.0};
  double angular_velocity_raw{0.0};
  double wheel_linear_velocity_left_raw{0.0};
  double wheel_linear_velocity_right_raw{0.0};
};

class PlanarSupportTracker {
 public:
  PlanarSupportTracker(SupportGeometry geometry,
                       const PlanarTrackerConfig& config);

  PlanarTrackingResult track(const PlanarTrackingInput& input) const;
  std::array<PlanarTrackingResult, 3> trackFleet(
      double common_load_progress, double common_channel_velocity,
      const std::array<PlanarPose, 3>& robot_poses,
      const std::array<PlanarPose, 3>& support_poses) const;

  const SupportGeometry& geometry() const noexcept { return geometry_; }

 private:
  SupportGeometry geometry_;
  PlanarTrackerConfig config_;
};

}  // namespace multi_agv_control
