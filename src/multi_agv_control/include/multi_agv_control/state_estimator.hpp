#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include "multi_agv_control/path_projector.hpp"
#include "multi_agv_control/support_geometry.hpp"

namespace multi_agv_control {

struct PlanarPose {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  double yaw{0.0};
};

PlanarPose composePose(const PlanarPose& parent, const PlanarPose& child);
double normalizeAngle(double angle);

struct RigidPoseEstimate {
  bool valid{false};
  PlanarPose pose;
  double rms_residual{0.0};
};

RigidPoseEstimate fitRigidLoadPose(
    const std::array<Eigen::Vector2d, 3>& measured_supports,
    const std::array<SupportOffset, 3>& load_offsets);

struct StateEstimatorConfig {
  double filter_alpha{0.8};
  double projection_half_window{0.35};
  double initial_progress{0.0};
  double minimum_measurement_interval{1e-5};
  double maximum_measurement_interval{0.2};
  double maximum_absolute_speed{0.5};
};

struct StateEstimate {
  bool valid{false};
  bool speed_initialized{false};
  double measurement_stamp{0.0};
  double progress{0.0};
  double speed{0.0};
  ProjectionResult projection;
};

class StateEstimator {
 public:
  StateEstimator(PathProjector projector, const StateEstimatorConfig& config);

  StateEstimate update(const Eigen::Vector2d& measured_position,
                       double measurement_stamp);
  void reset(double initial_progress = 0.0);

  bool initialized() const noexcept { return initialized_; }
  const StateEstimate& lastEstimate() const noexcept { return last_estimate_; }

 private:
  StateEstimate invalidEstimate(double measurement_stamp,
                                const ProjectionResult& projection) const;

  PathProjector projector_;
  StateEstimatorConfig config_;
  bool initialized_{false};
  StateEstimate last_estimate_;
};

}  // namespace multi_agv_control
