#pragma once

#include <cstddef>
#include <functional>

#include <Eigen/Core>

namespace multi_agv_control {

struct ProjectionCurveSample {
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d first_derivative{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d second_derivative{Eigen::Vector2d::Zero()};
};

struct PathProjectorConfig {
  std::size_t coarse_samples{81};
  std::size_t maximum_refinement_iterations{48};
  double convergence_tolerance{1e-9};
  double maximum_projection_distance{0.25};
};

struct ProjectionResult {
  bool valid{false};
  bool converged{false};
  bool hit_path_boundary{false};
  bool hit_window_boundary{false};
  double s{0.0};
  double distance{0.0};
  std::size_t refinement_iterations{0};
};

class PathProjector {
 public:
  using SampleFunction = std::function<ProjectionCurveSample(double)>;

  PathProjector(double path_length, SampleFunction sample,
                const PathProjectorConfig& config = {});

  double pathLength() const noexcept { return path_length_; }
  const PathProjectorConfig& config() const noexcept { return config_; }

  ProjectionResult project(const Eigen::Vector2d& point, double previous_s,
                           double half_window) const;

 private:
  double squaredDistance(const Eigen::Vector2d& point, double s) const;

  double path_length_{0.0};
  SampleFunction sample_;
  PathProjectorConfig config_;
};

}  // namespace multi_agv_control
