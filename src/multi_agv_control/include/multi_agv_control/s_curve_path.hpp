#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace multi_agv_control {

struct SCurveConfig {
  double amplitude{0.0};
  double longitudinal_length{1.0};
  std::size_t lookup_samples{10001};
  std::string model{"sine_single_period"};
  double circle_radius{1.0};
  double entry_straight_length{0.0};
  double curvature_ramp_length{0.0};
  double circle_direction{1.0};
  // Optional zero-curvature continuation after a graph path. It follows the
  // graph's terminal tangent, so position and heading remain continuous.
  double exit_straight_length{0.0};
};

struct PathSample {
  double s{0.0};
  double xi{0.0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d first_derivative{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d second_derivative{Eigen::Vector2d::Zero()};
  Eigen::Vector2d tangent{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d normal{Eigen::Vector2d::UnitY()};
  double heading{0.0};
  double curvature{0.0};
  double curvature_derivative{0.0};
};

class SCurvePath {
 public:
  explicit SCurvePath(const SCurveConfig& config);

  const SCurveConfig& config() const noexcept { return config_; }
  double length() const noexcept {
    return arc_length_.back() + config_.exit_straight_length;
  }
  double maximumAbsoluteCurvature() const noexcept {
    return maximum_absolute_curvature_;
  }

  PathSample sample(double s) const;
  double xiForArcLength(double s) const;
  double arcLengthForXi(double xi) const;

 private:
  double rawSlope(double xi) const noexcept;
  double rawSecondDerivative(double xi) const noexcept;
  double rawThirdDerivative(double xi) const noexcept;
  double rawSpeed(double xi) const noexcept;

  SCurveConfig config_;
  double wave_number_{0.0};
  double maximum_absolute_curvature_{0.0};
  std::vector<double> xi_;
  std::vector<double> arc_length_;
  std::vector<Eigen::Vector2d> path_position_;
};

}  // namespace multi_agv_control
