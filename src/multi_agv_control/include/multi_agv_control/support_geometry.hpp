#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "multi_agv_control/s_curve_path.hpp"

namespace multi_agv_control {

struct SupportOffset {
  double tangent{0.0};
  double normal{0.0};
};

struct SupportGeometryConfig {
  std::vector<SupportOffset> offsets;
  double minimum_nondegeneracy{0.2};
  double minimum_speed_scale{0.2};
  std::size_t validation_samples{10001};
};

struct SupportSample {
  // Zero-based index into SupportGeometryConfig::offsets. This is deliberately
  // not named robot_index because ROS robot_index values are one-based (1..3).
  std::size_t support_index{0};
  double s{0.0};
  SupportOffset offset;
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d first_derivative{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d second_derivative{Eigen::Vector2d::Zero()};
  Eigen::Vector2d tangent{Eigen::Vector2d::UnitX()};
  Eigen::Vector2d normal{Eigen::Vector2d::UnitY()};
  double heading{0.0};
  double centerline_curvature{0.0};
  double nondegeneracy_factor{1.0};
  double speed_scale{1.0};
  double heading_rate{0.0};
  double curvature{0.0};
};

class SupportGeometry {
 public:
  SupportGeometry(const SCurvePath& path,
                  const SupportGeometryConfig& config);

  const SCurvePath& path() const noexcept { return path_; }
  const SupportGeometryConfig& config() const noexcept { return config_; }
  std::size_t supportCount() const noexcept { return config_.offsets.size(); }

  SupportSample sample(std::size_t support_index, double s) const;

  double minimumNondegeneracy() const noexcept {
    return minimum_nondegeneracy_;
  }
  double minimumSpeedScale() const noexcept { return minimum_speed_scale_; }
  double maxCurvatureNormalOffsetProduct() const noexcept {
    return max_curvature_normal_offset_product_;
  }

 private:
  SupportSample uncheckedSample(std::size_t support_index, double s) const;
  void validateAndMeasure();

  SCurvePath path_;
  SupportGeometryConfig config_;
  double minimum_nondegeneracy_{1.0};
  double minimum_speed_scale_{1.0};
  double max_curvature_normal_offset_product_{0.0};
};

}  // namespace multi_agv_control
