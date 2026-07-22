#include "multi_agv_control/support_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace multi_agv_control {
namespace {

double cross(const Eigen::Vector2d& first,
             const Eigen::Vector2d& second) noexcept {
  return first.x() * second.y() - first.y() * second.x();
}

bool finiteOffset(const SupportOffset& offset) noexcept {
  return std::isfinite(offset.tangent) && std::isfinite(offset.normal);
}

}  // namespace

SupportGeometry::SupportGeometry(const SCurvePath& path,
                                 const SupportGeometryConfig& config)
    : path_(path), config_(config) {
  if (config_.offsets.size() != 3) {
    throw std::invalid_argument("support geometry requires exactly three offsets");
  }
  if (!std::isfinite(config_.minimum_nondegeneracy) ||
      config_.minimum_nondegeneracy <= 0.0 ||
      !std::isfinite(config_.minimum_speed_scale) ||
      config_.minimum_speed_scale <= 0.0 || config_.validation_samples < 2) {
    throw std::invalid_argument("invalid support geometry validation limits");
  }
  if (!std::all_of(config_.offsets.begin(), config_.offsets.end(),
                   finiteOffset)) {
    throw std::invalid_argument("support offsets must be finite");
  }
  validateAndMeasure();
}

SupportSample SupportGeometry::sample(std::size_t support_index, double s) const {
  if (support_index >= config_.offsets.size()) {
    throw std::out_of_range("support index is out of range");
  }
  if (!std::isfinite(s)) {
    throw std::invalid_argument("support arc length must be finite");
  }
  return uncheckedSample(support_index, s);
}

SupportSample SupportGeometry::uncheckedSample(std::size_t support_index,
                                               double s) const {
  const auto center = path_.sample(s);
  const auto offset = config_.offsets[support_index];
  const double kappa = center.curvature;
  const double kappa_prime = center.curvature_derivative;
  const double tangent_coefficient = 1.0 - kappa * offset.normal;
  const double normal_coefficient = kappa * offset.tangent;

  SupportSample value;
  value.support_index = support_index;
  value.s = center.s;
  value.offset = offset;
  value.centerline_curvature = kappa;
  value.nondegeneracy_factor = tangent_coefficient;
  value.position = center.position + offset.tangent * center.tangent +
      offset.normal * center.normal;
  value.first_derivative = tangent_coefficient * center.tangent +
      normal_coefficient * center.normal;

  const double second_tangent_coefficient =
      -kappa_prime * offset.normal - kappa * kappa * offset.tangent;
  const double second_normal_coefficient =
      kappa * tangent_coefficient + kappa_prime * offset.tangent;
  value.second_derivative =
      second_tangent_coefficient * center.tangent +
      second_normal_coefficient * center.normal;

  value.speed_scale = value.first_derivative.norm();
  if (!(value.speed_scale > 0.0) || !std::isfinite(value.speed_scale)) {
    throw std::invalid_argument("support path derivative is degenerate");
  }
  value.tangent = value.first_derivative / value.speed_scale;
  value.normal = {-value.tangent.y(), value.tangent.x()};
  value.heading = std::atan2(value.tangent.y(), value.tangent.x());
  const double signed_cross =
      cross(value.first_derivative, value.second_derivative);
  value.heading_rate = signed_cross /
      (value.speed_scale * value.speed_scale);
  value.curvature = value.heading_rate / value.speed_scale;
  return value;
}

void SupportGeometry::validateAndMeasure() {
  minimum_nondegeneracy_ = std::numeric_limits<double>::infinity();
  minimum_speed_scale_ = std::numeric_limits<double>::infinity();
  max_curvature_normal_offset_product_ = 0.0;

  // The sine path curvature continuously spans [-K, K]. Both chi(kappa) and
  // g(kappa)^2 are at most quadratic in kappa, so their global minima can be
  // checked analytically rather than trusting a configurable sampling grid.
  const double max_curvature = path_.maximumAbsoluteCurvature();
  for (std::size_t robot = 0; robot < config_.offsets.size(); ++robot) {
    const auto offset = config_.offsets[robot];
    const double exact_chi =
        1.0 - max_curvature * std::abs(offset.normal);
    const double offset_squared =
        offset.tangent * offset.tangent + offset.normal * offset.normal;
    double minimizing_curvature = 0.0;
    if (offset_squared > 0.0) {
      minimizing_curvature = std::clamp(
          offset.normal / offset_squared, -max_curvature, max_curvature);
    }
    const double tangent_coefficient =
        1.0 - minimizing_curvature * offset.normal;
    const double normal_coefficient =
        minimizing_curvature * offset.tangent;
    const double exact_speed_scale =
        std::hypot(tangent_coefficient, normal_coefficient);

    minimum_nondegeneracy_ =
        std::min(minimum_nondegeneracy_, exact_chi);
    minimum_speed_scale_ =
        std::min(minimum_speed_scale_, exact_speed_scale);
    max_curvature_normal_offset_product_ = std::max(
        max_curvature_normal_offset_product_,
        max_curvature * std::abs(offset.normal));

    if (exact_chi < config_.minimum_nondegeneracy ||
        exact_speed_scale < config_.minimum_speed_scale) {
      std::ostringstream message;
      message << "support geometry globally degenerates for robot " << robot
              << ": chi_min=" << exact_chi
              << ", g_min=" << exact_speed_scale;
      throw std::invalid_argument(message.str());
    }
  }

  for (std::size_t index = 0; index < config_.validation_samples; ++index) {
    const double s = path_.length() * static_cast<double>(index) /
        static_cast<double>(config_.validation_samples - 1);
    for (std::size_t robot = 0; robot < config_.offsets.size(); ++robot) {
      const auto value = uncheckedSample(robot, s);
      minimum_nondegeneracy_ =
          std::min(minimum_nondegeneracy_, value.nondegeneracy_factor);
      minimum_speed_scale_ =
          std::min(minimum_speed_scale_, value.speed_scale);
      if (value.nondegeneracy_factor < config_.minimum_nondegeneracy ||
          value.speed_scale < config_.minimum_speed_scale) {
        std::ostringstream message;
        message << "support geometry degenerates for robot " << robot
                << " at s=" << s << ": chi="
                << value.nondegeneracy_factor << ", g=" << value.speed_scale;
        throw std::invalid_argument(message.str());
      }
    }
  }
}

}  // namespace multi_agv_control
