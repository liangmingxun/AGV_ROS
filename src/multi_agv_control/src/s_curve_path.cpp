#include "multi_agv_control/s_curve_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace multi_agv_control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMinimumLookupSamples = 101;

double clampFinite(double value, double lower, double upper,
                   const char* quantity) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(quantity) + " must be finite");
  }
  return std::clamp(value, lower, upper);
}

}  // namespace

SCurvePath::SCurvePath(const SCurveConfig& config) : config_(config) {
  if (!std::isfinite(config_.amplitude) ||
      !std::isfinite(config_.longitudinal_length) ||
      config_.longitudinal_length <= 0.0 ||
      config_.lookup_samples < kMinimumLookupSamples) {
    throw std::invalid_argument("invalid S-curve configuration");
  }

  wave_number_ = 2.0 * kPi / config_.longitudinal_length;
  // For y=A*sin(k*xi), |curvature| is maximal where cos(k*xi)=0.
  maximum_absolute_curvature_ =
      std::abs(config_.amplitude) * wave_number_ * wave_number_;
  xi_.resize(config_.lookup_samples);
  arc_length_.resize(config_.lookup_samples);
  xi_.front() = 0.0;
  arc_length_.front() = 0.0;

  const double step = config_.longitudinal_length /
      static_cast<double>(config_.lookup_samples - 1);
  for (std::size_t index = 1; index < config_.lookup_samples; ++index) {
    const double lower = step * static_cast<double>(index - 1);
    const double upper = step * static_cast<double>(index);
    const double midpoint = 0.5 * (lower + upper);
    xi_[index] = upper;
    const double segment_length =
        (upper - lower) *
        (rawSpeed(lower) + 4.0 * rawSpeed(midpoint) + rawSpeed(upper)) / 6.0;
    arc_length_[index] = arc_length_[index - 1] + segment_length;
    if (!std::isfinite(arc_length_[index]) ||
        arc_length_[index] <= arc_length_[index - 1]) {
      throw std::invalid_argument("S-curve arc-length table is not monotonic");
    }
  }
}

double SCurvePath::rawSlope(double xi) const noexcept {
  return config_.amplitude * wave_number_ * std::cos(wave_number_ * xi);
}

double SCurvePath::rawSecondDerivative(double xi) const noexcept {
  return -config_.amplitude * wave_number_ * wave_number_ *
      std::sin(wave_number_ * xi);
}

double SCurvePath::rawThirdDerivative(double xi) const noexcept {
  return -config_.amplitude * wave_number_ * wave_number_ * wave_number_ *
      std::cos(wave_number_ * xi);
}

double SCurvePath::rawSpeed(double xi) const noexcept {
  return std::hypot(1.0, rawSlope(xi));
}

double SCurvePath::xiForArcLength(double s) const {
  const double bounded = clampFinite(s, 0.0, length(), "arc length");
  if (bounded <= 0.0) return 0.0;
  if (bounded >= length()) return config_.longitudinal_length;

  const auto upper = std::upper_bound(arc_length_.begin(), arc_length_.end(),
                                      bounded);
  const std::size_t upper_index =
      static_cast<std::size_t>(std::distance(arc_length_.begin(), upper));
  const std::size_t lower_index = upper_index - 1;
  const double ratio = (bounded - arc_length_[lower_index]) /
      (arc_length_[upper_index] - arc_length_[lower_index]);
  return xi_[lower_index] + ratio * (xi_[upper_index] - xi_[lower_index]);
}

double SCurvePath::arcLengthForXi(double xi) const {
  const double bounded = clampFinite(
      xi, 0.0, config_.longitudinal_length, "raw path coordinate");
  if (bounded <= 0.0) return 0.0;
  if (bounded >= config_.longitudinal_length) return length();

  const auto upper = std::upper_bound(xi_.begin(), xi_.end(), bounded);
  const std::size_t upper_index =
      static_cast<std::size_t>(std::distance(xi_.begin(), upper));
  const std::size_t lower_index = upper_index - 1;
  const double ratio = (bounded - xi_[lower_index]) /
      (xi_[upper_index] - xi_[lower_index]);
  return arc_length_[lower_index] +
      ratio * (arc_length_[upper_index] - arc_length_[lower_index]);
}

PathSample SCurvePath::sample(double s) const {
  PathSample value;
  value.s = clampFinite(s, 0.0, length(), "arc length");
  value.xi = xiForArcLength(value.s);

  const double phase = wave_number_ * value.xi;
  const double slope = rawSlope(value.xi);
  const double second = rawSecondDerivative(value.xi);
  const double third = rawThirdDerivative(value.xi);
  const double speed = std::hypot(1.0, slope);
  const double speed_squared = speed * speed;

  value.position = {value.xi, config_.amplitude * std::sin(phase)};
  value.tangent = {1.0 / speed, slope / speed};
  value.normal = {-value.tangent.y(), value.tangent.x()};
  value.heading = std::atan2(value.tangent.y(), value.tangent.x());
  value.curvature = second / (speed_squared * speed);
  const double curvature_xi = third / (speed_squared * speed) -
      3.0 * slope * second * second /
          (speed_squared * speed_squared * speed);
  value.curvature_derivative = curvature_xi / speed;
  value.first_derivative = value.tangent;
  value.second_derivative = value.curvature * value.normal;
  return value;
}

}  // namespace multi_agv_control
