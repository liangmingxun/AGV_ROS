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
  const bool circle = config_.model == "circle";
  const bool smooth_circle = config_.model == "circle_smooth_entry";
  const bool graph_path = config_.model == "sine_single_period" ||
                          config_.model == "straight_zero_amplitude" ||
                          config_.model ==
                              "sine_single_period_with_straight_exit";
  if (!std::isfinite(config_.amplitude) ||
      !std::isfinite(config_.longitudinal_length) ||
      config_.longitudinal_length <= 0.0 ||
      !std::isfinite(config_.exit_straight_length) ||
      config_.exit_straight_length < 0.0 ||
      ((!graph_path) && config_.exit_straight_length != 0.0) ||
      (config_.model != "sine_single_period_with_straight_exit" &&
       config_.exit_straight_length != 0.0) ||
      (config_.model == "sine_single_period_with_straight_exit" &&
       config_.exit_straight_length <= 0.0) ||
      (!circle && !smooth_circle && !graph_path) ||
      (circle && (!std::isfinite(config_.circle_radius) ||
                  config_.circle_radius <= 0.0 ||
                  std::abs(config_.longitudinal_length -
                           2.0 * kPi * config_.circle_radius) > 1e-6)) ||
      (smooth_circle &&
       (!std::isfinite(config_.circle_radius) ||
        config_.circle_radius <= 0.0 ||
        !std::isfinite(config_.entry_straight_length) ||
        config_.entry_straight_length < 0.0 ||
        !std::isfinite(config_.curvature_ramp_length) ||
        config_.curvature_ramp_length <= 0.0 ||
        !std::isfinite(config_.circle_direction) ||
        std::abs(std::abs(config_.circle_direction) - 1.0) > 1e-12 ||
        std::abs(config_.longitudinal_length -
                 (config_.entry_straight_length +
                  config_.curvature_ramp_length +
                  2.0 * kPi * config_.circle_radius)) > 1e-6)) ||
      config_.lookup_samples < kMinimumLookupSamples) {
    throw std::invalid_argument("invalid planar-path configuration");
  }

  wave_number_ = 2.0 * kPi / config_.longitudinal_length;
  maximum_absolute_curvature_ = (circle || smooth_circle)
      ? 1.0 / config_.circle_radius
      // For y=A*sin(k*xi), |curvature| is maximal where cos(k*xi)=0.
      : std::abs(config_.amplitude) * wave_number_ * wave_number_;
  xi_.resize(config_.lookup_samples);
  arc_length_.resize(config_.lookup_samples);
  if (smooth_circle) {
    path_position_.resize(config_.lookup_samples,
                          Eigen::Vector2d::Zero());
  }
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
    if (smooth_circle) {
      const auto heading_at = [this](double s) {
        const double straight = config_.entry_straight_length;
        const double ramp = config_.curvature_ramp_length;
        const double signed_curvature =
            config_.circle_direction / config_.circle_radius;
        if (s <= straight) return 0.0;
        if (s < straight + ramp) {
          const double t = (s - straight) / ramp;
          // Integral of kappa=k*(3*t^2-2*t^3): C2 curvature entry.
          return signed_curvature * ramp *
              (t * t * t - 0.5 * t * t * t * t);
        }
        return 0.5 * signed_curvature * ramp +
            signed_curvature * (s - straight - ramp);
      };
      const double lower_heading = heading_at(lower);
      const double midpoint_heading = heading_at(midpoint);
      const double upper_heading = heading_at(upper);
      path_position_[index] = path_position_[index - 1] +
          (upper - lower) / 6.0 *
          (Eigen::Vector2d(std::cos(lower_heading), std::sin(lower_heading)) +
           4.0 * Eigen::Vector2d(std::cos(midpoint_heading),
                                 std::sin(midpoint_heading)) +
           Eigen::Vector2d(std::cos(upper_heading), std::sin(upper_heading)));
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
  if (config_.model == "circle" ||
      config_.model == "circle_smooth_entry") return 1.0;
  return std::hypot(1.0, rawSlope(xi));
}

double SCurvePath::xiForArcLength(double s) const {
  const double bounded = clampFinite(s, 0.0, length(), "arc length");
  if (bounded <= 0.0) return 0.0;
  if (bounded >= arc_length_.back()) return config_.longitudinal_length;

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
  // xi parameterizes only the graph segment; the optional terminal straight
  // has no additional raw graph coordinate.
  if (bounded >= config_.longitudinal_length) return arc_length_.back();

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

  if (config_.exit_straight_length > 0.0 &&
      value.s > arc_length_.back()) {
    const double terminal_slope = rawSlope(config_.longitudinal_length);
    const double terminal_speed = std::hypot(1.0, terminal_slope);
    value.tangent = {1.0 / terminal_speed,
                     terminal_slope / terminal_speed};
    value.normal = {-value.tangent.y(), value.tangent.x()};
    value.heading = std::atan2(value.tangent.y(), value.tangent.x());
    value.curvature = 0.0;
    value.curvature_derivative = 0.0;
    value.first_derivative = value.tangent;
    value.second_derivative = Eigen::Vector2d::Zero();
    const Eigen::Vector2d terminal_position(
        config_.longitudinal_length,
        config_.amplitude * std::sin(
            wave_number_ * config_.longitudinal_length));
    value.position = terminal_position +
        (value.s - arc_length_.back()) * value.tangent;
    return value;
  }

  if (config_.model == "circle") {
    const double angle = value.s / config_.circle_radius;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    value.position = {config_.circle_radius * sine,
                      config_.circle_radius * (1.0 - cosine)};
    value.tangent = {cosine, sine};
    value.normal = {-sine, cosine};
    value.heading = std::atan2(sine, cosine);
    value.curvature = 1.0 / config_.circle_radius;
    value.curvature_derivative = 0.0;
    value.first_derivative = value.tangent;
    value.second_derivative = value.curvature * value.normal;
    return value;
  }

  if (config_.model == "circle_smooth_entry") {
    const double straight = config_.entry_straight_length;
    const double ramp = config_.curvature_ramp_length;
    const double signed_curvature =
        config_.circle_direction / config_.circle_radius;
    if (value.s <= straight) {
      value.heading = 0.0;
      value.curvature = 0.0;
      value.curvature_derivative = 0.0;
    } else if (value.s < straight + ramp) {
      const double t = (value.s - straight) / ramp;
      value.heading = signed_curvature * ramp *
          (t * t * t - 0.5 * t * t * t * t);
      value.curvature = signed_curvature *
          (3.0 * t * t - 2.0 * t * t * t);
      value.curvature_derivative = signed_curvature *
          (6.0 * t - 6.0 * t * t) / ramp;
    } else {
      value.heading = 0.5 * signed_curvature * ramp +
          signed_curvature * (value.s - straight - ramp);
      value.curvature = signed_curvature;
      value.curvature_derivative = 0.0;
    }
    value.tangent = {std::cos(value.heading), std::sin(value.heading)};
    value.normal = {-value.tangent.y(), value.tangent.x()};
    value.first_derivative = value.tangent;
    value.second_derivative = value.curvature * value.normal;

    const auto upper = std::upper_bound(
        arc_length_.begin(), arc_length_.end(), value.s);
    if (upper == arc_length_.begin()) {
      value.position = path_position_.front();
    } else if (upper == arc_length_.end()) {
      value.position = path_position_.back();
    } else {
      const std::size_t upper_index = static_cast<std::size_t>(
          std::distance(arc_length_.begin(), upper));
      const std::size_t lower_index = upper_index - 1;
      const double ratio = (value.s - arc_length_[lower_index]) /
          (arc_length_[upper_index] - arc_length_[lower_index]);
      value.position = path_position_[lower_index] + ratio *
          (path_position_[upper_index] - path_position_[lower_index]);
    }
    return value;
  }

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
