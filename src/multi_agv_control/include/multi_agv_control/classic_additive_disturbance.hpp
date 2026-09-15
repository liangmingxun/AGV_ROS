#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multi_agv_control {

struct ClassicAdditiveConfig {
  bool enabled{false};
  double scale{1.0};
  double trigger_progress{2.0};
  double linear_amplitude{0.030};
  double angular_amplitude{0.350};
  double linear_frequency{1.0};
  double angular_frequency{1.0};
  double angular_phase{1.5707963267948966};
  double duration{8.0};
  double ramp_in{0.5};
  double ramp_out{0.5};
  double wheel_separation{0.139284482};
};

inline bool isClassicAdditiveQualificationScale(double scale) {
  constexpr double tolerance = 1e-12;
  return std::isfinite(scale) &&
      (std::abs(scale - 0.0) <= tolerance ||
       std::abs(scale - 0.25) <= tolerance ||
       std::abs(scale - 0.50) <= tolerance ||
       std::abs(scale - 0.75) <= tolerance ||
       std::abs(scale - 1.00) <= tolerance);
}

inline bool isClassicAdditiveMaximumQualifiedScale(double scale) {
  constexpr double tolerance = 1e-12;
  return std::isfinite(scale) &&
      (std::abs(scale - 0.25) <= tolerance ||
       std::abs(scale - 0.50) <= tolerance ||
       std::abs(scale - 0.75) <= tolerance ||
       std::abs(scale - 1.00) <= tolerance);
}

inline void validateClassicAdditivePhysicalQualification(
    double requested_scale, double maximum_qualified_scale) {
  if (!isClassicAdditiveMaximumQualifiedScale(maximum_qualified_scale)) {
    throw std::invalid_argument(
        "classic additive physical maximum qualified scale is invalid");
  }
  if (!isClassicAdditiveQualificationScale(requested_scale)) {
    throw std::invalid_argument(
        "classic additive physical requested scale is invalid");
  }
  // Scale zero is the disturbance-disabled baseline and does not consume a
  // qualification level.  Every active physical request is fail-closed at
  // the highest level explicitly qualified in configuration.
  if (requested_scale > 0.0 &&
      requested_scale > maximum_qualified_scale + 1e-12) {
    throw std::invalid_argument(
        "classic additive physical scale exceeds maximum qualified scale");
  }
}

inline void validateClassicAdditiveConfig(const ClassicAdditiveConfig& c) {
  if (!c.enabled) return;
  const double pi = std::acos(-1.0);
  if (!std::isfinite(c.trigger_progress) ||
      !std::isfinite(c.linear_amplitude) ||
      !std::isfinite(c.angular_amplitude) ||
      !std::isfinite(c.linear_frequency) ||
      !std::isfinite(c.angular_frequency) ||
      !std::isfinite(c.angular_phase) || !std::isfinite(c.duration) ||
      !std::isfinite(c.ramp_in) || !std::isfinite(c.ramp_out) ||
      !std::isfinite(c.wheel_separation) ||
      !isClassicAdditiveQualificationScale(c.scale) ||
      std::abs(c.trigger_progress - 2.0) > 1e-12 ||
      std::abs(c.linear_amplitude - 0.030) > 1e-12 ||
      std::abs(c.angular_amplitude - 0.350) > 1e-12 ||
      std::abs(c.linear_frequency - 1.0) > 1e-12 ||
      std::abs(c.angular_frequency - 1.0) > 1e-12 ||
      std::abs(c.angular_phase - 0.5 * pi) > 1e-12 ||
      std::abs(c.duration - 8.0) > 1e-12 ||
      std::abs(c.ramp_in - 0.5) > 1e-12 ||
      std::abs(c.ramp_out - 0.5) > 1e-12 ||
      std::abs(c.wheel_separation - 0.139284482) > 1e-12) {
    throw std::invalid_argument(
        "classic additive validation permits only frozen Candidate A");
  }
}

struct ClassicAdditiveOutput {
  bool triggered{false};
  bool active{false};
  bool finished{false};
  double wall_time{0.0};
  double trigger_wall_time{std::numeric_limits<double>::quiet_NaN()};
  double elapsed{0.0};
  double progress{0.0};
  double envelope{0.0};
  double linear_disturbance{0.0};
  double angular_disturbance{0.0};
  double left_raw{0.0};
  double right_raw{0.0};
  double left_disturbed{0.0};
  double right_disturbed{0.0};
};

inline double classicQuinticSmoothstep(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

class ClassicAdditiveDisturbance {
 public:
  ClassicAdditiveOutput evaluate(const ClassicAdditiveConfig& c, int robot_id,
                                  double progress, double wall_time,
                                  double left_raw, double right_raw) {
    validateClassicAdditiveConfig(c);
    ClassicAdditiveOutput out;
    out.progress = progress;
    out.wall_time = wall_time;
    out.left_raw = out.left_disturbed = left_raw;
    out.right_raw = out.right_disturbed = right_raw;
    if (!c.enabled) return out;
    if (!std::isfinite(progress) || !std::isfinite(wall_time) ||
        !std::isfinite(left_raw) || !std::isfinite(right_raw) ||
        robot_id < 1 || robot_id > 3 || wall_time < last_wall_time_) {
      throw std::invalid_argument(
          "invalid/nonfinite/backward-clock classic additive input");
    }
    last_wall_time_ = wall_time;
    if (robot_id != 2) return out;
    if (!triggered_ && progress >= c.trigger_progress) {
      triggered_ = true;
      trigger_wall_time_ = wall_time;
    }
    if (!triggered_) return out;
    out.triggered = true;
    out.trigger_wall_time = trigger_wall_time_;
    out.elapsed = wall_time - trigger_wall_time_;
    out.finished = out.elapsed >= c.duration;
    out.active = !out.finished;
    if (out.elapsed >= 0.0 && !out.finished) {
      if (out.elapsed < c.ramp_in) {
        out.envelope = classicQuinticSmoothstep(out.elapsed / c.ramp_in);
      } else if (out.elapsed >= c.duration - c.ramp_out) {
        out.envelope = classicQuinticSmoothstep(
            (c.duration - out.elapsed) / c.ramp_out);
      } else {
        out.envelope = 1.0;
      }
      out.linear_disturbance = c.scale * out.envelope * c.linear_amplitude *
                               std::sin(c.linear_frequency * out.elapsed);
      out.angular_disturbance = c.scale * out.envelope * c.angular_amplitude *
          std::sin(c.angular_frequency * out.elapsed + c.angular_phase);
      const double half_track = 0.5 * c.wheel_separation;
      out.left_disturbed +=
          out.linear_disturbance - half_track * out.angular_disturbance;
      out.right_disturbed +=
          out.linear_disturbance + half_track * out.angular_disturbance;
    }
    return out;
  }

 private:
  bool triggered_{false};
  double trigger_wall_time_{0.0};
  double last_wall_time_{-std::numeric_limits<double>::infinity()};
};

}  // namespace multi_agv_control
