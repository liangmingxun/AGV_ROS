#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace multi_agv_control {

struct CandidateBConfig {
  bool enabled{false};
  int target_robot_id{2};
  double minimum_effectiveness{0.85};
  double yaw_amplitude{0.2625};
  double yaw_frequency{1.0};
  double yaw_phase{1.5707963267948966};
  double trigger_progress{2.0};
  double duration{8.0};
  double ramp_in{0.5};
  double ramp_out{0.5};
  double wheel_separation{0.139284482};
  std::string disturbance_model_version{"candidate_B_v1"};
  std::string freeze_id{"candidate_B_v1_rho0p85"};
};

inline double candidateBQuinticSmoothstep(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

inline void validateCandidateBConfig(const CandidateBConfig& c) {
  if (!c.enabled) return;
  const double pi = std::acos(-1.0);
  if (c.target_robot_id != 2 ||
      !std::isfinite(c.minimum_effectiveness) ||
      !std::isfinite(c.yaw_amplitude) ||
      !std::isfinite(c.yaw_frequency) ||
      !std::isfinite(c.yaw_phase) ||
      !std::isfinite(c.trigger_progress) || !std::isfinite(c.duration) ||
      !std::isfinite(c.ramp_in) || !std::isfinite(c.ramp_out) ||
      !std::isfinite(c.wheel_separation) ||
      std::abs(c.minimum_effectiveness - 0.85) > 1e-12 ||
      std::abs(c.yaw_amplitude - 0.2625) > 1e-12 ||
      std::abs(c.yaw_frequency - 1.0) > 1e-12 ||
      std::abs(c.yaw_phase - 0.5 * pi) > 1e-12 ||
      std::abs(c.trigger_progress - 2.0) > 1e-12 ||
      std::abs(c.duration - 8.0) > 1e-12 ||
      std::abs(c.ramp_in - 0.5) > 1e-12 ||
      std::abs(c.ramp_out - 0.5) > 1e-12 ||
      c.wheel_separation <= 0.0 ||
      c.disturbance_model_version != "candidate_B_v1" ||
      c.freeze_id != "candidate_B_v1_rho0p85") {
    throw std::invalid_argument(
        "Candidate B permits only the frozen rho=0.85 fake profile");
  }
}

struct CandidateBOutput {
  bool triggered{false};
  bool active{false};
  bool finished{false};
  double wall_time{0.0};
  double trigger_wall_time{std::numeric_limits<double>::quiet_NaN()};
  double elapsed{0.0};
  double progress{0.0};
  double envelope{0.0};
  double effectiveness{1.0};
  double yaw_disturbance{0.0};
  double left_native{0.0};
  double right_native{0.0};
  double longitudinal_native{0.0};
  double yaw_native{0.0};
  double longitudinal_after_effectiveness{0.0};
  double yaw_after_disturbance{0.0};
  double left_post_disturbance{0.0};
  double right_post_disturbance{0.0};
};

class CandidateBEffectivenessDisturbance {
 public:
  CandidateBOutput evaluate(const CandidateBConfig& c, int robot_id,
                            double progress, double wall_time,
                            double left_native, double right_native) {
    validateCandidateBConfig(c);
    CandidateBOutput out;
    out.progress = progress;
    out.wall_time = wall_time;
    out.left_native = out.left_post_disturbance = left_native;
    out.right_native = out.right_post_disturbance = right_native;
    out.longitudinal_native = 0.5 * (left_native + right_native);
    out.yaw_native = (right_native - left_native) / c.wheel_separation;
    out.longitudinal_after_effectiveness = out.longitudinal_native;
    out.yaw_after_disturbance = out.yaw_native;
    if (!c.enabled) return out;
    if (!std::isfinite(progress) || !std::isfinite(wall_time) ||
        !std::isfinite(left_native) || !std::isfinite(right_native) ||
        robot_id < 1 || robot_id > 3 || wall_time < last_wall_time_) {
      throw std::invalid_argument(
          "invalid/nonfinite/backward-clock Candidate B input");
    }
    last_wall_time_ = wall_time;
    if (robot_id != c.target_robot_id) return out;
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
        out.envelope = candidateBQuinticSmoothstep(out.elapsed / c.ramp_in);
      } else if (out.elapsed >= c.duration - c.ramp_out) {
        out.envelope = candidateBQuinticSmoothstep(
            (c.duration - out.elapsed) / c.ramp_out);
      } else {
        out.envelope = 1.0;
      }
      out.effectiveness =
          1.0 - (1.0 - c.minimum_effectiveness) * out.envelope;
      out.yaw_disturbance = out.envelope * c.yaw_amplitude *
          std::sin(c.yaw_frequency * out.elapsed + c.yaw_phase);
      // Deliberately scale only the longitudinal component.  The native yaw
      // component is preserved and the bounded yaw disturbance is then added.
      out.longitudinal_after_effectiveness =
          out.effectiveness * out.longitudinal_native;
      out.yaw_after_disturbance = out.yaw_native + out.yaw_disturbance;
      const double half_track = 0.5 * c.wheel_separation;
      out.left_post_disturbance = out.longitudinal_after_effectiveness -
          half_track * out.yaw_after_disturbance;
      out.right_post_disturbance = out.longitudinal_after_effectiveness +
          half_track * out.yaw_after_disturbance;
    }
    return out;
  }

 private:
  bool triggered_{false};
  double trigger_wall_time_{0.0};
  double last_wall_time_{-std::numeric_limits<double>::infinity()};
};

}  // namespace multi_agv_control
