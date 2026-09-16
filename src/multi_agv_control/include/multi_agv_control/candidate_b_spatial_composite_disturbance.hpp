#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace multi_agv_control {

struct CandidateBSpatialCompositeConfig {
  bool enabled{false};
  double minimum_effectiveness{0.85};
  double longitudinal_amplitude{0.020};
  double longitudinal_frequency{1.0};
  double longitudinal_phase{0.0};
  double yaw_amplitude{0.2625};
  double yaw_frequency{1.0};
  double yaw_phase{1.5707963267948966};
  double zone_start{2.0};
  double zone_end{2.8};
  double ramp_in_distance{0.05};
  double ramp_out_distance{0.05};
  std::array<double, 3> wheel_separation{{0.0, 0.0, 0.0}};
  std::string disturbance_model_version{
      "candidate_B_v2_spatial_composite"};
  std::string freeze_id{
      "candidate_B_v2_spatial_rho0p85_av0p020_aw0p2625"};
};

inline double candidateBSpatialQ5(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

inline void validateCandidateBSpatialCompositeConfig(
    const CandidateBSpatialCompositeConfig& c) {
  if (!c.enabled) return;
  const double pi = std::acos(-1.0);
  const bool finite_tracks = std::all_of(
      c.wheel_separation.begin(), c.wheel_separation.end(),
      [](double value) { return std::isfinite(value) && value > 0.0; });
  if (!std::isfinite(c.minimum_effectiveness) ||
      !std::isfinite(c.longitudinal_amplitude) ||
      !std::isfinite(c.longitudinal_frequency) ||
      !std::isfinite(c.longitudinal_phase) ||
      !std::isfinite(c.yaw_amplitude) ||
      !std::isfinite(c.yaw_frequency) ||
      !std::isfinite(c.yaw_phase) ||
      !std::isfinite(c.zone_start) || !std::isfinite(c.zone_end) ||
      !std::isfinite(c.ramp_in_distance) ||
      !std::isfinite(c.ramp_out_distance) || !finite_tracks ||
      std::abs(c.minimum_effectiveness - 0.85) > 1e-12 ||
      std::abs(c.longitudinal_amplitude - 0.020) > 1e-12 ||
      std::abs(c.longitudinal_frequency - 1.0) > 1e-12 ||
      std::abs(c.longitudinal_phase) > 1e-12 ||
      std::abs(c.yaw_amplitude - 0.2625) > 1e-12 ||
      std::abs(c.yaw_frequency - 1.0) > 1e-12 ||
      std::abs(c.yaw_phase - 0.5 * pi) > 1e-12 ||
      std::abs(c.zone_start - 2.0) > 1e-12 ||
      std::abs(c.zone_end - 2.8) > 1e-12 ||
      std::abs(c.ramp_in_distance - 0.05) > 1e-12 ||
      std::abs(c.ramp_out_distance - 0.05) > 1e-12 ||
      c.zone_start + c.ramp_in_distance >=
          c.zone_end - c.ramp_out_distance ||
      c.disturbance_model_version !=
          "candidate_B_v2_spatial_composite" ||
      c.freeze_id !=
          "candidate_B_v2_spatial_rho0p85_av0p020_aw0p2625") {
    throw std::invalid_argument(
        "Candidate B v2 permits only the frozen spatial composite profile");
  }
}

struct CandidateBSpatialCompositeOutput {
  int robot_id{0};
  bool triggered{false};
  bool active{false};
  bool finished{false};
  double wall_time{0.0};
  double entry_wall_time{std::numeric_limits<double>::quiet_NaN()};
  double exit_wall_time{std::numeric_limits<double>::quiet_NaN()};
  double local_elapsed{0.0};
  double spatial_progress{0.0};
  double envelope{0.0};
  double effectiveness{1.0};
  double longitudinal_disturbance{0.0};
  double yaw_disturbance{0.0};
  double left_native{0.0};
  double right_native{0.0};
  double longitudinal_native{0.0};
  double yaw_native{0.0};
  double longitudinal_after_effectiveness{0.0};
  double longitudinal_after_composite{0.0};
  double yaw_after_disturbance{0.0};
  double left_post_disturbance{0.0};
  double right_post_disturbance{0.0};
};

class CandidateBSpatialCompositeDisturbance {
 public:
  CandidateBSpatialCompositeOutput evaluate(
      const CandidateBSpatialCompositeConfig& c, int robot_id,
      double spatial_progress, double wall_time,
      double left_native, double right_native) {
    validateCandidateBSpatialCompositeConfig(c);
    if (robot_id < 1 || robot_id > 3 ||
        !std::isfinite(spatial_progress) || !std::isfinite(wall_time) ||
        !std::isfinite(left_native) || !std::isfinite(right_native) ||
        wall_time < last_wall_time_) {
      throw std::invalid_argument(
          "invalid/nonfinite/backward-clock Candidate B v2 input");
    }
    last_wall_time_ = wall_time;

    CandidateBSpatialCompositeOutput out;
    out.robot_id = robot_id;
    out.wall_time = wall_time;
    out.spatial_progress = spatial_progress;
    out.left_native = out.left_post_disturbance = left_native;
    out.right_native = out.right_post_disturbance = right_native;
    out.longitudinal_native = 0.5 * (left_native + right_native);
    const double track = c.wheel_separation[static_cast<std::size_t>(robot_id - 1)];
    out.yaw_native = (right_native - left_native) / track;
    out.longitudinal_after_effectiveness = out.longitudinal_native;
    out.longitudinal_after_composite = out.longitudinal_native;
    out.yaw_after_disturbance = out.yaw_native;
    if (!c.enabled) return out;

    if (!triggered_ && spatial_progress >= c.zone_start) {
      triggered_ = true;
      entry_wall_time_ = wall_time;
    }
    if (triggered_ && !finished_ && spatial_progress >= c.zone_end) {
      finished_ = true;
      exit_wall_time_ = wall_time;
    }
    out.triggered = triggered_;
    out.finished = finished_;
    out.entry_wall_time = triggered_
        ? entry_wall_time_ : std::numeric_limits<double>::quiet_NaN();
    out.exit_wall_time = finished_
        ? exit_wall_time_ : std::numeric_limits<double>::quiet_NaN();
    out.local_elapsed = triggered_ ? wall_time - entry_wall_time_ : 0.0;
    out.active = triggered_ && !finished_ &&
        spatial_progress > c.zone_start && spatial_progress < c.zone_end;

    if (spatial_progress > c.zone_start &&
        spatial_progress < c.zone_start + c.ramp_in_distance) {
      out.envelope = candidateBSpatialQ5(
          (spatial_progress - c.zone_start) / c.ramp_in_distance);
    } else if (spatial_progress >= c.zone_start + c.ramp_in_distance &&
               spatial_progress <= c.zone_end - c.ramp_out_distance) {
      out.envelope = 1.0;
    } else if (spatial_progress > c.zone_end - c.ramp_out_distance &&
               spatial_progress < c.zone_end) {
      out.envelope = candidateBSpatialQ5(
          (c.zone_end - spatial_progress) / c.ramp_out_distance);
    }

    out.effectiveness =
        1.0 - (1.0 - c.minimum_effectiveness) * out.envelope;
    out.longitudinal_disturbance = c.longitudinal_amplitude * out.envelope *
        std::sin(c.longitudinal_frequency * out.local_elapsed +
                 c.longitudinal_phase);
    out.yaw_disturbance = c.yaw_amplitude * out.envelope *
        std::sin(c.yaw_frequency * out.local_elapsed + c.yaw_phase);
    out.longitudinal_after_effectiveness =
        out.effectiveness * out.longitudinal_native;
    out.longitudinal_after_composite =
        out.longitudinal_after_effectiveness + out.longitudinal_disturbance;
    // rho is deliberately longitudinal-only. Native yaw is never scaled.
    out.yaw_after_disturbance = out.yaw_native + out.yaw_disturbance;
    const double half_track = 0.5 * track;
    out.left_post_disturbance = out.longitudinal_after_composite -
        half_track * out.yaw_after_disturbance;
    out.right_post_disturbance = out.longitudinal_after_composite +
        half_track * out.yaw_after_disturbance;
    return out;
  }

 private:
  bool triggered_{false};
  bool finished_{false};
  double entry_wall_time_{0.0};
  double exit_wall_time_{0.0};
  double last_wall_time_{-std::numeric_limits<double>::infinity()};
};

}  // namespace multi_agv_control
