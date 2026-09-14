#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_agv_control {

// Execution-only acceleration perturbation; never a controller compensation
// input, wheel limit, capability report or simulated measurement.
struct RiskDisturbanceConfig {
  bool enabled{false};
  double start_s{2.0};
  double end_s{2.8};
  double transition_length{0.15};
  double nominal_speed{0.10};
  double frequency_hz{0.30};
  double peak_fraction{0.30};
};

struct RiskDisturbanceOutput {
  double window{0.0};
  double base{0.0};
  double velocity{0.0};
  double sinusoid{0.0};
  double acceleration{0.0};
};

inline RiskDisturbanceOutput evaluateRiskDisturbance(
    const RiskDisturbanceConfig& cfg, int robot_id, double path_s,
    double actual_velocity, double elapsed_seconds, double available_accel) {
  RiskDisturbanceOutput out;
  if (!cfg.enabled) return out;  // Disabled path is exactly zero.
  if (!std::isfinite(cfg.start_s) || !std::isfinite(cfg.end_s) ||
      !std::isfinite(cfg.transition_length) ||
      !std::isfinite(cfg.nominal_speed) || !std::isfinite(cfg.frequency_hz) ||
      !std::isfinite(cfg.peak_fraction) || cfg.start_s < 0.0 ||
      cfg.transition_length <= 0.0 ||
      cfg.end_s - cfg.start_s < 2.0 * cfg.transition_length ||
      cfg.nominal_speed <= 0.0 || cfg.frequency_hz < 0.0 ||
      cfg.peak_fraction <= 0.0 || cfg.peak_fraction > 0.70 ||
      robot_id < 1 || robot_id > 3 || !std::isfinite(path_s) ||
      !std::isfinite(actual_velocity) || !std::isfinite(elapsed_seconds) ||
      !std::isfinite(available_accel) || available_accel < 0.0 ||
      elapsed_seconds < 0.0) {
    throw std::invalid_argument("invalid risk disturbance input");
  }
  if (robot_id != 2 || path_s <= cfg.start_s || path_s >= cfg.end_s) return out;
  const auto smooth = [](double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x*x*x*(10.0 + x*(-15.0 + 6.0*x));
  };
  out.window = smooth((path_s - cfg.start_s) / cfg.transition_length) *
      smooth((cfg.end_s - path_s) / cfg.transition_length);
  // Fixed relative composition, normalised to an explicit maximum fraction.
  // Saturating |v|/v_nom preserves the bound even during overshoot.
  const double scale = -out.window * available_accel * cfg.peak_fraction;
  out.base = scale * 0.60;
  out.velocity = scale * 0.25 *
      std::clamp(std::abs(actual_velocity) / cfg.nominal_speed, 0.0, 1.0);
  out.sinusoid = scale * 0.15 *
      std::sin(2.0 * 3.14159265358979323846 * cfg.frequency_hz * elapsed_seconds);
  out.acceleration = out.base + out.velocity + out.sinusoid;
  return out;
}

}  // namespace multi_agv_control
