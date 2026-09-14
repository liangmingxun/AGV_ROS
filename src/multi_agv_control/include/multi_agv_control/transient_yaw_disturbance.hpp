#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multi_agv_control {

struct TransientYawConfig {
  bool enabled{false};
  bool effect_exploration{false};
  double trigger_progress{2.0};
  double amplitude{0.024};
  double duration{1.5};
};

inline void validateTransientYawConfig(const TransientYawConfig& config) {
  if (!config.enabled) return;
  const bool candidate_a = std::abs(config.amplitude - .024) < 1e-12 &&
                           std::abs(config.duration - 1.5) < 1e-12;
  const bool candidate_b = std::abs(config.amplitude - .022) < 1e-12 &&
                           std::abs(config.duration - 2.0) < 1e-12;
  const bool exploration_candidate = config.effect_exploration && (
      (std::abs(config.amplitude - .022) < 1e-12 &&
       (std::abs(config.duration - 2.5) < 1e-12 || std::abs(config.duration - 3.0) < 1e-12)) ||
      ((std::abs(config.amplitude - .024) < 1e-12 || std::abs(config.amplitude - .026) < 1e-12) &&
       std::abs(config.duration - 2.5) < 1e-12));
  if (!std::isfinite(config.amplitude) || !std::isfinite(config.duration) ||
      !std::isfinite(config.trigger_progress) ||
      std::abs(config.trigger_progress - 2.0) > 1e-12 ||
      !(candidate_a || candidate_b || exploration_candidate)) {
    throw std::invalid_argument("Exp2c-v4 permits only A=.024/T=1.5 or A=.022/T=2.0 at s2=2.0");
  }
}

struct TransientYawOutput {
  bool triggered{false};
  bool active{false};
  bool finished{false};
  double wall_time{0.0};
  double trigger_wall_time{std::numeric_limits<double>::quiet_NaN()};
  double elapsed{0.0};
  double progress{0.0};
  double envelope{0.0};
  double disturbance{0.0};
  double left_raw{0.0};
  double right_raw{0.0};
  double left_disturbed{0.0};
  double right_disturbed{0.0};
};

// One-shot, wall-clock pulse. No capability/risk input, no progress-based
// termination, no retrigger after recovery or fail-zero.
class TransientYawPulse {
 public:
  TransientYawOutput evaluate(const TransientYawConfig& config, int robot_id,
                             double progress, double wall_time,
                             double left_raw, double right_raw) {
    validateTransientYawConfig(config);
    TransientYawOutput out;
    out.progress = progress;
    out.wall_time = wall_time;
    out.left_raw = out.left_disturbed = left_raw;
    out.right_raw = out.right_disturbed = right_raw;
    if (!config.enabled) return out;
    if (!std::isfinite(progress) || !std::isfinite(wall_time) ||
        !std::isfinite(left_raw) || !std::isfinite(right_raw) ||
        robot_id < 1 || robot_id > 3 || wall_time < last_wall_time_) {
      throw std::invalid_argument("invalid/nonfinite/backward-clock transient yaw input");
    }
    last_wall_time_ = wall_time;
    if (robot_id != 2) return out;
    if (!triggered_ && progress >= config.trigger_progress) {
      triggered_ = true;
      trigger_wall_time_ = wall_time;
    }
    if (!triggered_) return out;
    out.triggered = true;
    out.trigger_wall_time = trigger_wall_time_;
    out.elapsed = wall_time - trigger_wall_time_;
    out.finished = out.elapsed >= config.duration;
    out.active = !out.finished;
    if (out.elapsed > 0.0 && !out.finished) {
      const double u = std::clamp(2.0 * std::min(
          out.elapsed, config.duration - out.elapsed) / config.duration, 0.0, 1.0);
      out.envelope = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      out.disturbance = config.amplitude * out.envelope;
      out.left_disturbed -= out.disturbance;
      out.right_disturbed += out.disturbance;
    }
    return out;
  }

 private:
  bool triggered_{false};
  double trigger_wall_time_{0.0};
  double last_wall_time_{-std::numeric_limits<double>::infinity()};
};

}  // namespace multi_agv_control
