#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_agv_control {

// Optional, method-independent reconciliation between the persistent R1
// channel command and the linear command that the chassis reports as applied
// after its own speed/acceleration limits. It is deliberately much slower than
// the chassis loop:
// the R1 acceleration integrator remains authoritative and this term only
// removes a sustained execution mismatch.  No differentiated camera velocity
// is fed back into the command state.
struct ExecutionChannelReconcilerConfig {
  bool enabled{false};
  bool require_robot2_derating{true};
  double gain_per_second{0.05};
  double mismatch_deadband{0.002};
  double maximum_correction_acceleration{0.001};
  double minimum_channel_speed_scale{0.20};
};

struct ExecutionChannelReconciliation {
  bool valid{false};
  bool active{false};
  bool correction_limited{false};
  double equivalent_channel_velocity{0.0};
  double mismatch{0.0};
  double correction_acceleration{0.0};
  double next_channel_velocity{0.0};
};

class ExecutionChannelReconciler {
 public:
  explicit ExecutionChannelReconciler(
      const ExecutionChannelReconcilerConfig& config)
      : config_(config) {
    if (!std::isfinite(config_.gain_per_second) ||
        config_.gain_per_second < 0.0 ||
        !std::isfinite(config_.mismatch_deadband) ||
        config_.mismatch_deadband < 0.0 ||
        !std::isfinite(config_.maximum_correction_acceleration) ||
        config_.maximum_correction_acceleration < 0.0 ||
        !std::isfinite(config_.minimum_channel_speed_scale) ||
        config_.minimum_channel_speed_scale <= 0.0) {
      throw std::invalid_argument(
          "invalid execution channel reconciliation configuration");
    }
  }

  ExecutionChannelReconciliation step(
      double channel_velocity, double applied_linear_velocity,
      double feedforward_linear_velocity, double heading_error,
      double channel_speed_scale, double dt_seconds,
      double velocity_lower_bound, double velocity_upper_bound,
      bool robot2_derating_active) const {
    ExecutionChannelReconciliation result;
    result.next_channel_velocity = channel_velocity;
    if (!std::isfinite(channel_velocity) ||
        !std::isfinite(applied_linear_velocity) ||
        !std::isfinite(feedforward_linear_velocity) ||
        !std::isfinite(heading_error) ||
        !std::isfinite(channel_speed_scale) ||
        !std::isfinite(dt_seconds) ||
        !std::isfinite(velocity_lower_bound) ||
        !std::isfinite(velocity_upper_bound) ||
        dt_seconds <= 0.0 || dt_seconds > 0.1 ||
        velocity_lower_bound > velocity_upper_bound) {
      return result;
    }
    result.valid = true;
    if (!config_.enabled ||
        (config_.require_robot2_derating && !robot2_derating_active) ||
        std::abs(channel_speed_scale) < config_.minimum_channel_speed_scale) {
      return result;
    }

    const double nominal_linear_velocity =
        feedforward_linear_velocity * std::cos(heading_error);
    result.equivalent_channel_velocity = channel_velocity +
        (applied_linear_velocity - nominal_linear_velocity) /
            channel_speed_scale;
    if (!std::isfinite(result.equivalent_channel_velocity)) {
      result.valid = false;
      return result;
    }
    result.mismatch = result.equivalent_channel_velocity - channel_velocity;
    const double magnitude = std::abs(result.mismatch);
    if (magnitude <= config_.mismatch_deadband) {
      return result;
    }
    const double mismatch_outside_deadband = std::copysign(
        magnitude - config_.mismatch_deadband, result.mismatch);
    const double correction_unlimited =
        config_.gain_per_second * mismatch_outside_deadband;
    result.correction_acceleration = std::clamp(
        correction_unlimited, -config_.maximum_correction_acceleration,
        config_.maximum_correction_acceleration);
    result.correction_limited =
        result.correction_acceleration != correction_unlimited;
    result.next_channel_velocity = std::clamp(
        channel_velocity + dt_seconds * result.correction_acceleration,
        velocity_lower_bound, velocity_upper_bound);
    result.active = result.next_channel_velocity != channel_velocity;
    result.valid = std::isfinite(result.next_channel_velocity);
    return result;
  }

  const ExecutionChannelReconcilerConfig& config() const noexcept {
    return config_;
  }

 private:
  ExecutionChannelReconcilerConfig config_;
};

}  // namespace multi_agv_control
