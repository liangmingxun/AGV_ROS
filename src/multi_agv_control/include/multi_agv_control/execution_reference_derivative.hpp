#pragma once
#include <cmath>

namespace multi_agv_control {
struct ExecutionReferenceDerivativeResult {
  bool valid{false};
  double acceleration{0.0};
};

// Differentiate the final projected/ramped execution reference, never camera
// measurements. Reset to the actual zero reference after fail-zero. Do not
// smooth velocity here: an actuator-capability constraint must not be delayed.
class ExecutionReferenceDerivative {
 public:
  void reset() noexcept { previous_velocity_ = 0.0; }
  ExecutionReferenceDerivativeResult update(double velocity, double dt) {
    ExecutionReferenceDerivativeResult result;
    if (!std::isfinite(velocity) || !std::isfinite(dt) ||
        dt <= 0.0 || dt > 0.1) {
      reset();
      return result;
    }
    result.acceleration = (velocity - previous_velocity_) / dt;
    result.valid = std::isfinite(result.acceleration);
    if (result.valid) previous_velocity_ = velocity;
    else reset();
    return result;
  }
 private:
  double previous_velocity_{0.0};
};
}  // namespace multi_agv_control
