#pragma once

#include <cmath>

namespace multi_agv_control {
// Uses measurement time, not repeated publication time. A confirmed violation
// is latched until the estimator is restarted for a new experiment.
class RigidFitRuntimeGate {
 public:
  bool accept(double residual, double stamp, double limit, double duration) {
    if (latched_ || !std::isfinite(residual) || !std::isfinite(stamp))
      return false;
    if (stamp < last_stamp_) {
      since_ = -1.0;
      last_stamp_ = stamp;
      return false;
    }
    last_stamp_ = stamp;
    if (residual <= limit) {
      since_ = -1.0;
      return true;
    }
    if (since_ < 0.0) since_ = stamp;
    latched_ = stamp - since_ >= duration;
    return !latched_;
  }
 private:
  double since_{-1.0};
  double last_stamp_{-1.0};
  bool latched_{false};
};
// Observation mode changes only the deformation gate. The independent range
// gate is immediate and latched, and non-finite fit/range inputs never pass.
class FormationRuntimeGate {
 public:
  bool accept(double residual, double stamp, double residual_limit,
              double duration, bool observation, double pair_distance,
              double range_limit) {
    if (!std::isfinite(residual) || !std::isfinite(pair_distance)) return false;
    if (observation) {
      return range_gate_.accept(pair_distance, stamp, range_limit, 0.0);
    }
    return residual_gate_.accept(residual, stamp, residual_limit, duration);
  }
 private:
  RigidFitRuntimeGate residual_gate_;
  RigidFitRuntimeGate range_gate_;
};
}  // namespace multi_agv_control
