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
}  // namespace multi_agv_control
