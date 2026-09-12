#pragma once
#include <algorithm>
#include <cmath>
namespace multi_agv_control {
inline double recoveryWheelSlew(double target, double previous, double dt) {
  if (!(dt > 0.0 && dt <= 0.1) || !std::isfinite(target) || !std::isfinite(previous)) return target;
  const double step=0.30*dt;
  return std::clamp(target,previous-step,previous+step);
}
}  // namespace multi_agv_control
