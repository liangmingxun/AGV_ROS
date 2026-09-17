#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_agv_control {

struct M2cNativeGovernorOutput {
  double left_unbounded{0.0};
  double right_unbounded{0.0};
  double left_governed{0.0};
  double right_governed{0.0};
  double scale{1.0};
  bool active{false};
};

inline M2cNativeGovernorOutput applyM2cNativeGovernor(
    double left, double right, double limit) {
  if (!std::isfinite(left) || !std::isfinite(right) ||
      !std::isfinite(limit) || limit <= 0.0) {
    throw std::invalid_argument("invalid M2c native-governor input");
  }
  M2cNativeGovernorOutput output;
  output.left_unbounded = left;
  output.right_unbounded = right;
  const double peak = std::max(std::abs(left), std::abs(right));
  output.scale = peak > limit ? limit / peak : 1.0;
  output.active = output.scale < 1.0;
  output.left_governed = left * output.scale;
  output.right_governed = right * output.scale;
  return output;
}

}  // namespace multi_agv_control
