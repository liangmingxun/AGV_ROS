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

inline double backCalculateM2dPersistentCommand(
    double persistent_command, const M2cNativeGovernorOutput& governor,
    double channel_speed_scale, double heading_error,
    double lower_bound = -0.15, double upper_bound = 0.52) {
  if (!std::isfinite(persistent_command) ||
      !std::isfinite(channel_speed_scale) ||
      !std::isfinite(heading_error) ||
      !std::isfinite(lower_bound) || !std::isfinite(upper_bound) ||
      lower_bound > upper_bound) {
    throw std::invalid_argument("invalid M2d back-calculation input");
  }
  if (!governor.active) return persistent_command;
  const double unbounded_center =
      0.5 * (governor.left_unbounded + governor.right_unbounded);
  const double governed_center =
      0.5 * (governor.left_governed + governor.right_governed);
  const double channel_to_linear =
      channel_speed_scale * std::cos(heading_error);
  double projected = persistent_command * governor.scale;
  if (std::abs(channel_to_linear) > 1e-6) {
    projected = persistent_command +
        (governed_center - unbounded_center) / channel_to_linear;
  }
  if (!std::isfinite(projected)) {
    throw std::invalid_argument("nonfinite M2d back-calculation output");
  }
  return std::clamp(projected, lower_bound, upper_bound);
}

}  // namespace multi_agv_control
