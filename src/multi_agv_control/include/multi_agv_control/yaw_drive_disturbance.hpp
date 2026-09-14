#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_agv_control {

// Exp2c-v2 fake-only execution disturbance. It sees neither controller risk
// state nor capability state and is applied to a copy of the tracker output.
struct YawDriveDisturbanceConfig {
  bool enabled{false};
  double start_s{2.0};
  double end_s{2.8};
  double transition_length{0.15};
  double amplitude{0.004};
};

struct YawDriveDisturbanceOutput {
  bool active{false};
  double path_progress{0.0};
  double window{0.0};
  double phase{0.0};
  double amplitude{0.0};
  double signed_disturbance{0.0};
  double left_nominal{0.0};
  double right_nominal{0.0};
  double left_disturbed{0.0};
  double right_disturbed{0.0};
  double mean_longitudinal_delta{0.0};
  double yaw_differential_delta{0.0};
};

inline YawDriveDisturbanceOutput evaluateYawDriveDisturbance(
    const YawDriveDisturbanceConfig& config, int robot_id,
    double actual_path_progress, double left_nominal, double right_nominal) {
  YawDriveDisturbanceOutput output;
  output.path_progress = actual_path_progress;
  output.amplitude = config.amplitude;
  output.left_nominal = left_nominal;
  output.right_nominal = right_nominal;
  output.left_disturbed = left_nominal;
  output.right_disturbed = right_nominal;
  if (!config.enabled) return output;
  if (!std::isfinite(config.start_s) || !std::isfinite(config.end_s) ||
      !std::isfinite(config.transition_length) ||
      !std::isfinite(config.amplitude) ||
      !std::isfinite(actual_path_progress) || !std::isfinite(left_nominal) ||
      !std::isfinite(right_nominal) || config.start_s < 0.0 ||
      config.end_s <= config.start_s || config.transition_length <= 0.0 ||
      2.0 * config.transition_length >= config.end_s - config.start_s ||
      config.amplitude <= 0.0 || config.amplitude > 0.024 ||
      robot_id < 1 || robot_id > 3) {
    throw std::invalid_argument("invalid Exp2c-v2 yaw disturbance input");
  }
  if (robot_id != 2 || actual_path_progress <= config.start_s ||
      actual_path_progress >= config.end_s) {
    return output;
  }
  const auto smoothstep5 = [](double value) {
    value = std::clamp(value, 0.0, 1.0);
    return value * value * value *
        (10.0 + value * (-15.0 + 6.0 * value));
  };
  const double length = config.end_s - config.start_s;
  const double xi = (actual_path_progress - config.start_s) / length;
  output.window =
      smoothstep5((actual_path_progress - config.start_s) /
                  config.transition_length) *
      smoothstep5((config.end_s - actual_path_progress) /
                  config.transition_length);
  output.phase = 2.0 * 3.14159265358979323846 * xi;
  output.signed_disturbance =
      config.amplitude * output.window * std::sin(output.phase);
  output.left_disturbed = left_nominal - output.signed_disturbance;
  output.right_disturbed = right_nominal + output.signed_disturbance;
  output.mean_longitudinal_delta = 0.5 * (
      output.left_disturbed - left_nominal +
      output.right_disturbed - right_nominal);
  output.yaw_differential_delta =
      (output.right_disturbed - right_nominal) -
      (output.left_disturbed - left_nominal);
  output.active = std::abs(output.signed_disturbance) > 0.0;
  return output;
}

}  // namespace multi_agv_control
