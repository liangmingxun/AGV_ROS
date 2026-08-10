#include "multi_agv_control/lower_channel_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_agv_control {
namespace {

bool finite(double value) {
  return std::isfinite(value);
}

double smoothSign(double value, double layer) {
  return std::tanh(value / layer);
}

}  // namespace

LowerMode lowerModeFromString(const std::string& value) {
  if (value == "R1") return LowerMode::kR1;
  if (value == "R2") return LowerMode::kR2;
  if (value == "R3") return LowerMode::kR3;
  if (value == "R4") return LowerMode::kR4;
  if (value == "M2b") return LowerMode::kM2b;
  throw std::invalid_argument("lower mode must be R1, R2, R3, R4 or M2b");
}

const char* lowerModeName(LowerMode mode) {
  switch (mode) {
    case LowerMode::kR1: return "R1";
    case LowerMode::kR2: return "R2";
    case LowerMode::kR3: return "R3";
    case LowerMode::kR4: return "R4";
    case LowerMode::kM2b: return "M2b";
  }
  return "UNKNOWN";
}

bool lowerAdaptationEnabled(LowerMode mode) {
  return mode == LowerMode::kR1 || mode == LowerMode::kR2;
}

bool lowerDisturbanceCompensationEnabled(LowerMode mode) {
  return mode == LowerMode::kR1 || mode == LowerMode::kR3;
}

LowerChannelController::LowerChannelController(
    const LowerChannelConfig& config)
    : config_(config) {
  if (!finite(config_.composite_gain) || config_.composite_gain <= 0.0 ||
      !finite(config_.feedback_gain) || config_.feedback_gain <= 0.0 ||
      !finite(config_.disturbance_adaptation_gain) ||
      config_.disturbance_adaptation_gain < 0.0 ||
      !finite(config_.parameter_leakage) ||
      config_.parameter_leakage < 0.0 ||
      !finite(config_.disturbance_leakage) ||
      config_.disturbance_leakage < 0.0 ||
      !finite(config_.robust_boundary_layer) ||
      config_.robust_boundary_layer <= 0.0 ||
      !finite(config_.disturbance_estimate_max) ||
      config_.disturbance_estimate_max < 0.0 ||
      !finite(config_.constraint_margin) ||
      config_.constraint_margin <= 0.0 ||
      !finite(config_.sustained_saturation_seconds) ||
      config_.sustained_saturation_seconds <= 0.0) {
    throw std::invalid_argument("invalid lower channel configuration");
  }
  for (std::size_t index = 0; index < parameter_estimate_.size(); ++index) {
    if (!finite(config_.adaptation_gain[index]) ||
        config_.adaptation_gain[index] < 0.0 ||
        !finite(config_.parameter_min[index]) ||
        !finite(config_.parameter_max[index]) ||
        config_.parameter_min[index] > config_.parameter_max[index]) {
      throw std::invalid_argument("invalid lower parameter configuration");
    }
  }
}

void LowerChannelController::reset(
    const std::array<double, 2>& initial_parameter,
    double initial_disturbance) {
  for (std::size_t index = 0; index < parameter_estimate_.size(); ++index) {
    if (!finite(initial_parameter[index])) {
      throw std::invalid_argument("non-finite initial parameter");
    }
    parameter_estimate_[index] = std::clamp(
        initial_parameter[index],
        config_.parameter_min[index], config_.parameter_max[index]);
  }
  if (!finite(initial_disturbance)) {
    throw std::invalid_argument("non-finite initial disturbance");
  }
  disturbance_estimate_ = std::clamp(
      initial_disturbance, 0.0, config_.disturbance_estimate_max);
  physical_saturation_duration_ = 0.0;
  output_ = {};
}

LowerChannelOutput LowerChannelController::step(
    const LowerChannelInput& input) {
  output_ = {};
  // The M2b nonlinear mapping is implemented only by M2bController.
  if (config_.mode == LowerMode::kM2b) return output_;
  const std::array<double, 12> scalar{{
      input.position_actual, input.velocity_actual,
      input.position_reference, input.velocity_reference,
      input.acceleration_reference, input.velocity_lower_bound,
      input.velocity_upper_bound, input.available_acceleration,
      input.available_deceleration, input.regressor[0],
      input.regressor[1], input.dt_seconds}};
  for (double value : scalar) {
    if (!finite(value)) return output_;
  }
  if (!finite(input.injected_matched_disturbance) ||
      input.dt_seconds <= 0.0 ||
      input.available_acceleration <= 0.0 ||
      input.available_deceleration <= 0.0 ||
      input.velocity_lower_bound >= input.velocity_upper_bound ||
      input.velocity_reference <=
          input.velocity_lower_bound + config_.constraint_margin ||
      input.velocity_reference >=
          input.velocity_upper_bound - config_.constraint_margin) {
    return output_;
  }

  output_.position_error =
      input.position_actual - input.position_reference;
  output_.velocity_error =
      input.velocity_actual - input.velocity_reference;
  const double error_lower =
      input.velocity_lower_bound - input.velocity_reference;
  const double error_upper =
      input.velocity_upper_bound - input.velocity_reference;
  if (output_.velocity_error <= error_lower + config_.constraint_margin ||
      output_.velocity_error >= error_upper - config_.constraint_margin) {
    return output_;
  }

  const double epsilon =
      input.velocity_reference - input.velocity_lower_bound;
  const double xi =
      input.velocity_upper_bound - input.velocity_reference;
  const double lower_distance = epsilon + output_.velocity_error;
  const double upper_distance = xi - output_.velocity_error;
  const double denominator = lower_distance * upper_distance;
  const double epsilon_xi = epsilon * xi;
  const double omega = epsilon_xi / denominator;
  const double omega_inverse = 1.0 / omega;
  output_.transformed_error = omega * output_.velocity_error;
  output_.transformation_gain =
      epsilon_xi *
      (epsilon_xi +
       output_.velocity_error * output_.velocity_error) /
      (denominator * denominator);
  output_.inverse_gain_term =
      output_.velocity_error * output_.velocity_error /
      (epsilon_xi +
       output_.velocity_error * output_.velocity_error) *
      (upper_distance / epsilon + lower_distance / xi) *
      input.acceleration_reference;
  output_.composite_error =
      output_.transformed_error +
      config_.composite_gain * output_.position_error;
  output_.adaptation_enabled = lowerAdaptationEnabled(config_.mode);
  output_.disturbance_compensation_enabled =
      lowerDisturbanceCompensationEnabled(config_.mode);

  double model_compensation = 0.0;
  if (output_.adaptation_enabled) {
    for (std::size_t index = 0; index < parameter_estimate_.size(); ++index) {
      model_compensation +=
          parameter_estimate_[index] * input.regressor[index];
    }
  }
  const double robust_compensation =
      output_.disturbance_compensation_enabled
          ? disturbance_estimate_ * smoothSign(
                output_.transformation_gain *
                    output_.composite_error,
                config_.robust_boundary_layer)
          : 0.0;
  output_.input_raw =
      input.acceleration_reference -
      model_compensation - output_.inverse_gain_term +
      (-config_.composite_gain * output_.velocity_error -
       omega_inverse * output_.position_error -
       config_.feedback_gain * output_.composite_error) /
          output_.transformation_gain -
      robust_compensation;
  output_.input_limited = std::clamp(
      output_.input_raw,
      -input.available_deceleration, input.available_acceleration);
  output_.input_limit_active =
      output_.input_limited != output_.input_raw;
  // A matched disturbance enters the plant after actuator limiting. It must
  // not be folded into the command or limited a second time.
  output_.input_plant =
      output_.input_limited + input.injected_matched_disturbance;

  // The command uses the current estimates.  The adaptive states are then
  // advanced explicitly, matching experiment1_v37 sample ordering.
  if (output_.adaptation_enabled) {
    for (std::size_t index = 0; index < parameter_estimate_.size(); ++index) {
      const double derivative =
          config_.adaptation_gain[index] *
              output_.transformation_gain *
              output_.composite_error * input.regressor[index] -
          config_.parameter_leakage * parameter_estimate_[index];
      parameter_estimate_[index] = std::clamp(
          parameter_estimate_[index] + input.dt_seconds * derivative,
          config_.parameter_min[index], config_.parameter_max[index]);
    }
  }
  if (output_.disturbance_compensation_enabled) {
    const double derivative =
        config_.disturbance_adaptation_gain *
            std::abs(output_.transformation_gain *
                     output_.composite_error) -
        config_.disturbance_leakage * disturbance_estimate_;
    disturbance_estimate_ = std::clamp(
        disturbance_estimate_ + input.dt_seconds * derivative,
        0.0, config_.disturbance_estimate_max);
  }

  const double unprojected_velocity =
      input.velocity_actual + input.dt_seconds * output_.input_limited;
  output_.channel_velocity_command = std::clamp(
      unprojected_velocity,
      input.velocity_lower_bound, input.velocity_upper_bound);
  output_.velocity_projection_active =
      output_.channel_velocity_command != unprojected_velocity;

  if (input.physical_wheel_saturation) {
    physical_saturation_duration_ += input.dt_seconds;
  } else {
    physical_saturation_duration_ = 0.0;
  }
  output_.sustained_physical_saturation =
      physical_saturation_duration_ >=
      config_.sustained_saturation_seconds;
  output_.parameter_estimate = parameter_estimate_;
  output_.disturbance_estimate = disturbance_estimate_;
  output_.valid = finite(output_.input_raw) &&
      finite(output_.channel_velocity_command);
  return output_;
}

}  // namespace multi_agv_control
