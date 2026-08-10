#include "multi_agv_control/m2b_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_agv_control {
namespace {

using Vector = std::array<double, 3>;

bool finite(double value) { return std::isfinite(value); }

Vector multiply(
    const std::array<std::array<double, 3>, 3>& matrix,
    const Vector& value) {
  Vector result{{0.0, 0.0, 0.0}};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row] += matrix[row][column] * value[column];
    }
  }
  return result;
}

double switching(double value, double layer) {
  if (layer == 0.0) return (value > 0.0) - (value < 0.0);
  return std::clamp(value / layer, -1.0, 1.0);
}

struct Rhs {
  Vector position;
  Vector velocity;
  Vector auxiliary;
  Vector zeta_position;
  Vector zeta_velocity;
  Vector zeta_auxiliary;
  bool valid{false};
};

Rhs rhs(const M2bConfig& config, const M2bGeneratorState& state,
        double leader_position, double leader_velocity,
        double leader_acceleration) {
  Rhs result;
  Vector ep, ev, ea;
  for (std::size_t i = 0; i < 3; ++i) {
    ep[i] = state.position[i] - leader_position;
    ev[i] = state.velocity[i] - leader_velocity;
    ea[i] = state.auxiliary[i] - leader_acceleration;
  }
  result.zeta_position = multiply(config.graph, ep);
  result.zeta_velocity = multiply(config.graph, ev);
  result.zeta_auxiliary = multiply(config.graph, ea);
  for (std::size_t i = 0; i < 3; ++i) {
    const double denominator =
        (config.beta - state.velocity[i]) *
        (state.velocity[i] + config.alpha);
    if (!finite(denominator) || denominator <= config.generator_guard) {
      return result;
    }
    result.position[i] =
        -config.l0 * result.zeta_position[i] + state.velocity[i];
    result.velocity[i] =
        -config.l1 * result.zeta_position[i] -
        (config.l2 + config.h[i] / denominator) *
            result.zeta_velocity[i] +
        state.auxiliary[i];
    result.auxiliary[i] =
        -config.l3 * switching(
            result.zeta_auxiliary[i], config.auxiliary_boundary_layer);
  }
  result.valid = true;
  return result;
}

M2bGeneratorState add(
    const M2bGeneratorState& state, const Rhs& derivative, double scale) {
  M2bGeneratorState result = state;
  for (std::size_t i = 0; i < 3; ++i) {
    result.position[i] += scale * derivative.position[i];
    result.velocity[i] += scale * derivative.velocity[i];
    result.auxiliary[i] += scale * derivative.auxiliary[i];
  }
  return result;
}

}  // namespace

M2bController::M2bController(const M2bConfig& config) : config_(config) {
  const std::array<double, 11> values{{
      config_.alpha, config_.beta, config_.l0, config_.l1, config_.l2,
      config_.l3, config_.generator_guard, config_.mapping_guard,
      config_.auxiliary_boundary_layer, config_.h[0], config_.k1[0]}};
  for (double value : values) {
    if (!finite(value)) throw std::invalid_argument("non-finite M2b config");
  }
  if (config_.alpha <= 0.0 || config_.beta <= 0.0 ||
      config_.l0 <= 0.0 || config_.l1 <= 0.0 || config_.l2 <= 0.0 ||
      config_.l3 <= 0.0 || config_.generator_guard <= 0.0 ||
      config_.mapping_guard <= 0.0 ||
      config_.auxiliary_boundary_layer < 0.0 ||
      config_.generator_substeps == 0U) {
    throw std::invalid_argument("invalid M2b configuration");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (!finite(config_.h[i]) || !finite(config_.k1[i]) ||
        !finite(config_.k2[i]) ||
        !finite(config_.fixed_acceleration_limit[i]) ||
        !finite(config_.fixed_deceleration_limit[i]) ||
        config_.h[i] <= 0.0 || config_.k1[i] <= 0.0 ||
        config_.k2[i] <= 0.0 ||
        config_.fixed_acceleration_limit[i] <= 0.0 ||
        config_.fixed_deceleration_limit[i] <= 0.0) {
      throw std::invalid_argument("invalid M2b agent gain");
    }
  }
  reset(state_);
}

void M2bController::reset(const M2bGeneratorState& state) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (!finite(state.position[i]) || !finite(state.velocity[i]) ||
        !finite(state.auxiliary[i]) ||
        state.velocity[i] <= -config_.alpha ||
        state.velocity[i] >= config_.beta) {
      throw std::invalid_argument("invalid M2b initial state");
    }
  }
  state_ = state;
}

M2bOutput M2bController::step(const M2bInput& input) {
  M2bOutput output;
  output.current = state_;
  output.reported_capability = input.reported_capability;
  if (!finite(input.dt_seconds) || input.dt_seconds <= 0.0 ||
      !finite(input.leader_position) || !finite(input.leader_velocity) ||
      !finite(input.leader_acceleration)) {
    return output;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (!finite(input.position_actual[i]) ||
        !finite(input.velocity_actual[i]) ||
        !finite(input.reported_capability[i])) {
      return output;
    }
  }

  const Rhs current_rhs = rhs(
      config_, state_, input.leader_position, input.leader_velocity,
      input.leader_acceleration);
  if (!current_rhs.valid) return output;
  output.position_disagreement = current_rhs.zeta_position;
  output.velocity_disagreement = current_rhs.zeta_velocity;
  output.auxiliary_disagreement = current_rhs.zeta_auxiliary;
  output.reference_acceleration = current_rhs.velocity;

  M2bGeneratorState integrated = state_;
  const double h =
      input.dt_seconds / static_cast<double>(config_.generator_substeps);
  for (std::size_t step = 0; step < config_.generator_substeps; ++step) {
    const Rhs k1 = rhs(config_, integrated, input.leader_position,
                       input.leader_velocity, input.leader_acceleration);
    const Rhs k2 = rhs(config_, add(integrated, k1, 0.5 * h),
                       input.leader_position, input.leader_velocity,
                       input.leader_acceleration);
    const Rhs k3 = rhs(config_, add(integrated, k2, 0.5 * h),
                       input.leader_position, input.leader_velocity,
                       input.leader_acceleration);
    const Rhs k4 = rhs(config_, add(integrated, k3, h),
                       input.leader_position, input.leader_velocity,
                       input.leader_acceleration);
    if (!k1.valid || !k2.valid || !k3.valid || !k4.valid) return output;
    for (std::size_t i = 0; i < 3; ++i) {
      integrated.position[i] += h *
          (k1.position[i] + 2.0 * k2.position[i] +
           2.0 * k3.position[i] + k4.position[i]) / 6.0;
      integrated.velocity[i] += h *
          (k1.velocity[i] + 2.0 * k2.velocity[i] +
           2.0 * k3.velocity[i] + k4.velocity[i]) / 6.0;
      integrated.auxiliary[i] += h *
          (k1.auxiliary[i] + 2.0 * k2.auxiliary[i] +
           2.0 * k3.auxiliary[i] + k4.auxiliary[i]) / 6.0;
    }
  }

  for (std::size_t i = 0; i < 3; ++i) {
    output.position_error[i] =
        input.position_actual[i] - state_.position[i];
    output.velocity_error[i] =
        input.velocity_actual[i] - state_.velocity[i];
    const double epsilon = state_.velocity[i] + config_.alpha;
    const double xi = config_.beta - state_.velocity[i];
    const double error = output.velocity_error[i];
    if (error <= -epsilon + config_.mapping_guard ||
        error >= xi - config_.mapping_guard) {
      return output;
    }
    const double denominator = (epsilon + error) * (xi - error);
    const double epsilon_xi = epsilon * xi;
    const double delta = epsilon_xi / denominator;
    output.delta_inverse[i] = denominator / epsilon_xi;
    output.transformed_error[i] = delta * error;
    output.transformation_gain[i] =
        epsilon_xi * (error * error + epsilon_xi) /
        (denominator * denominator);
    const double epsilon_dot = current_rhs.velocity[i];
    const double xi_dot = -epsilon_dot;
    const double numerator =
        (epsilon_dot * xi * xi - xi_dot * epsilon * epsilon) *
            error * error -
        (epsilon_dot * xi + xi_dot * epsilon) *
            error * error * error;
    output.mapping_zeta[i] =
        numerator / (denominator * denominator);
    output.inverse_gain_term[i] =
        output.mapping_zeta[i] / output.transformation_gain[i];
    output.composite_error[i] =
        output.transformed_error[i] +
        config_.k1[i] * output.position_error[i];
    output.input_raw[i] =
        (-config_.k2[i] * output.composite_error[i] -
         output.mapping_zeta[i] -
         config_.k1[i] * error -
         output.delta_inverse[i] * output.position_error[i]) /
            output.transformation_gain[i] +
        current_rhs.velocity[i];
    output.input_limited[i] = std::clamp(
        output.input_raw[i], -config_.fixed_deceleration_limit[i],
        config_.fixed_acceleration_limit[i]);
    output.input_limit_active[i] =
        std::abs(output.input_limited[i] - output.input_raw[i]) > 1.0e-12;
  }
  output.next = integrated;
  output.load_progress_reference =
      (integrated.position[0] + integrated.position[1] +
       integrated.position[2]) / 3.0;
  output.load_velocity_reference = std::max(
      0.0, (integrated.velocity[0] + integrated.velocity[1] +
            integrated.velocity[2]) / 3.0);
  output.load_acceleration_reference =
      (current_rhs.velocity[0] + current_rhs.velocity[1] +
       current_rhs.velocity[2]) / 3.0;
  output.valid = true;
  state_ = integrated;
  return output;
}

}  // namespace multi_agv_control
