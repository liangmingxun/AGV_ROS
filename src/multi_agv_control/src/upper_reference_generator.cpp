#include "multi_agv_control/upper_reference_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multi_agv_control {
namespace {

bool finite(double value) {
  return std::isfinite(value);
}

double riskFactor(double margin, double decay) {
  const double clamped = std::clamp(margin, 0.0, 1.0);
  const double denominator = 1.0 - std::exp(-1.0 / decay);
  return (std::exp(-clamped / decay) - std::exp(-1.0 / decay)) /
         denominator;
}

std::array<double, 3> multiply(
    const std::array<std::array<double, 3>, 3>& matrix,
    const std::array<double, 3>& value) {
  std::array<double, 3> result{{0.0, 0.0, 0.0}};
  for (std::size_t row = 0; row < result.size(); ++row) {
    for (std::size_t column = 0; column < value.size(); ++column) {
      result[row] += matrix[row][column] * value[column];
    }
  }
  return result;
}

}  // namespace

DistributedReferenceOutput stepDistributedReference(
    const DistributedReferenceConfig& config,
    const DistributedReferenceState& state,
    const DistributedReferenceInput& input) {
  DistributedReferenceOutput output;
  const std::array<double, 9> scalar{{
      config.position_gain, config.position_coupling_gain,
      config.velocity_coupling_gain, config.auxiliary_gain,
      config.barrier_gain, config.auxiliary_boundary_layer,
      config.acceleration_limit, input.inner_margin, input.dt_seconds}};
  for (double value : scalar) {
    if (!finite(value)) return output;
  }
  if (config.position_gain <= 0.0 ||
      config.position_coupling_gain <= 0.0 ||
      config.velocity_coupling_gain <= 0.0 ||
      config.auxiliary_gain <= 0.0 ||
      config.barrier_gain <= 0.0 ||
      config.auxiliary_boundary_layer <= 0.0 ||
      config.acceleration_limit <= 0.0 ||
      input.inner_margin <= 0.0 || input.dt_seconds <= 0.0 ||
      !finite(input.leader_position) ||
      !finite(input.leader_velocity) ||
      !finite(input.leader_acceleration)) {
    return output;
  }

  std::array<double, 3> position_error;
  std::array<double, 3> velocity_error;
  std::array<double, 3> auxiliary_error;
  for (std::size_t i = 0; i < 3; ++i) {
    if (!finite(state.position[i]) || !finite(state.velocity[i]) ||
        !finite(state.auxiliary[i]) ||
        !finite(input.boundary[i].lower) ||
        !finite(input.boundary[i].upper) ||
        !finite(input.boundary_derivative[i].lower) ||
        !finite(input.boundary_derivative[i].upper)) {
      return output;
    }
    position_error[i] = state.position[i] - input.leader_position;
    velocity_error[i] = state.velocity[i] - input.leader_velocity;
    auxiliary_error[i] =
        state.auxiliary[i] - input.leader_acceleration;
  }
  output.position_disagreement = multiply(config.graph, position_error);
  output.velocity_disagreement = multiply(config.graph, velocity_error);
  output.auxiliary_disagreement = multiply(config.graph, auxiliary_error);
  output.common_lower = -std::numeric_limits<double>::infinity();
  output.common_upper = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < 3; ++i) {
    const double lower_inner =
        input.boundary[i].lower + input.inner_margin;
    const double upper_inner =
        input.boundary[i].upper - input.inner_margin;
    const double denominator =
        (input.boundary[i].upper - state.velocity[i]) *
        (state.velocity[i] - input.boundary[i].lower);
    if (lower_inner > upper_inner || denominator <= 1.0e-8) return output;
    const double barrier = config.barrier_gain / denominator;
    output.acceleration_unconstrained[i] =
        -config.position_coupling_gain *
             output.position_disagreement[i] -
        (config.velocity_coupling_gain + barrier) *
             output.velocity_disagreement[i] +
        state.auxiliary[i];
    double acceleration = output.acceleration_unconstrained[i];
    if (state.velocity[i] <= lower_inner) {
      acceleration =
          std::max(acceleration, input.boundary_derivative[i].lower);
    } else if (state.velocity[i] >= upper_inner) {
      acceleration =
          std::min(acceleration, input.boundary_derivative[i].upper);
    }
    output.acceleration[i] = std::clamp(
        acceleration,
        -config.acceleration_limit, config.acceleration_limit);
    output.next.position[i] =
        state.position[i] + input.dt_seconds *
        (-config.position_gain * output.position_disagreement[i] +
         state.velocity[i]);
    output.next.velocity[i] = std::clamp(
        state.velocity[i] +
            input.dt_seconds * output.acceleration[i],
        lower_inner, upper_inner);
    output.next.auxiliary[i] =
        state.auxiliary[i] + input.dt_seconds *
        (-config.auxiliary_gain * std::tanh(
            output.auxiliary_disagreement[i] /
            config.auxiliary_boundary_layer));
    output.common_lower = std::max(output.common_lower, lower_inner);
    output.common_upper = std::min(output.common_upper, upper_inner);
  }
  if (output.common_lower > output.common_upper) return output;
  double velocity_sum = 0.0;
  for (double velocity : output.next.velocity) velocity_sum += velocity;
  output.common_velocity = std::clamp(
      velocity_sum / 3.0, output.common_lower, output.common_upper);
  output.common_velocity = std::max(0.0, output.common_velocity);
  output.valid = true;
  return output;
}

UpperMode upperModeFromString(const std::string& value) {
  if (value == "M1") return UpperMode::kM1;
  if (value == "M2a") return UpperMode::kM2a;
  if (value == "M2b") return UpperMode::kM2b;
  if (value == "M4") return UpperMode::kM4;
  throw std::invalid_argument("upper mode must be M1, M2a, M2b or M4");
}

const char* upperModeName(UpperMode mode) {
  switch (mode) {
    case UpperMode::kM1: return "M1";
    case UpperMode::kM2a: return "M2a";
    case UpperMode::kM2b: return "M2b";
    case UpperMode::kM4: return "M4";
  }
  return "UNKNOWN";
}

UpperReferenceGenerator::UpperReferenceGenerator(
    const UpperReferenceConfig& config)
    : config_(config) {
  if (config_.controller_ticks_per_update == 0U ||
      !finite(config_.fixed_m4_speed) || config_.fixed_m4_speed < 0.0) {
    throw std::invalid_argument("invalid upper reference configuration");
  }
  for (std::size_t index = 0; index < boundary_.size(); ++index) {
    const auto& agent = config_.agents[index];
    if (!finite(agent.inner_margin) || agent.inner_margin <= 0.0 ||
        !(2.0 * agent.inner_margin <
          agent.admissible_set.minimum_width) ||
        !finite(agent.restore_gain_lower) ||
        !finite(agent.restore_gain_upper) ||
        !finite(agent.risk_gain_lower) ||
        !finite(agent.risk_gain_upper) ||
        !finite(agent.risk_scale) ||
        !finite(agent.risk_decay) ||
        agent.restore_gain_lower < 0.0 ||
        agent.restore_gain_upper < 0.0 ||
        agent.risk_gain_lower < 0.0 ||
        agent.risk_gain_upper < 0.0 ||
        agent.risk_scale < 0.0 || agent.risk_decay <= 0.0 ||
        !DynamicBoundaryProjector::feasible(
            agent.initial_boundary, agent.admissible_set) ||
        !DynamicBoundaryProjector::feasible(
            agent.nominal_boundary, agent.admissible_set)) {
      throw std::invalid_argument("invalid upper agent configuration");
    }
    boundary_[index] = agent.initial_boundary;
  }
}

bool UpperReferenceGenerator::setFixedM4Speed(double speed) {
  if (run_active_ || !finite(speed) || speed < 0.0) return false;
  config_.fixed_m4_speed = speed;
  return true;
}

UpperReferenceOutput UpperReferenceGenerator::step(
    const std::array<UpperAgentInput, 3>& input,
    double controller_dt_seconds) {
  // M2b is a complete coupled literature method and must use
  // M2bController; silently treating it as M2a would invalidate Exp2b.
  if (config_.mode == UpperMode::kM2b) {
    output_ = {};
    return output_;
  }
  if (!finite(controller_dt_seconds) || controller_dt_seconds <= 0.0) {
    output_.valid = false;
    return output_;
  }
  ++tick_;
  if ((tick_ - 1U) % config_.controller_ticks_per_update != 0U) {
    output_.updated = false;
    return output_;
  }

  const double upper_dt =
      controller_dt_seconds *
      static_cast<double>(config_.controller_ticks_per_update);
  output_ = {};
  output_.updated = true;
  output_.common_lower = -std::numeric_limits<double>::infinity();
  output_.common_upper = std::numeric_limits<double>::infinity();
  double candidate_sum = 0.0;

  for (std::size_t index = 0; index < input.size(); ++index) {
    const auto& agent_input = input[index];
    const auto& agent_config = config_.agents[index];
    auto& agent_output = output_.agents[index];
    if (!finite(agent_input.candidate_velocity) ||
        !finite(agent_input.mapped_upper_capability)) {
      return output_;
    }
    agent_output.candidate_velocity = agent_input.candidate_velocity;
    agent_output.logged_mapped_capability =
        agent_input.mapped_upper_capability;
    candidate_sum += agent_input.candidate_velocity;

    DynamicBoundarySet active_set = agent_config.admissible_set;
    double robust_margin = 1.0;
    for (double margin : agent_input.normalised_causal_margins) {
      if (!finite(margin)) return output_;
      robust_margin = std::min(robust_margin, std::clamp(margin, 0.0, 1.0));
    }
    agent_output.robust_margin = robust_margin;
    agent_output.risk_factor =
        riskFactor(robust_margin, agent_config.risk_decay);
    agent_output.risk_signal =
        agent_config.risk_scale * agent_output.risk_factor;

    if (config_.mode == UpperMode::kM1) {
      agent_output.pre_projection.lower =
          boundary_[index].lower + upper_dt * (
              agent_config.restore_gain_lower *
                  (agent_config.nominal_boundary.lower -
                   boundary_[index].lower) +
              agent_config.risk_gain_lower * agent_output.risk_signal);
      agent_output.pre_projection.upper =
          boundary_[index].upper + upper_dt * (
              agent_config.restore_gain_upper *
                  (agent_config.nominal_boundary.upper -
                   boundary_[index].upper) -
              agent_config.risk_gain_upper * agent_output.risk_signal);
      const auto projection =
          projector_.project(agent_output.pre_projection, active_set);
      if (!projection.valid) return output_;
      boundary_[index] = projection.projected;
      agent_output.projection_correction = projection.correction;
      agent_output.active_constraints = projection.active_constraints;
    } else {
      agent_output.pre_projection = agent_config.initial_boundary;
      boundary_[index] = agent_config.initial_boundary;
    }

    agent_output.boundary = boundary_[index];
    agent_output.inner_lower =
        boundary_[index].lower + agent_config.inner_margin;
    agent_output.inner_upper =
        boundary_[index].upper - agent_config.inner_margin;
    agent_output.valid =
        DynamicBoundaryProjector::feasible(boundary_[index], active_set) &&
        agent_output.inner_lower <= agent_output.inner_upper;
    if (!agent_output.valid) return output_;
    output_.common_lower =
        std::max(output_.common_lower, agent_output.inner_lower);
    output_.common_upper =
        std::min(output_.common_upper, agent_output.inner_upper);
  }

  if (output_.common_lower > output_.common_upper) return output_;
  if (config_.mode == UpperMode::kM4) {
    if (config_.fixed_m4_speed < output_.common_lower ||
        config_.fixed_m4_speed > output_.common_upper) {
      return output_;
    }
    output_.common_velocity = config_.fixed_m4_speed;
  } else {
    output_.common_velocity = std::clamp(
        candidate_sum / static_cast<double>(input.size()),
        output_.common_lower, output_.common_upper);
  }
  output_.valid = true;
  return output_;
}

}  // namespace multi_agv_control
