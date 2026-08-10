#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "multi_agv_control/dynamic_boundary.hpp"

namespace multi_agv_control {

enum class UpperMode {
  kM1,
  kM2a,
  kM2b,
  kM4,
};

struct UpperAgentConfig {
  DynamicBoundarySet admissible_set;
  DynamicBoundaryState initial_boundary{-0.20, 0.20};
  DynamicBoundaryState nominal_boundary{-0.20, 0.20};
  double inner_margin{0.01};
  double restore_gain_lower{1.0};
  double restore_gain_upper{1.0};
  double risk_gain_lower{0.10};
  double risk_gain_upper{0.10};
  double risk_scale{1.0};
  double risk_decay{0.25};
};

struct UpperReferenceConfig {
  UpperMode mode{UpperMode::kM1};
  std::array<UpperAgentConfig, 3> agents;
  std::size_t controller_ticks_per_update{4U};
  double fixed_m4_speed{0.03};
  bool golden_vectors_verified{false};
};

struct UpperAgentInput {
  double candidate_velocity{0.0};
  double mapped_upper_capability{0.5};
  // Normalised causal margins in [0,1]. Index 0 may use the current
  // reported capability; all remaining entries must be one-step delayed.
  std::array<double, 5> normalised_causal_margins{{1, 1, 1, 1, 1}};
};

struct UpperAgentOutput {
  double candidate_velocity{0.0};
  double logged_mapped_capability{0.0};
  double robust_margin{0.0};
  double risk_factor{0.0};
  double risk_signal{0.0};
  DynamicBoundaryState pre_projection;
  DynamicBoundaryState boundary;
  DynamicBoundaryState projection_correction;
  std::uint32_t active_constraints{0U};
  double inner_lower{0.0};
  double inner_upper{0.0};
  bool valid{false};
};

struct UpperReferenceOutput {
  std::array<UpperAgentOutput, 3> agents;
  double common_lower{0.0};
  double common_upper{0.0};
  double common_velocity{0.0};
  bool updated{false};
  bool valid{false};
};

struct DistributedReferenceConfig {
  std::array<std::array<double, 3>, 3> graph{{
      {{2.8, -1.0, -0.8}},
      {{-1.0, 2.0, -1.0}},
      {{-0.8, -1.0, 2.45}}}};
  double position_gain{0.90};
  double position_coupling_gain{1.20};
  double velocity_coupling_gain{2.25};
  double auxiliary_gain{1.10};
  double barrier_gain{0.006};
  double auxiliary_boundary_layer{0.06};
  double acceleration_limit{1.10};
};

struct DistributedReferenceState {
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> auxiliary{{0.0, 0.0, 0.0}};
};

struct DistributedReferenceInput {
  double leader_position{0.0};
  double leader_velocity{0.0};
  double leader_acceleration{0.0};
  std::array<DynamicBoundaryState, 3> boundary;
  std::array<DynamicBoundaryState, 3> boundary_derivative;
  double inner_margin{0.01};
  double dt_seconds{0.005};
};

struct DistributedReferenceOutput {
  DistributedReferenceState next;
  std::array<double, 3> position_disagreement{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity_disagreement{{0.0, 0.0, 0.0}};
  std::array<double, 3> auxiliary_disagreement{{0.0, 0.0, 0.0}};
  std::array<double, 3> acceleration_unconstrained{{0.0, 0.0, 0.0}};
  std::array<double, 3> acceleration{{0.0, 0.0, 0.0}};
  double common_lower{0.0};
  double common_upper{0.0};
  double common_velocity{0.0};
  bool valid{false};
};

DistributedReferenceOutput stepDistributedReference(
    const DistributedReferenceConfig& config,
    const DistributedReferenceState& state,
    const DistributedReferenceInput& input);

class UpperReferenceGenerator {
 public:
  explicit UpperReferenceGenerator(const UpperReferenceConfig& config);

  UpperReferenceOutput step(
      const std::array<UpperAgentInput, 3>& input,
      double controller_dt_seconds);
  bool setFixedM4Speed(double speed);
  void setRunActive(bool active) noexcept { run_active_ = active; }
  bool formalExecutionReady() const noexcept {
    return config_.golden_vectors_verified;
  }
  const UpperReferenceOutput& output() const noexcept { return output_; }

 private:
  UpperReferenceConfig config_;
  DynamicBoundaryProjector projector_;
  std::array<DynamicBoundaryState, 3> boundary_;
  UpperReferenceOutput output_;
  std::size_t tick_{0U};
  bool run_active_{false};
};

UpperMode upperModeFromString(const std::string& value);
const char* upperModeName(UpperMode mode);

}  // namespace multi_agv_control
