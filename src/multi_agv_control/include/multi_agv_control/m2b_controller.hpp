#pragma once

#include <array>
#include <cstddef>

namespace multi_agv_control {

struct M2bConfig {
  std::array<std::array<double, 3>, 3> graph{{
      {{2.8, -1.0, -0.8}},
      {{-1.0, 2.0, -1.0}},
      {{-0.8, -1.0, 2.45}}}};
  double alpha{0.15};
  double beta{0.52};
  double l0{0.90};
  double l1{1.20};
  double l2{2.25};
  double l3{1.10};
  std::array<double, 3> h{{0.006, 0.006, 0.006}};
  std::array<double, 3> k1{{1.25, 1.25, 1.25}};
  std::array<double, 3> k2{{2.75, 2.75, 2.75}};
  // Frozen literature-channel actuator limits. Runtime CapabilityReport
  // values are deliberately excluded from the M2b equations; the common
  // chassis limiter remains the only runtime physical-capability authority.
  std::array<double, 3> fixed_acceleration_limit{{1.10, 1.10, 1.10}};
  std::array<double, 3> fixed_deceleration_limit{{1.10, 1.10, 1.10}};
  double generator_guard{2.0e-5};
  double mapping_guard{2.0e-5};
  // Zero reproduces the authoritative paper-reference sign function.
  // A positive value selects the frozen linear saturation boundary layer.
  double auxiliary_boundary_layer{0.0};
  std::size_t generator_substeps{2U};
  bool golden_vectors_verified{false};
};

struct M2bGeneratorState {
  std::array<double, 3> position{{0.025, -0.018, 0.012}};
  std::array<double, 3> velocity{{0.425, 0.415, 0.440}};
  std::array<double, 3> auxiliary{{0.0, 0.0, 0.0}};
};

struct M2bInput {
  double leader_position{0.0};
  double leader_velocity{0.435};
  double leader_acceleration{0.0};
  std::array<double, 3> position_actual{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity_actual{{0.0, 0.0, 0.0}};
  // Logged for the M1/M2b comparison only; never enters M2b equations.
  std::array<double, 3> reported_capability{{0.0, 0.0, 0.0}};
  double dt_seconds{0.005};
};

struct M2bOutput {
  M2bGeneratorState current;
  M2bGeneratorState next;
  std::array<double, 3> position_disagreement{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity_disagreement{{0.0, 0.0, 0.0}};
  std::array<double, 3> auxiliary_disagreement{{0.0, 0.0, 0.0}};
  std::array<double, 3> reference_acceleration{{0.0, 0.0, 0.0}};
  std::array<double, 3> position_error{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity_error{{0.0, 0.0, 0.0}};
  std::array<double, 3> transformed_error{{0.0, 0.0, 0.0}};
  std::array<double, 3> transformation_gain{{0.0, 0.0, 0.0}};
  std::array<double, 3> inverse_gain_term{{0.0, 0.0, 0.0}};
  std::array<double, 3> composite_error{{0.0, 0.0, 0.0}};
  std::array<double, 3> delta_inverse{{0.0, 0.0, 0.0}};
  std::array<double, 3> mapping_zeta{{0.0, 0.0, 0.0}};
  std::array<double, 3> input_raw{{0.0, 0.0, 0.0}};
  std::array<double, 3> input_limited{{0.0, 0.0, 0.0}};
  std::array<double, 3> reported_capability{{0.0, 0.0, 0.0}};
  std::array<bool, 3> input_limit_active{{false, false, false}};
  double load_progress_reference{0.0};
  double load_velocity_reference{0.0};
  double load_acceleration_reference{0.0};
  bool valid{false};
};

class M2bController {
 public:
  explicit M2bController(const M2bConfig& config);

  void reset(const M2bGeneratorState& state);
  M2bOutput step(const M2bInput& input);
  const M2bGeneratorState& state() const noexcept { return state_; }
  bool formalExecutionReady() const noexcept {
    return config_.golden_vectors_verified;
  }

 private:
  M2bConfig config_;
  M2bGeneratorState state_;
};

}  // namespace multi_agv_control
