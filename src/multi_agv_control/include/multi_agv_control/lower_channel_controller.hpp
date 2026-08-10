#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace multi_agv_control {

enum class LowerMode {
  kR1,
  kR2,
  kR3,
  kR4,
  kM2b,
};

struct LowerChannelConfig {
  LowerMode mode{LowerMode::kR1};
  // Defaults are the approved experiment1_v37 complete-lower parameters.
  double composite_gain{1.2};
  double feedback_gain{2.4};
  std::array<double, 2> adaptation_gain{{0.12, 0.12}};
  double disturbance_adaptation_gain{1.0};
  double parameter_leakage{0.025};
  double disturbance_leakage{0.08};
  double robust_boundary_layer{0.055};
  std::array<double, 2> parameter_min{{-0.8, -0.8}};
  std::array<double, 2> parameter_max{{0.8, 0.8}};
  double disturbance_estimate_max{0.55};
  double constraint_margin{2.0e-5};
  double sustained_saturation_seconds{0.5};
  bool golden_vectors_verified{false};
};

struct LowerChannelInput {
  double position_actual{0.0};
  double velocity_actual{0.0};
  double position_reference{0.0};
  double velocity_reference{0.0};
  double acceleration_reference{0.0};
  double velocity_lower_bound{-0.5};
  double velocity_upper_bound{0.5};
  double available_acceleration{1.0};
  double available_deceleration{1.0};
  std::array<double, 2> regressor{{0.0, 0.0}};
  double injected_matched_disturbance{0.0};
  bool physical_wheel_saturation{false};
  double dt_seconds{0.01};
};

struct LowerChannelOutput {
  double position_error{0.0};
  double velocity_error{0.0};
  // Paper notation: psi, Gamma and Gamma^{-1}Upsilon.
  double transformed_error{0.0};
  double transformation_gain{0.0};
  double inverse_gain_term{0.0};
  double composite_error{0.0};
  double input_raw{0.0};
  double input_plant{0.0};
  double input_limited{0.0};
  double channel_velocity_command{0.0};
  std::array<double, 2> parameter_estimate{{0.0, 0.0}};
  double disturbance_estimate{0.0};
  bool adaptation_enabled{false};
  bool disturbance_compensation_enabled{false};
  bool input_limit_active{false};
  bool velocity_projection_active{false};
  bool sustained_physical_saturation{false};
  bool valid{false};
};

class LowerChannelController {
 public:
  explicit LowerChannelController(const LowerChannelConfig& config);

  LowerChannelOutput step(const LowerChannelInput& input);
  void reset(
      const std::array<double, 2>& initial_parameter = {{0.0, 0.0}},
      double initial_disturbance = 0.0);
  bool formalExecutionReady() const noexcept {
    return config_.golden_vectors_verified;
  }
  const LowerChannelOutput& output() const noexcept { return output_; }

 private:
  LowerChannelConfig config_;
  std::array<double, 2> parameter_estimate_{{0.0, 0.0}};
  double disturbance_estimate_{0.0};
  double physical_saturation_duration_{0.0};
  LowerChannelOutput output_;
};

LowerMode lowerModeFromString(const std::string& value);
const char* lowerModeName(LowerMode mode);
bool lowerAdaptationEnabled(LowerMode mode);
bool lowerDisturbanceCompensationEnabled(LowerMode mode);

}  // namespace multi_agv_control
