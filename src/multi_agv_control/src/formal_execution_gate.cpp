#include "multi_agv_control/formal_execution_gate.hpp"

#include <cstddef>

namespace multi_agv_control {

FormalExecutionGateResult evaluateFormalFakeGate(
    const FormalExecutionGateInput& input) {
  if (input.transport_type != "fake") {
    return {false, "formal algorithm integration accepts fake transport only"};
  }
  if (!input.command_publication_authorized) {
    return {false, "formal fake command publication is disabled"};
  }
  if (!input.upper_algorithm_authorized ||
      !input.lower_algorithm_authorized) {
    return {false, "formal algorithm execution gate is disabled"};
  }
  if (!input.upper_golden_verified || !input.lower_golden_verified) {
    return {false, "formal golden-vector gate is disabled"};
  }
  if (input.upper_hardware_authorized ||
      input.lower_hardware_authorized) {
    return {
        false,
        "hardware authorization must remain false in fake-only entry"};
  }
  if (input.m4_selected && !input.m4_speed_preregistered) {
    return {false, "M4 speed must be pre-registered"};
  }
  return {true, ""};
}

FakeChassisBindingResult evaluateFakeChassisBinding(
    const FakeChassisBindingInput& input) {
  for (std::size_t index = 0; index < input.transport_type.size(); ++index) {
    if (!input.transport_parameter_present[index]) {
      return {
          FakeChassisBindingStatus::kWaiting,
          "waiting for agv" + std::to_string(index + 1U) +
              " chassis transport parameter"};
    }
    if (input.transport_type[index] != "fake") {
      return {
          FakeChassisBindingStatus::kRejected,
          "agv" + std::to_string(index + 1U) +
              " chassis transport is not fake"};
    }
  }
  return {FakeChassisBindingStatus::kAllowed, ""};
}

}  // namespace multi_agv_control
