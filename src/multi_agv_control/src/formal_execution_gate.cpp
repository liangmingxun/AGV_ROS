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

FormalExecutionGateResult evaluateFormalSerialM1R1Gate(
    const FormalExecutionGateInput& input) {
  if (input.transport_type != "serial") {
    return {false, "formal serial entry requires serial transport"};
  }
  if (!input.command_publication_authorized ||
      !input.serial_execution_authorized) {
    return {false, "formal serial command publication is disabled"};
  }
  if (!input.upper_algorithm_authorized ||
      !input.lower_algorithm_authorized ||
      !input.upper_golden_verified ||
      !input.lower_golden_verified) {
    return {false, "formal serial algorithm/golden gate is disabled"};
  }
  if (!input.upper_hardware_authorized ||
      !input.lower_hardware_authorized) {
    return {false, "formal serial hardware authorization is disabled"};
  }
  if (!input.m1_r1_selected) {
    return {false, "first formal serial entry permits M1+R1 only"};
  }
  if (!input.recorder_required) {
    return {false, "formal serial entry requires the recorder heartbeat"};
  }
  if (!input.test_area_confirmed ||
      !input.wheels_on_floor_confirmed ||
      !input.unloaded_fixture_confirmed) {
    return {false, "formal serial physical confirmations are incomplete"};
  }
  return {true, ""};
}

FakeChassisBindingResult evaluateFakeChassisBinding(
    const FakeChassisBindingInput& input) {
  return evaluateChassisBinding(input, "fake");
}

FakeChassisBindingResult evaluateChassisBinding(
    const FakeChassisBindingInput& input,
    const std::string& required_transport) {
  if (required_transport != "fake" && required_transport != "serial") {
    return {
        FakeChassisBindingStatus::kRejected,
        "unsupported required chassis transport"};
  }
  for (std::size_t index = 0; index < input.transport_type.size(); ++index) {
    if (!input.transport_parameter_present[index]) {
      return {
          FakeChassisBindingStatus::kWaiting,
          "waiting for agv" + std::to_string(index + 1U) +
              " chassis transport parameter"};
    }
    if (input.transport_type[index] != required_transport) {
      return {
          FakeChassisBindingStatus::kRejected,
          "agv" + std::to_string(index + 1U) +
              " chassis transport is not " + required_transport};
    }
  }
  return {FakeChassisBindingStatus::kAllowed, ""};
}

}  // namespace multi_agv_control
