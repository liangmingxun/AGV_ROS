#pragma once

#include <array>
#include <string>

namespace multi_agv_control {

struct FormalExecutionGateInput {
  std::string transport_type;
  bool command_publication_authorized{false};
  bool upper_algorithm_authorized{false};
  bool lower_algorithm_authorized{false};
  bool upper_golden_verified{false};
  bool lower_golden_verified{false};
  bool upper_hardware_authorized{false};
  bool lower_hardware_authorized{false};
  bool m4_selected{false};
  bool m4_speed_preregistered{false};
};

struct FormalExecutionGateResult {
  bool allowed{false};
  std::string reason;
};

FormalExecutionGateResult evaluateFormalFakeGate(
    const FormalExecutionGateInput& input);

enum class FakeChassisBindingStatus {
  kWaiting,
  kAllowed,
  kRejected,
};

struct FakeChassisBindingInput {
  std::array<bool, 3> transport_parameter_present{{false, false, false}};
  std::array<std::string, 3> transport_type;
};

struct FakeChassisBindingResult {
  FakeChassisBindingStatus status{FakeChassisBindingStatus::kWaiting};
  std::string reason;
};

FakeChassisBindingResult evaluateFakeChassisBinding(
    const FakeChassisBindingInput& input);

}  // namespace multi_agv_control
