#pragma once

#include <array>
#include <cstdint>
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
  bool serial_execution_authorized{false};
  bool m1_r1_selected{false};
  bool m2a_r1_selected{false};
  bool m2b_m2b_selected{false};
  bool recorder_required{false};
  bool test_area_confirmed{false};
  bool wheels_on_floor_confirmed{false};
  bool unloaded_fixture_confirmed{false};
};

struct FormalExecutionGateResult {
  bool allowed{false};
  std::string reason;
};

FormalExecutionGateResult evaluateFormalFakeGate(
    const FormalExecutionGateInput& input);

FormalExecutionGateResult evaluateFormalSerialM1R1Gate(
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

FakeChassisBindingResult evaluateChassisBinding(
    const FakeChassisBindingInput& input,
    const std::string& required_transport);

struct SerialWheelDemandAssessment {
  bool available_limit_exceeded{false};
  bool demand_threshold_exceeded{false};
  bool emergency_abort{false};
  std::string reason;
};

struct WheelPublicationCommand {
  bool valid{false};
  bool limited{false};
  double left{0.0};
  double right{0.0};
  double linear{0.0};
  double angular{0.0};
};

// Fixed per-wheel publication envelope, independent of runtime capability.
// Call only AFTER assessing and recording the original planar demand.
WheelPublicationCommand limitSerialWheelPublication(
    double demand_left, double demand_right, double wheel_separation);

SerialWheelDemandAssessment assessSerialWheelDemand(
    double raw_left, double raw_right,
    double available_left, double available_right,
    double emergency_abort_limit,
    bool raw_exceedance_warning_only = false);

enum class FeedbackFreshnessStatus {
  kFresh,
  kTransientHold,
  kStale,
  kInvalidTiming,
};

FeedbackFreshnessStatus assessFeedbackFreshness(
    double receive_age_seconds,
    double maximum_fresh_age_seconds,
    double transient_hold_seconds);

bool seedCommandSequenceFromFeedback(
    std::uint32_t command_seq_applied,
    std::uint32_t* command_sequence);

}  // namespace multi_agv_control
