#include <gtest/gtest.h>

#include "multi_agv_control/formal_execution_gate.hpp"

namespace mac = multi_agv_control;

namespace {

mac::FormalExecutionGateInput validInput() {
  mac::FormalExecutionGateInput input;
  input.transport_type = "fake";
  input.command_publication_authorized = true;
  input.upper_algorithm_authorized = true;
  input.lower_algorithm_authorized = true;
  input.upper_golden_verified = true;
  input.lower_golden_verified = true;
  return input;
}

}  // namespace

TEST(FormalExecutionGate, AcceptsOnlyCompleteFakeSoftwareGate) {
  EXPECT_TRUE(mac::evaluateFormalFakeGate(validInput()).allowed);
}

TEST(FormalExecutionGate, StructurallyRejectsSerialTransport) {
  auto input = validInput();
  input.transport_type = "serial";
  EXPECT_FALSE(mac::evaluateFormalFakeGate(input).allowed);
}

TEST(FormalExecutionGate, RejectsDisabledAlgorithmOrGoldenGate) {
  auto input = validInput();
  input.upper_algorithm_authorized = false;
  EXPECT_FALSE(mac::evaluateFormalFakeGate(input).allowed);
  input = validInput();
  input.lower_golden_verified = false;
  EXPECT_FALSE(mac::evaluateFormalFakeGate(input).allowed);
}

TEST(FormalExecutionGate, RejectsAnyHardwareAuthorization) {
  auto input = validInput();
  input.upper_hardware_authorized = true;
  EXPECT_FALSE(mac::evaluateFormalFakeGate(input).allowed);
  input = validInput();
  input.lower_hardware_authorized = true;
  EXPECT_FALSE(mac::evaluateFormalFakeGate(input).allowed);
}

TEST(FormalExecutionGate, M4RequiresPreregisteredSpeed) {
  auto input = validInput();
  input.m4_selected = true;
  EXPECT_FALSE(mac::evaluateFormalFakeGate(input).allowed);
  input.m4_speed_preregistered = true;
  EXPECT_TRUE(mac::evaluateFormalFakeGate(input).allowed);
}

TEST(FormalExecutionGate, WaitsUntilAllActualChassisTransportsExist) {
  mac::FakeChassisBindingInput input;
  input.transport_parameter_present = {{true, true, false}};
  input.transport_type = {{"fake", "fake", ""}};
  EXPECT_EQ(
      mac::FakeChassisBindingStatus::kWaiting,
      mac::evaluateFakeChassisBinding(input).status);
}

TEST(FormalExecutionGate, RejectsAnActualSerialChassisBinding) {
  mac::FakeChassisBindingInput input;
  input.transport_parameter_present = {{true, true, true}};
  input.transport_type = {{"fake", "serial", "fake"}};
  EXPECT_EQ(
      mac::FakeChassisBindingStatus::kRejected,
      mac::evaluateFakeChassisBinding(input).status);
}

TEST(FormalExecutionGate, AllowsThreeActualFakeChassisBindings) {
  mac::FakeChassisBindingInput input;
  input.transport_parameter_present = {{true, true, true}};
  input.transport_type = {{"fake", "fake", "fake"}};
  EXPECT_EQ(
      mac::FakeChassisBindingStatus::kAllowed,
      mac::evaluateFakeChassisBinding(input).status);
}
