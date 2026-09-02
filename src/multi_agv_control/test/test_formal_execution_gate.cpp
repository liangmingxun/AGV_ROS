#include <gtest/gtest.h>

#include <limits>

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

TEST(FormalExecutionGate, AcceptsCompleteSerialM1R1Gate) {
  auto input = validInput();
  input.transport_type = "serial";
  input.upper_hardware_authorized = true;
  input.lower_hardware_authorized = true;
  input.serial_execution_authorized = true;
  input.m1_r1_selected = true;
  input.recorder_required = true;
  input.test_area_confirmed = true;
  input.wheels_on_floor_confirmed = true;
  input.unloaded_fixture_confirmed = true;
  EXPECT_TRUE(mac::evaluateFormalSerialM1R1Gate(input).allowed);
}

TEST(FormalExecutionGate, SerialGateRejectsMissingRecorderOrConfirmation) {
  auto input = validInput();
  input.transport_type = "serial";
  input.upper_hardware_authorized = true;
  input.lower_hardware_authorized = true;
  input.serial_execution_authorized = true;
  input.m1_r1_selected = true;
  input.recorder_required = false;
  input.test_area_confirmed = true;
  input.wheels_on_floor_confirmed = true;
  input.unloaded_fixture_confirmed = true;
  EXPECT_FALSE(mac::evaluateFormalSerialM1R1Gate(input).allowed);
  input.recorder_required = true;
  input.wheels_on_floor_confirmed = false;
  EXPECT_FALSE(mac::evaluateFormalSerialM1R1Gate(input).allowed);
}

TEST(FormalExecutionGate, SerialBindingRequiresAllThreeSerialChassis) {
  mac::FakeChassisBindingInput input;
  input.transport_parameter_present = {{true, true, true}};
  input.transport_type = {{"serial", "serial", "serial"}};
  EXPECT_EQ(
      mac::FakeChassisBindingStatus::kAllowed,
      mac::evaluateChassisBinding(input, "serial").status);
  input.transport_type[1] = "fake";
  EXPECT_EQ(
      mac::FakeChassisBindingStatus::kRejected,
      mac::evaluateChassisBinding(input, "serial").status);
}

TEST(FormalExecutionGate, AvailableWheelExcessIsLimitedNotAborted) {
  const auto result = mac::assessSerialWheelDemand(
      0.066403, 0.080072, 0.08, 0.08, 0.12);
  EXPECT_TRUE(result.available_limit_exceeded);
  EXPECT_FALSE(result.emergency_abort);
}

TEST(FormalExecutionGate, EmergencyWheelExcessAndNonfiniteDemandAbort) {
  auto result = mac::assessSerialWheelDemand(
      0.07, 0.120001, 0.08, 0.08, 0.12);
  EXPECT_TRUE(result.available_limit_exceeded);
  EXPECT_TRUE(result.emergency_abort);
  result = mac::assessSerialWheelDemand(
      std::numeric_limits<double>::quiet_NaN(), 0.0,
      0.08, 0.08, 0.12);
  EXPECT_TRUE(result.emergency_abort);
}

TEST(FormalExecutionGate, EmergencyLimitMustBeIndependentOfAvailableLimit) {
  const auto result = mac::assessSerialWheelDemand(
      0.01, 0.01, 0.08, 0.08, 0.08);
  EXPECT_TRUE(result.emergency_abort);
}
