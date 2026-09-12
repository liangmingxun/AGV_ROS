#include <gtest/gtest.h>
#include <limits>
#include <cmath>
#include "multi_agv_control/execution_reference_derivative.hpp"
#include "multi_agv_control/upper_reference_generator.hpp"
#include "multi_agv_control/lower_channel_controller.hpp"
namespace mac = multi_agv_control;

TEST(ExecutionReferenceDerivative, ProjectedSteadySpeedHasZeroAcceleration) {
  mac::ExecutionReferenceDerivative derivative;
  derivative.update(.08, .01);
  for (int tick = 0; tick < 1000; ++tick) {
    const auto out = derivative.update(.08, .01);
    ASSERT_TRUE(out.valid);
    EXPECT_DOUBLE_EQ(out.acceleration, 0.);
  }
}

TEST(ExecutionReferenceDerivative, QuinticStartAndFaultRestartStayConsistent) {
  mac::ExecutionReferenceDerivative derivative;
  for (int repeat = 0; repeat < 2; ++repeat) {
    derivative.reset();
    double previous = 0.;
    for (int tick = 1; tick <= 320; ++tick) {
      double q = tick / 320.;
      double velocity = .1*q*q*q*(10.+q*(-15.+6.*q));
      const auto out = derivative.update(velocity, .01);
      ASSERT_TRUE(out.valid);
      EXPECT_NEAR(out.acceleration, (velocity-previous)/.01, 1e-12);
      EXPECT_GE(out.acceleration, -1e-12);
      EXPECT_LE(out.acceleration, .059);
      previous = velocity;
    }
    EXPECT_NEAR(derivative.update(.1, .01).acceleration, 0., 1e-12);
  }
}

TEST(ExecutionReferenceDerivative, DownAndUpAreSignedAndNoConstraintIsDelayed) {
  mac::ExecutionReferenceDerivative derivative;
  derivative.update(.1, .01);
  for (int tick = 1; tick <= 100; ++tick)
    EXPECT_NEAR(derivative.update(.1-.02*tick/100., .01).acceleration, -.02, 1e-12);
  EXPECT_NEAR(derivative.update(.08, .01).acceleration, 0., 1e-12);
  for (int tick = 1; tick <= 100; ++tick)
    EXPECT_NEAR(derivative.update(.08+.02*tick/100., .01).acceleration, .02, 1e-12);
  // An abrupt projected boundary must be represented, not hidden by smoothing.
  EXPECT_NEAR(derivative.update(.07, .01).acceleration, -3., 1e-12);
}

TEST(ExecutionReferenceDerivative, RejectsNonfiniteAndInvalidTime) {
  mac::ExecutionReferenceDerivative derivative;
  EXPECT_FALSE(derivative.update(std::numeric_limits<double>::infinity(), .01).valid);
  EXPECT_FALSE(derivative.update(.1, 0.).valid);
  EXPECT_FALSE(derivative.update(.1, -.01).valid);
  EXPECT_FALSE(derivative.update(.1, .101).valid);
  EXPECT_TRUE(derivative.update(.001, .01).valid);
}

TEST(ExecutionReferenceDerivative, RealDistributedProjectionDoesNotSupplyFalseFeedforward) {
  mac::DistributedReferenceConfig config;
  mac::DistributedReferenceState state;
  state.velocity = {{.09, .08, .09}};
  mac::DistributedReferenceInput input;
  input.leader_velocity = .1;
  input.inner_margin = .005;
  input.dt_seconds = .01;
  input.boundary = {{{-.015, .115}, {-.015, .085}, {-.015, .115}}};
  mac::ExecutionReferenceDerivative derivative;
  derivative.update(.08, .01);
  bool saw_mismatch = false;
  for (int tick = 0; tick < 1000; ++tick) {
    input.leader_position += .001;
    const auto out = mac::stepDistributedReference(config, state, input);
    ASSERT_TRUE(out.valid);
    const auto actual_derivative = derivative.update(out.common_velocity, .01);
    ASSERT_TRUE(actual_derivative.valid);
    const double average_local = (out.acceleration[0]+out.acceleration[1]+out.acceleration[2])/3.;
    if (std::abs(out.common_velocity-.08)<1e-12 &&
        std::abs(actual_derivative.acceleration)<1e-12 &&
        std::abs(average_local)>.01) saw_mismatch = true;
    state = out.next;
  }
  EXPECT_TRUE(saw_mismatch);
}

TEST(ExecutionReferenceDerivative, RecordedFalseFeedforwardBiasInUncalibratedSensitivityPlant) {
  // Exposes the interface mechanism only; this is not a three-car or
  // identified STM32 simulation, and its RMS is not an actual-run prediction.
  const auto run = [](double acceleration_feedforward) {
    mac::LowerChannelController lower{mac::LowerChannelConfig{}};
    lower.reset({{0., 0.}}, .08);
    double s = 0., v = .08, reference = 0., command = .08, sum = 0.;
    for (int tick = 0; tick < 5000; ++tick) {
      reference += .0008;
      mac::LowerChannelInput in;
      in.position_actual = s; in.velocity_actual = v;
      in.position_reference = reference; in.velocity_reference = .08;
      in.acceleration_reference = acceleration_feedforward;
      in.velocity_lower_bound = -.15; in.velocity_upper_bound = .58;
      in.available_acceleration = .3; in.available_deceleration = .3;
      in.regressor = {{std::sin(s), v*v}};
      const auto out = lower.step(in);
      if (!out.valid) return std::numeric_limits<double>::infinity();
      command = std::max(-.15, std::min(.125, command+.01*out.input_limited));
      const double demand = std::max(-.16, std::min(.16, command-(s-reference)));
      v += -std::expm1(-.01/.15)*(1.13*demand-v);
      s += .01*v;
      if (tick >= 3000) sum += (s-reference)*(s-reference);
    }
    return std::sqrt(sum/2000.);
  };
  const double original = run(.08324);
  const double consistent = run(0.);
  std::cout << "Uncalibrated 1-D RMS: original=" << original
            << ", consistent=" << consistent << std::endl;
  EXPECT_TRUE(std::isfinite(original));
  EXPECT_LT(consistent, original);
}
