#include <gtest/gtest.h>

#include <limits>

#include "multi_agv_control/execution_channel_reconciler.hpp"

namespace mac = multi_agv_control;

TEST(ExecutionChannelReconciler, DisabledCandidateIsAnExactNoOp) {
  mac::ExecutionChannelReconcilerConfig config;
  config.enabled = false;
  const mac::ExecutionChannelReconciler reconciler(config);
  const auto result = reconciler.step(
      .099, .080, .099, 0.0, 1.0, .01, -.15, .095, true);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.next_channel_velocity, .099);
}

TEST(ExecutionChannelReconciler, SlowlyRemovesSustainedExcessCommand) {
  mac::ExecutionChannelReconcilerConfig config;
  config.enabled = true;
  const mac::ExecutionChannelReconciler reconciler(config);
  const auto result = reconciler.step(
      .099, .080, .099, 0.0, 1.0, .01, -.15, .105, true);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.equivalent_channel_velocity, .080, 1e-12);
  EXPECT_NEAR(result.mismatch, -.019, 1e-12);
  EXPECT_NEAR(result.correction_acceleration, -.00085, 1e-12);
  EXPECT_NEAR(result.next_channel_velocity, .0989915, 1e-12);
}

TEST(ExecutionChannelReconciler, DeadbandAndDeratingGatePreventChatter) {
  mac::ExecutionChannelReconcilerConfig config;
  config.enabled = true;
  const mac::ExecutionChannelReconciler reconciler(config);
  auto result = reconciler.step(
      .080, .079, .080, 0.0, 1.0, .01, -.15, .105, true);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.active);
  result = reconciler.step(
      .099, .080, .099, 0.0, 1.0, .01, -.15, .105, false);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.active);
}

TEST(ExecutionChannelReconciler, LimitsRateAndProjectsToExistingEnvelope) {
  mac::ExecutionChannelReconcilerConfig config;
  config.enabled = true;
  config.gain_per_second = 10.0;
  const mac::ExecutionChannelReconciler reconciler(config);
  const auto result = reconciler.step(
      .09499, .2, .09499, 0.0, 1.0, .01, -.15, .095, true);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.correction_limited);
  EXPECT_DOUBLE_EQ(result.next_channel_velocity, .095);
}

TEST(ExecutionChannelReconciler, RejectsInvalidTimingAndGeometry) {
  mac::ExecutionChannelReconcilerConfig config;
  config.enabled = true;
  const mac::ExecutionChannelReconciler reconciler(config);
  EXPECT_FALSE(reconciler.step(
      .08, .08, .08, 0.0, 1.0, 0.0, -.15, .105, true).valid);
  const auto small_scale = reconciler.step(
      .08, .08, .08, 0.0, .1, .01, -.15, .105, true);
  EXPECT_TRUE(small_scale.valid);
  EXPECT_FALSE(small_scale.active);
  EXPECT_FALSE(reconciler.step(
      .08, std::numeric_limits<double>::infinity(), .08, 0.0, 1.0,
      .01, -.15, .105, true).valid);
}
