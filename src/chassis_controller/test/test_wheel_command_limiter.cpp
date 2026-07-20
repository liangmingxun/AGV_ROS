#include <cmath>
#include <limits>
#include <gtest/gtest.h>

#include "chassis_controller/wheel_command_limiter.hpp"

using chassis_controller::WheelCommand;
using chassis_controller::WheelCommandLimiter;
using chassis_controller::WheelLimits;

TEST(WheelCommandLimiter, FinalClampHonorsNewLowerLimit) {
  WheelCommandLimiter limiter({0.8, 0.8});
  WheelLimits limits{0.5, 0.5, 1.0, 1.0, 0.2, 0.2};
  const auto out = limiter.update({0.8, 0.8}, limits, 0.01);
  EXPECT_DOUBLE_EQ(out.applied.left, 0.5);
  EXPECT_TRUE(out.speed_limited_left);
}

TEST(WheelCommandLimiter, ReversalDeceleratesToZeroFirst) {
  WheelCommandLimiter limiter({0.4, 0.4});
  WheelLimits limits{0.9, 0.9, 1.0, 1.0, 2.0, 2.0};
  const auto out = limiter.update({-0.4, -0.4}, limits, 0.1);
  EXPECT_NEAR(out.applied.left, 0.2, 1e-12);
  EXPECT_GT(out.applied.left, 0.0);
  EXPECT_TRUE(out.decel_limited_left);
}

TEST(WheelCommandLimiter, ReversalThatReachesZeroStillReportsDecelLimit) {
  WheelCommandLimiter limiter({0.1, 0.1});
  WheelLimits limits{0.9, 0.9, 1.0, 1.0, 2.0, 2.0};
  const auto out = limiter.update({-0.4, -0.4}, limits, 0.1);
  EXPECT_DOUBLE_EQ(out.applied.left, 0.0);
  EXPECT_TRUE(out.decel_limited_left);
}

TEST(WheelCommandLimiter, UsesAccelerationAndDecelerationIndependently) {
  WheelCommandLimiter limiter({0.1, 0.5});
  WheelLimits limits{1.0, 1.0, 1.0, 4.0, 2.0, 1.0};
  const auto out = limiter.update({0.8, 0.2}, limits, 0.1);
  EXPECT_NEAR(out.applied.left, 0.2, 1e-12);
  EXPECT_NEAR(out.applied.right, 0.4, 1e-12);
  EXPECT_TRUE(out.accel_limited_left);
  EXPECT_TRUE(out.decel_limited_right);
}

TEST(WheelCommandLimiter, RejectsInvalidInputsWithoutChangingState) {
  WheelCommandLimiter limiter({0.2, 0.2});
  const WheelLimits limits{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  EXPECT_FALSE(limiter.update({0.8, 0.8}, limits, 0.0).valid);
  EXPECT_FALSE(limiter.update({std::numeric_limits<double>::quiet_NaN(), 0.8},
                              limits, 0.1).valid);
  EXPECT_DOUBLE_EQ(limiter.applied().left, 0.2);
}
