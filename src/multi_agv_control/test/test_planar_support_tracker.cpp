#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/planar_support_tracker.hpp"

using multi_agv_control::PlanarPose;
using multi_agv_control::PlanarSupportTracker;
using multi_agv_control::PlanarTrackerConfig;
using multi_agv_control::PlanarTrackingInput;
using multi_agv_control::SCurvePath;
using multi_agv_control::SupportGeometry;
using multi_agv_control::SupportGeometryConfig;
using multi_agv_control::SupportOffset;

namespace {

SupportGeometry geometry() {
  SupportGeometryConfig config;
  config.offsets = {{0.18, 0.12}, {-0.18, 0.12}, {0.0, -0.16}};
  return SupportGeometry(SCurvePath({0.12, 2.0, 20001}), config);
}

PlanarSupportTracker tracker() {
  return PlanarSupportTracker(geometry(), PlanarTrackerConfig{});
}

PlanarTrackingInput exactInput(std::size_t index, double progress,
                               double speed) {
  const auto sample = geometry().sample(index, progress);
  const PlanarPose pose{sample.position, sample.heading};
  return {index, progress, speed, pose, pose};
}

}  // namespace

TEST(PlanarSupportTracker, ZeroErrorReturnsAnalyticFeedforward) {
  const auto input = exactInput(0U, 0.4, 0.1);
  const auto expected = geometry().sample(0U, 0.4);
  const auto result = tracker().track(input);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.linear_velocity_raw, expected.speed_scale * 0.1, 1e-12);
  EXPECT_NEAR(result.angular_velocity_raw, expected.heading_rate * 0.1, 1e-12);
}

TEST(PlanarSupportTracker, PositiveBodyLongitudinalErrorRaisesLinearCommand) {
  auto input = exactInput(1U, 0.5, 0.1);
  const double baseline = tracker().track(input).linear_velocity_raw;
  input.support_pose_actual.position -=
      0.03 * Eigen::Vector2d{std::cos(input.robot_pose_actual.yaw),
                             std::sin(input.robot_pose_actual.yaw)};
  EXPECT_GT(tracker().track(input).linear_velocity_raw, baseline);
}

TEST(PlanarSupportTracker, HeadingWrapIsContinuousAcrossPi) {
  auto input = exactInput(0U, 0.0, 0.1);
  input.robot_pose_actual.yaw = input.support_pose_actual.yaw =
      input.robot_pose_actual.yaw + 2.0 * 3.14159265358979323846 - 1e-6;
  const auto result = tracker().track(input);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.heading_error, 1e-6, 1e-10);
}

TEST(PlanarSupportTracker, WheelConversionUsesFrozenSeparation) {
  const auto result = tracker().track(exactInput(2U, 0.7, 0.12));
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.wheel_linear_velocity_right_raw -
                  result.wheel_linear_velocity_left_raw,
              0.114 * result.angular_velocity_raw, 1e-12);
  EXPECT_NEAR(0.5 * (result.wheel_linear_velocity_left_raw +
                     result.wheel_linear_velocity_right_raw),
              result.linear_velocity_raw, 1e-12);
}

TEST(PlanarSupportTracker, FleetUsesOneCommonLoadProgress) {
  std::array<PlanarPose, 3> robots;
  std::array<PlanarPose, 3> supports;
  for (std::size_t index = 0; index < 3; ++index) {
    const auto sample = geometry().sample(index, 0.6);
    robots[index] = supports[index] = {sample.position, sample.heading};
  }
  const auto results = tracker().trackFleet(0.6, 0.1, robots, supports);
  for (std::size_t index = 0; index < 3; ++index) {
    EXPECT_NEAR((results[index].support_pose_reference.position -
                 geometry().sample(index, 0.6).position).norm(), 0.0, 1e-12);
  }
}

TEST(PlanarSupportTracker, RejectsBadConfigAndInput) {
  auto config = PlanarTrackerConfig{};
  config.wheel_separation[1] = 0.0;
  EXPECT_THROW({ const PlanarSupportTracker value(geometry(), config); },
               std::invalid_argument);
  auto input = exactInput(0U, 0.1, 0.1);
  input.channel_velocity_command = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker().track(input).valid);
}
