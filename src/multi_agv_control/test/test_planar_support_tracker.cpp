#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/planar_support_tracker.hpp"

using multi_agv_control::PlanarPose;
using multi_agv_control::FleetPlanarExecutionAdapter;
using multi_agv_control::FleetPlanarExecutionConfig;
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
  config.offsets = {
      {0.1732050807568877, 0.0},
      {-0.0866025403784439, 0.1500000000000000},
      {-0.0866025403784439, -0.1500000000000000}};
  return SupportGeometry(SCurvePath({0.05, 1.0, 20001}), config);
}

PlanarSupportTracker tracker() {
  PlanarTrackerConfig config;
  config.base_to_support = {
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Zero()};
  return PlanarSupportTracker(geometry(), config);
}

PlanarSupportTracker measuredOffsetTracker() {
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
  input.robot_pose_actual.position -=
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

TEST(FleetPlanarExecutionAdapter,
     ExactFleetUsesFrozenDirectionalFeedforwardAndWheelGeometry) {
  const auto fleet_tracker = tracker();
  FleetPlanarExecutionConfig config;
  config.wheel_separation = {{0.13, 0.14, 0.15}};
  config.angular_feedforward_scale_positive = {{0.78, 0.80, 0.82}};
  config.angular_feedforward_scale_negative = {{0.88, 0.90, 0.92}};
  const FleetPlanarExecutionAdapter adapter(fleet_tracker, config);
  std::array<PlanarPose, 3> robots;
  std::array<PlanarPose, 3> supports;
  constexpr double progress = 0.35;
  constexpr double speed = 0.05;
  for (std::size_t index = 0U; index < 3U; ++index) {
    const auto preview = fleet_tracker.track(
        exactInput(index, progress, speed));
    ASSERT_TRUE(preview.valid);
    robots[index] = preview.chassis_pose_reference;
    supports[index] = preview.support_pose_reference;
  }
  auto result = fleet_tracker.trackFleet(
      progress, speed, robots, supports);
  const auto unadapted = result;
  adapter.adapt(
      progress, {{speed, speed, speed}}, robots, supports, &result);
  for (std::size_t index = 0U; index < 3U; ++index) {
    ASSERT_TRUE(result[index].valid);
    const double expected_scale =
        unadapted[index].angular_velocity_feedforward > 0.0
            ? config.angular_feedforward_scale_positive[index]
            : config.angular_feedforward_scale_negative[index];
    EXPECT_NEAR(
        result[index].angular_velocity_feedforward,
        expected_scale * unadapted[index].angular_velocity_feedforward,
        1e-12);
    EXPECT_NEAR(
        result[index].wheel_linear_velocity_right_raw -
            result[index].wheel_linear_velocity_left_raw,
        config.wheel_separation[index] * result[index].angular_velocity_raw,
        1e-12);
  }
}

TEST(PlanarSupportTracker, MeasuredRearSupportOffsetChangesChassisReference) {
  auto measured_tracker = measuredOffsetTracker();
  auto input = exactInput(0U, 0.4, 0.1);
  const auto preview = measured_tracker.track(input);
  ASSERT_TRUE(preview.valid);
  input.robot_pose_actual = preview.chassis_pose_reference;
  input.support_pose_actual = preview.support_pose_reference;
  const auto measured = measured_tracker.track(input);
  const auto zero_offset = tracker().track(exactInput(0U, 0.4, 0.1));
  ASSERT_TRUE(measured.valid);
  EXPECT_GT((measured.chassis_pose_reference.position -
             measured.support_pose_reference.position).norm(), 0.017);
  const Eigen::Vector2d reconstructed_support =
      measured.chassis_pose_reference.position +
      Eigen::Vector2d{
          -0.01783 * std::cos(measured.chassis_pose_reference.yaw),
          -0.01783 * std::sin(measured.chassis_pose_reference.yaw)};
  EXPECT_NEAR((reconstructed_support -
               measured.support_pose_reference.position).norm(), 0.0, 1e-12);
  EXPECT_NE(measured.chassis_pose_reference.yaw,
            measured.support_pose_reference.yaw);
  EXPECT_NE(measured.linear_velocity_feedforward,
            zero_offset.linear_velocity_feedforward);
  EXPECT_NE(measured.angular_velocity_feedforward,
            zero_offset.angular_velocity_feedforward);

  constexpr double ds = 1e-5;
  const auto before =
      measured_tracker.track(exactInput(0U, 0.4 - ds, 0.1));
  const auto after =
      measured_tracker.track(exactInput(0U, 0.4 + ds, 0.1));
  const Eigen::Vector2d chassis_derivative =
      (after.chassis_pose_reference.position -
       before.chassis_pose_reference.position) /
      (2.0 * ds);
  const Eigen::Vector2d tangent{
      std::cos(measured.chassis_pose_reference.yaw),
      std::sin(measured.chassis_pose_reference.yaw)};
  const Eigen::Vector2d normal{-tangent.y(), tangent.x()};
  EXPECT_NEAR(normal.dot(chassis_derivative), 0.0, 2e-5);
  EXPECT_NEAR(tangent.dot(chassis_derivative),
              measured.linear_velocity_feedforward / 0.1, 2e-5);
}

TEST(PlanarSupportTracker, Robot1ShortSPretestStaysInsideDedicatedWheelBound) {
  const SCurvePath path({0.05, 1.0, 20001});
  SupportGeometryConfig geometry_config;
  geometry_config.offsets = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
  PlanarTrackerConfig tracker_config;
  const PlanarSupportTracker pretest_tracker(
      SupportGeometry(path, geometry_config), tracker_config);

  double maximum_wheel_speed = 0.0;
  double minimum_wheel_difference = std::numeric_limits<double>::infinity();
  double maximum_wheel_difference = -std::numeric_limits<double>::infinity();
  const PlanarPose dummy{{0.0, 0.0}, 0.0};
  for (std::size_t index = 0U; index <= 2000U; ++index) {
    const double progress =
        path.length() * static_cast<double>(index) / 2000.0;
    const auto preview = pretest_tracker.track(
        {0U, progress, 0.05, dummy, dummy});
    ASSERT_TRUE(preview.valid);
    const auto exact = pretest_tracker.track(
        {0U, progress, 0.05, preview.chassis_pose_reference,
         preview.support_pose_reference});
    ASSERT_TRUE(exact.valid);
    maximum_wheel_speed = std::max(
        maximum_wheel_speed,
        std::max(std::abs(exact.wheel_linear_velocity_left_raw),
                 std::abs(exact.wheel_linear_velocity_right_raw)));
    const double difference =
        exact.wheel_linear_velocity_right_raw -
        exact.wheel_linear_velocity_left_raw;
    minimum_wheel_difference =
        std::min(minimum_wheel_difference, difference);
    maximum_wheel_difference =
        std::max(maximum_wheel_difference, difference);
  }
  EXPECT_LT(maximum_wheel_speed, 0.08);
  EXPECT_LT(minimum_wheel_difference, -1e-3);
  EXPECT_GT(maximum_wheel_difference, 1e-3);
}

TEST(PlanarSupportTracker, SmoothClockwiseHalfMetreCircleStaysInsideWheelBound) {
  multi_agv_control::SCurveConfig path_config;
  path_config.amplitude = 0.0;
  path_config.longitudinal_length =
      0.60 + 3.14159265358979323846;
  path_config.lookup_samples = 40001;
  path_config.model = "circle_smooth_entry";
  path_config.circle_radius = 0.5;
  path_config.entry_straight_length = 0.20;
  path_config.curvature_ramp_length = 0.40;
  path_config.circle_direction = -1.0;
  const SCurvePath path(path_config);
  SupportGeometryConfig geometry_config;
  geometry_config.offsets = {
      {0.1732050807568877, 0.0},
      {-0.0866025403784439, 0.1500000000000000},
      {-0.0866025403784439, -0.1500000000000000}};
  const SupportGeometry circle_geometry(path, geometry_config);
  PlanarTrackerConfig tracker_config;
  tracker_config.wheel_separation = {
      0.135484339, 0.139284482, 0.136802843};
  const PlanarSupportTracker circle_tracker(
      circle_geometry, tracker_config);

  double maximum_wheel_speed = 0.0;
  double maximum_initial_heading = 0.0;
  const PlanarPose dummy{{0.0, 0.0}, 0.0};
  for (std::size_t step = 0U; step <= 4000U; ++step) {
    const double progress = path.length() *
        static_cast<double>(step) / 4000.0;
    for (std::size_t robot = 0U; robot < 3U; ++robot) {
      const auto preview = circle_tracker.track(
          {robot, progress, 0.08, dummy, dummy});
      ASSERT_TRUE(preview.valid);
      if (step == 0U) {
        maximum_initial_heading = std::max(
            maximum_initial_heading,
            std::abs(preview.chassis_pose_reference.yaw));
      }
      const auto exact = circle_tracker.track(
          {robot, progress, 0.08, preview.chassis_pose_reference,
           preview.support_pose_reference});
      ASSERT_TRUE(exact.valid);
      maximum_wheel_speed = std::max(
          maximum_wheel_speed,
          std::max(std::abs(exact.wheel_linear_velocity_left_raw),
                   std::abs(exact.wheel_linear_velocity_right_raw)));
    }
  }
  // The straight entry must be compatible with the existing parallel camera
  // formation initializer; the old immediate circle violated this property.
  EXPECT_LT(maximum_initial_heading, 0.05);
  EXPECT_LT(maximum_wheel_speed, 0.14);
}

TEST(PlanarSupportTracker,
     FrozenEquilateralFixtureMatchesStartTransformsAndWheelBound) {
  const auto fixture_geometry = geometry();
  const auto& offsets = fixture_geometry.config().offsets;
  const auto distance = [](const SupportOffset& lhs,
                           const SupportOffset& rhs) {
    return std::hypot(lhs.tangent - rhs.tangent, lhs.normal - rhs.normal);
  };
  EXPECT_NEAR(distance(offsets[0], offsets[1]), 0.30, 1e-14);
  EXPECT_NEAR(distance(offsets[0], offsets[2]), 0.30, 1e-14);
  EXPECT_NEAR(distance(offsets[1], offsets[2]), 0.30, 1e-14);
  EXPECT_GT(offsets[0].tangent, 0.0);
  EXPECT_GT(offsets[1].normal, 0.0);
  EXPECT_LT(offsets[2].normal, 0.0);

  const auto measured_tracker = measuredOffsetTracker();
  const PlanarPose dummy{{0.0, 0.0}, 0.0};
  const std::array<PlanarPose, 3> expected_support{{
      {{0.165242534262182, 0.051912473169863}, 0.304395797364615},
      {{-0.127578787669470, 0.117147995871838}, 0.304395797364615},
      {{-0.037663746592711, -0.169060469041701}, 0.304395797364615}}};
  const std::array<PlanarPose, 3> expected_chassis{{
      {{0.182428656870112, 0.056660745456199}, 0.269560710208377},
      {{-0.110631929722283, 0.122689918436964}, 0.316055759004345},
      {{-0.020728527955413, -0.163483079893820}, 0.318149288880721}}};
  for (std::size_t index = 0U; index < 3U; ++index) {
    const auto preview =
        measured_tracker.track({index, 0.0, 0.05, dummy, dummy});
    ASSERT_TRUE(preview.valid);
    EXPECT_NEAR(
        (preview.support_pose_reference.position -
         expected_support[index].position).norm(), 0.0, 1e-12);
    EXPECT_NEAR(
        (preview.chassis_pose_reference.position -
         expected_chassis[index].position).norm(), 0.0, 1e-12);
    EXPECT_NEAR(preview.chassis_pose_reference.yaw,
                expected_chassis[index].yaw, 1e-12);
  }

  double maximum_wheel_speed = 0.0;
  for (std::size_t sample = 0U; sample <= 2000U; ++sample) {
    const double progress =
        fixture_geometry.path().length() *
        static_cast<double>(sample) / 2000.0;
    for (std::size_t index = 0U; index < 3U; ++index) {
      const auto preview =
          measured_tracker.track({index, progress, 0.05, dummy, dummy});
      const auto exact = measured_tracker.track(
          {index, progress, 0.05, preview.chassis_pose_reference,
           preview.support_pose_reference});
      ASSERT_TRUE(exact.valid);
      maximum_wheel_speed = std::max(
          maximum_wheel_speed,
          std::max(std::abs(exact.wheel_linear_velocity_left_raw),
                   std::abs(exact.wheel_linear_velocity_right_raw)));
    }
  }
  EXPECT_LT(maximum_wheel_speed, 0.08);
}

TEST(PlanarSupportTracker, RejectsBadConfigAndInput) {
  auto config = PlanarTrackerConfig{};
  config.wheel_separation[1] = 0.0;
  EXPECT_THROW({ const PlanarSupportTracker value(geometry(), config); },
               std::invalid_argument);
  config = PlanarTrackerConfig{};
  config.chassis_reference_samples = 1U;
  EXPECT_THROW({ const PlanarSupportTracker value(geometry(), config); },
               std::invalid_argument);
  config = PlanarTrackerConfig{};
  config.base_to_support[0].x() =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW({ const PlanarSupportTracker value(geometry(), config); },
               std::invalid_argument);
  auto input = exactInput(0U, 0.1, 0.1);
  input.channel_velocity_command = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker().track(input).valid);
}
