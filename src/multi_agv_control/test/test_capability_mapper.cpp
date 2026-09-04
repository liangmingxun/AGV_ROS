#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "multi_agv_control/capability_mapper.hpp"
#include "multi_agv_control/s_curve_path.hpp"
#include "multi_agv_control/support_geometry.hpp"

using multi_agv_control::CapabilityGeometry;
using multi_agv_control::CapabilityMapper;
using multi_agv_control::CapabilityReserve;
using multi_agv_control::WheelCapability;

namespace {

WheelCapability wheels(double velocity = 0.9, double acceleration = 1.0,
                       double deceleration = 2.0) {
  WheelCapability value;
  value.maximum_velocity_left = velocity;
  value.maximum_velocity_right = velocity;
  value.maximum_acceleration_left = acceleration;
  value.maximum_acceleration_right = acceleration;
  value.maximum_deceleration_left = deceleration;
  value.maximum_deceleration_right = deceleration;
  value.source_stamp = ros::Time(12, 34);
  value.source_sequence = 77U;
  return value;
}

CapabilityGeometry straight() {
  return {1.0, 0.0, 0.114};
}

std::array<double, 4> frozenScaleMinima(double robot2_wheel_limit) {
  multi_agv_control::SCurveConfig path_config;
  path_config.amplitude = 0.05;
  path_config.longitudinal_length = 1.0;
  path_config.lookup_samples = 20001U;
  path_config.model = "sine_single_period";
  const multi_agv_control::SCurvePath path(path_config);
  multi_agv_control::SupportGeometryConfig support_config;
  support_config.offsets = {
      {0.1732050807568877, 0.0},
      {-0.0866025403784439, 0.1500000000000000},
      {-0.0866025403784439, -0.1500000000000000}};
  support_config.minimum_nondegeneracy = 0.20;
  support_config.minimum_speed_scale = 0.20;
  support_config.validation_samples = 10001U;
  const multi_agv_control::SupportGeometry support(path, support_config);
  const std::array<double, 3> wheel_separation{{
      0.135484339, 0.139284482, 0.136802843}};
  const Eigen::Vector2d base_to_support(-0.01783, 0.0);
  const std::array<WheelCapability, 3> capability{{
      wheels(0.15), wheels(robot2_wheel_limit), wheels(0.15)}};
  const CapabilityMapper mapper(CapabilityReserve{});
  double minimum_public = std::numeric_limits<double>::infinity();
  std::array<double, 3> minimum_robot{{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()}};
  for (std::size_t sample = 0; sample <= 10000U; ++sample) {
    const double s0 = path.length() * static_cast<double>(sample) / 10000.0;
    const double step = std::max(path.length() / 10000.0, 1.0e-5);
    const double sm = std::max(0.0, s0 - step);
    const double sp = std::min(path.length(), s0 + step);
    std::array<CapabilityGeometry, 3> geometry;
    for (std::size_t robot = 0; robot < 3U; ++robot) {
      const auto minus = support.sample(robot, sm);
      const auto current = support.sample(robot, s0);
      const auto plus = support.sample(robot, sp);
      const auto chassis_position = [&](const auto& value) {
        return value.position -
            Eigen::Rotation2Dd(value.heading) * base_to_support;
      };
      const Eigen::Vector2d p_minus = chassis_position(minus);
      const Eigen::Vector2d p_current = chassis_position(current);
      const Eigen::Vector2d p_plus = chassis_position(plus);
      const double ds = std::max(sp - sm, 1.0e-9);
      geometry[robot].speed_scale = (p_plus - p_minus).norm() / ds;
      const Eigen::Vector2d tangent_minus = p_current - p_minus;
      const Eigen::Vector2d tangent_plus = p_plus - p_current;
      if (tangent_minus.squaredNorm() > 1.0e-18 &&
          tangent_plus.squaredNorm() > 1.0e-18) {
        const double heading_minus =
            std::atan2(tangent_minus.y(), tangent_minus.x());
        const double heading_plus =
            std::atan2(tangent_plus.y(), tangent_plus.x());
        const double delta = heading_plus - heading_minus;
        geometry[robot].heading_rate =
            std::atan2(std::sin(delta), std::cos(delta)) / (0.5 * ds);
      } else {
        const double delta = plus.heading - minus.heading;
        geometry[robot].heading_rate =
            std::atan2(std::sin(delta), std::cos(delta)) / ds;
      }
      geometry[robot].wheel_separation = wheel_separation[robot];
    }
    const auto mapped = mapper.mapFleet(capability, geometry);
    EXPECT_TRUE(mapped.valid);
    minimum_public = std::min(minimum_public, mapped.public_upper_velocity);
    for (std::size_t robot = 0; robot < 3U; ++robot) {
      minimum_robot[robot] = std::min(
          minimum_robot[robot],
          mapped.robots[robot].actuator_upper_velocity);
    }
  }
  return {{minimum_public, minimum_robot[0], minimum_robot[1],
           minimum_robot[2]}};
}

}  // namespace

TEST(CapabilityMapper, StraightPathEqualsReportedWheelLimit) {
  const CapabilityMapper mapper(CapabilityReserve{});
  const auto result = mapper.map(wheels(), straight());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.upper_velocity, 0.9);
  EXPECT_DOUBLE_EQ(result.lower_velocity, -0.9);
  EXPECT_DOUBLE_EQ(result.available_acceleration, 1.0);
  EXPECT_DOUBLE_EQ(result.available_deceleration, 2.0);
  EXPECT_EQ(result.source_sequence, 77U);
  EXPECT_EQ(result.source_stamp, ros::Time(12, 34));
}

TEST(CapabilityMapper, FormalAvailableWheelLimitEntersPathBoundary) {
  const auto result = CapabilityMapper(CapabilityReserve{}).map(
      wheels(0.08), straight());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.actuator_upper_velocity, 0.08);
  EXPECT_DOUBLE_EQ(result.upper_velocity, 0.08);
}

TEST(CapabilityMapper, PhysicalSpeedScale008MatchesFrozenSPath) {
  const auto nominal = frozenScaleMinima(0.15);
  EXPECT_NEAR(nominal[1], 0.121933, 2e-6);
  EXPECT_NEAR(nominal[2], 0.103626, 2e-6);
  EXPECT_NEAR(nominal[3], 0.103808, 2e-6);
  EXPECT_GT(*std::min_element(nominal.begin() + 1, nominal.end()), 0.090);
  EXPECT_GT(nominal[0] - 0.090, 0.006);

  // Robot2 degrades to gamma=0.68 of the 0.15 m/s nominal wheel limit.
  const auto degraded = frozenScaleMinima(0.102);
  EXPECT_GT(degraded[0], 0.065);
  EXPECT_GT(degraded[0] - 0.055, 0.010);
  EXPECT_LT(degraded[0], 0.080);
}

TEST(CapabilityMapper, CurvatureAndAsymmetricWheelsUseCorrectSide) {
  WheelCapability wheel = wheels();
  wheel.maximum_velocity_left = 0.8;
  wheel.maximum_velocity_right = 0.6;
  const CapabilityGeometry geometry{1.2, 2.0, 0.2};
  const auto result = CapabilityMapper(CapabilityReserve{}).map(wheel, geometry);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.upper_velocity, 0.6 / 1.4, 1e-12);
}

TEST(CapabilityMapper, ReservesClampAtZero) {
  CapabilityReserve reserve;
  reserve.wheel_linear_velocity = 1.0;
  const auto result = CapabilityMapper(reserve).map(wheels(0.5), straight());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.upper_velocity, 0.0);
  EXPECT_DOUBLE_EQ(result.lower_velocity, 0.0);
}

TEST(CapabilityMapper, FleetPublishesMinimumAcrossThree) {
  const std::array<WheelCapability, 3> reports{
      wheels(0.9), wheels(0.4), wheels(0.7)};
  const std::array<CapabilityGeometry, 3> geometry{
      straight(), straight(), straight()};
  const auto fleet = CapabilityMapper(CapabilityReserve{}).mapFleet(
      reports, geometry);
  ASSERT_TRUE(fleet.valid);
  EXPECT_DOUBLE_EQ(fleet.public_upper_velocity, 0.4);
}

TEST(CapabilityMapper, AccelerationReserveIsNotDoubleDeducted) {
  CapabilityReserve baseline;
  CapabilityReserve acceleration_reserved;
  acceleration_reserved.path_acceleration = 0.3;
  const auto a = CapabilityMapper(baseline).map(wheels(), straight());
  const auto b = CapabilityMapper(acceleration_reserved).map(wheels(), straight());
  EXPECT_DOUBLE_EQ(a.upper_velocity, b.upper_velocity);
  EXPECT_NEAR(b.available_acceleration, 0.7, 1e-12);
  EXPECT_DOUBLE_EQ(b.available_deceleration, 2.0);
}

TEST(CapabilityMapper, RejectsInvalidInputsAndConfiguration) {
  CapabilityReserve reserve;
  reserve.heading_rate_epsilon = 0.0;
  EXPECT_THROW({ const CapabilityMapper mapper(reserve); },
               std::invalid_argument);
  auto wheel = wheels();
  wheel.maximum_velocity_left = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(CapabilityMapper(CapabilityReserve{}).map(wheel, straight()).valid);
}
