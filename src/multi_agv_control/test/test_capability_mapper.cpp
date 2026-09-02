#include <array>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/capability_mapper.hpp"

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
