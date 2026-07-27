#include <cmath>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>

#include "chassis_controller/odometry_integrator.hpp"

using chassis_controller::OdometryIntegrator;
using chassis_controller::Pose2DState;

TEST(OdometryIntegrator, IntegratesStraightMotion) {
  OdometryIntegrator odom;
  ASSERT_TRUE(odom.update(0.2, 0.2, 0.0, 1.0));
  EXPECT_NEAR(odom.state().x, 0.2, 1e-12);
  EXPECT_NEAR(odom.state().y, 0.0, 1e-12);
}

TEST(OdometryIntegrator, UsesMidpointHeadingForTurn) {
  OdometryIntegrator odom;
  ASSERT_TRUE(odom.update(1.0, 1.0, 1.0, 1.0));
  EXPECT_NEAR(odom.state().x, std::cos(0.5), 1e-12);
  EXPECT_NEAR(odom.state().y, std::sin(0.5), 1e-12);
  EXPECT_NEAR(odom.state().yaw, 1.0, 1e-12);
}

TEST(OdometryIntegrator, FreezesGyroYawWhileDriveWheelsAreStationary) {
  OdometryIntegrator odom(0.005);
  ASSERT_TRUE(odom.update(0.004, -0.004, 0.2, 2.0));
  EXPECT_DOUBLE_EQ(odom.state().yaw, 0.0);
  EXPECT_DOUBLE_EQ(odom.yawRate(), 0.0);

  ASSERT_TRUE(odom.update(0.006, -0.006, 0.2, 1.0));
  EXPECT_NEAR(odom.state().yaw, 0.2, 1e-12);
  EXPECT_NEAR(odom.yawRate(), 0.2, 1e-12);
}

TEST(OdometryIntegrator, RejectsInvalidStationaryTolerance) {
  EXPECT_THROW(OdometryIntegrator(-0.001), std::invalid_argument);
  EXPECT_THROW(OdometryIntegrator(
                   std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(OdometryIntegrator, ResetUsesRequestedPoseAndYawWraps) {
  OdometryIntegrator odom;
  odom.reset({1.0, 2.0, 3.0});
  ASSERT_TRUE(odom.update(0.0, 0.0, 1.0, 1.0));
  EXPECT_NEAR(odom.state().x, 1.0, 1e-12);
  EXPECT_NEAR(odom.state().y, 2.0, 1e-12);
  EXPECT_GT(odom.state().yaw, -3.14159265358979323846);
  EXPECT_LE(odom.state().yaw, 3.14159265358979323846);
}

TEST(OdometryIntegrator, InvalidInputDoesNotMutateState) {
  OdometryIntegrator odom;
  EXPECT_FALSE(odom.update(0.2, 0.2, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(odom.state().x, 0.0);
  EXPECT_FALSE(odom.update(0.2, 0.2, 0.0, -0.1));
}
