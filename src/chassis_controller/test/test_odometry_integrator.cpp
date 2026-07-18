#include <cmath>
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
