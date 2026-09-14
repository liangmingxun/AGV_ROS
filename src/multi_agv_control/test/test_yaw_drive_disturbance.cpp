#include <cmath>

#include <gtest/gtest.h>

#include "multi_agv_control/yaw_drive_disturbance.hpp"

namespace mac = multi_agv_control;

TEST(YawDriveDisturbance, DisabledIsExactIdentity) {
  mac::YawDriveDisturbanceConfig config;
  const auto out = mac::evaluateYawDriveDisturbance(config, 2, 2.4, .10, .11);
  EXPECT_FALSE(out.active);
  EXPECT_DOUBLE_EQ(out.signed_disturbance, 0.0);
  EXPECT_DOUBLE_EQ(out.left_disturbed, .10);
  EXPECT_DOUBLE_EQ(out.right_disturbed, .11);
}

TEST(YawDriveDisturbance, OnlyRobot2InsideSpatialWindowIsAffected) {
  mac::YawDriveDisturbanceConfig config;
  config.enabled = true;
  for (int robot : {1, 3}) {
    const auto out = mac::evaluateYawDriveDisturbance(config, robot, 2.2, .1, .1);
    EXPECT_FALSE(out.active);
    EXPECT_DOUBLE_EQ(out.signed_disturbance, 0.0);
  }
  EXPECT_FALSE(mac::evaluateYawDriveDisturbance(config, 2, 1.9, .1, .1).active);
  EXPECT_FALSE(mac::evaluateYawDriveDisturbance(config, 2, 2.9, .1, .1).active);
  EXPECT_TRUE(mac::evaluateYawDriveDisturbance(config, 2, 2.2, .1, .1).active);
}

TEST(YawDriveDisturbance, IsAntisymmetricAndSpatiallyZeroMean) {
  mac::YawDriveDisturbanceConfig config;
  config.enabled = true;
  config.amplitude = .024;
  double integral = 0.0;
  bool positive = false;
  bool negative = false;
  constexpr int samples = 8000;
  constexpr double ds = .8 / samples;
  for (int i = 0; i <= samples; ++i) {
    const double s = 2.0 + i * ds;
    const auto out = mac::evaluateYawDriveDisturbance(config, 2, s, .1, .11);
    EXPECT_NEAR(out.mean_longitudinal_delta, 0.0, 1e-15);
    EXPECT_NEAR(out.yaw_differential_delta,
                2.0 * out.signed_disturbance, 1e-15);
    integral += out.signed_disturbance * ds;
    positive = positive || out.signed_disturbance > 0.0;
    negative = negative || out.signed_disturbance < 0.0;
  }
  EXPECT_TRUE(positive);
  EXPECT_TRUE(negative);
  EXPECT_NEAR(integral, 0.0, 1e-12);
}

TEST(YawDriveDisturbance, RejectsAmplitudeBeyondRound2FakeBound) {
  mac::YawDriveDisturbanceConfig config;
  config.enabled = true;
  config.amplitude = .024001;
  EXPECT_THROW(mac::evaluateYawDriveDisturbance(
      config, 2, 2.2, .1, .1), std::invalid_argument);
}

TEST(YawDriveDisturbance, EntryAndExitAreContinuous) {
  mac::YawDriveDisturbanceConfig config;
  config.enabled = true;
  const auto entry = mac::evaluateYawDriveDisturbance(config, 2, 2.0, .1, .1);
  const auto near_entry = mac::evaluateYawDriveDisturbance(
      config, 2, 2.0 + 1e-7, .1, .1);
  const auto exit = mac::evaluateYawDriveDisturbance(config, 2, 2.8, .1, .1);
  const auto near_exit = mac::evaluateYawDriveDisturbance(
      config, 2, 2.8 - 1e-7, .1, .1);
  EXPECT_DOUBLE_EQ(entry.signed_disturbance, 0.0);
  EXPECT_DOUBLE_EQ(exit.signed_disturbance, 0.0);
  EXPECT_NEAR(near_entry.signed_disturbance, 0.0, 1e-15);
  EXPECT_NEAR(near_exit.signed_disturbance, 0.0, 1e-15);
}
