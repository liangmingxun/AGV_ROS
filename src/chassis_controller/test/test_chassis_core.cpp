#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>

#include "chassis_controller/chassis_core.hpp"

using namespace chassis_controller;

namespace {
ChassisConfig testConfig() {
  ChassisConfig config;
  config.robot_index = 2;
  config.wheel_separation = 0.114;
  config.nominal_limits = {0.9, 0.9, 1.0, 1.0, 2.0, 2.0};
  config.control_loop_overrun_seconds = 0.01;
  return config;
}
}

TEST(ChassisCore, LinksRawAppliedAndCapability) {
  ChassisCore core(testConfig());
  EXPECT_TRUE(core.acceptCommand({2, 7, {0.8, 0.8}}));
  core.step(0.01);
  EXPECT_EQ(core.feedback().command_seq_applied, 7u);
  EXPECT_LE(std::abs(core.feedback().applied.left),
            core.capability().limits.max_velocity_left);
}

TEST(ChassisCore, RejectsWrongRobotAndOldCommand) {
  ChassisCore core(testConfig());
  EXPECT_FALSE(core.acceptCommand({1, 1, {0.1, 0.1}}));
  EXPECT_TRUE(core.acceptCommand({2, 2, {0.2, 0.2}}));
  EXPECT_FALSE(core.acceptCommand({2, 2, {0.4, 0.4}}));
  EXPECT_FALSE(core.acceptCommand({2, 1, {0.4, 0.4}}));
}

TEST(ChassisCore, DeratingChangesReportedAndExecutedCapability) {
  ChassisCore core(testConfig());
  DeratingInput input;
  input.robot_id = 2;
  input.sequence = 1;
  input.active = true;
  input.mode = 3;
  input.ratios = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  input.ramp_down_seconds = 0.0;
  ASSERT_TRUE(core.acceptDerating(input));
  core.step(0.004);
  EXPECT_DOUBLE_EQ(core.capability().limits.max_velocity_left, 0.45);
  EXPECT_DOUBLE_EQ(core.capability().limits.max_acceleration_right, 0.8);
  EXPECT_TRUE(core.capability().derating_active);
  EXPECT_EQ(core.capability().derating_mode, 3);

  EXPECT_TRUE(core.acceptDerating(input));
  EXPECT_EQ(core.capability().local_revision, 1u);
  input.ratios.speed_left = 0.4;
  EXPECT_FALSE(core.acceptDerating(input));
}

TEST(ChassisCore, ConvertsMillimetresPerSecondAndReportsOverrun) {
  ChassisCore core(testConfig());
  SensorInput sensor;
  sensor.wheel_left_mm_per_second = 200.0;
  sensor.wheel_right_mm_per_second = 400.0;
  core.updateSensors(sensor);
  core.step(0.02);
  EXPECT_NEAR(core.feedback().actual.left, 0.2, 1e-12);
  EXPECT_NEAR(core.feedback().actual.right, 0.4, 1e-12);
  EXPECT_TRUE(core.feedback().control_loop_overrun);
}

TEST(ChassisCore, ResetOdometryClearsPoseAndVelocity) {
  ChassisCore core(testConfig());
  SensorInput sensor;
  sensor.wheel_left_mm_per_second = 200.0;
  sensor.wheel_right_mm_per_second = 200.0;
  core.updateSensors(sensor);
  core.step(1.0);
  ASSERT_GT(core.feedback().odometry.x, 0.1);
  core.resetOdometry();
  EXPECT_DOUBLE_EQ(core.feedback().odometry.x, 0.0);
  EXPECT_DOUBLE_EQ(core.feedback().odometry.y, 0.0);
  EXPECT_DOUBLE_EQ(core.feedback().odometry.yaw, 0.0);
  EXPECT_DOUBLE_EQ(core.feedback().linear_velocity, 0.0);
}

TEST(ChassisCore, RejectsUnsafeConfiguration) {
  auto config = testConfig();
  config.nominal_limits.max_velocity_left = 0.0;
  EXPECT_THROW(ChassisCore core(config), std::invalid_argument);
  config = testConfig();
  config.robot_index = 0;
  EXPECT_THROW(ChassisCore core(config), std::invalid_argument);
}
