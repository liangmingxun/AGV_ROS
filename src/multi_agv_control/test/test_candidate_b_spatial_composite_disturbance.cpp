#include <gtest/gtest.h>

#include <cmath>

#include "multi_agv_control/candidate_b_spatial_composite_disturbance.hpp"

namespace multi_agv_control {
namespace {

CandidateBSpatialCompositeConfig config() {
  CandidateBSpatialCompositeConfig c;
  c.enabled = true;
  c.wheel_separation = {{0.14, 0.16, 0.18}};
  return c;
}

TEST(CandidateBSpatialComposite, IdentityOutsideZone) {
  auto c = config();
  CandidateBSpatialCompositeDisturbance d;
  const auto before = d.evaluate(c, 1, 1.9, 10.0, 0.08, 0.12);
  EXPECT_DOUBLE_EQ(before.envelope, 0.0);
  EXPECT_DOUBLE_EQ(before.effectiveness, 1.0);
  EXPECT_DOUBLE_EQ(before.longitudinal_disturbance, 0.0);
  EXPECT_DOUBLE_EQ(before.yaw_disturbance, 0.0);
  EXPECT_DOUBLE_EQ(before.left_post_disturbance, 0.08);
  EXPECT_DOUBLE_EQ(before.right_post_disturbance, 0.12);
  const auto after = d.evaluate(c, 1, 2.81, 20.0, 0.07, 0.11);
  EXPECT_TRUE(after.finished);
  EXPECT_DOUBLE_EQ(after.envelope, 0.0);
  EXPECT_DOUBLE_EQ(after.left_post_disturbance, 0.07);
  EXPECT_DOUBLE_EQ(after.right_post_disturbance, 0.11);
}

TEST(CandidateBSpatialComposite, SpatialRampAndFrozenCenter) {
  auto c = config();
  CandidateBSpatialCompositeDisturbance d;
  const auto ramp_in = d.evaluate(c, 1, 2.025, 1.0, 0.1, 0.1);
  EXPECT_NEAR(ramp_in.envelope, 0.5, 1e-12);
  const auto center = d.evaluate(c, 1, 2.40, 2.0, 0.1, 0.1);
  EXPECT_DOUBLE_EQ(center.envelope, 1.0);
  EXPECT_NEAR(center.effectiveness, 0.85, 1e-12);
  const auto ramp_out = d.evaluate(c, 1, 2.775, 3.0, 0.1, 0.1);
  EXPECT_NEAR(ramp_out.envelope, 0.5, 1e-12);
}

TEST(CandidateBSpatialComposite, LongitudinalAndYawChannelsAreDecoupled) {
  auto c = config();
  CandidateBSpatialCompositeDisturbance d;
  // Entry at tau=0 has d_v=0 and d_omega=A because phases are 0 and pi/2.
  d.evaluate(c, 2, 2.0, 5.0, 0.08, 0.12);
  const auto out = d.evaluate(c, 2, 2.40, 5.0, 0.08, 0.12);
  const double native_yaw = (0.12 - 0.08) / 0.16;
  EXPECT_NEAR(out.longitudinal_after_effectiveness, 0.85 * 0.10, 1e-12);
  EXPECT_NEAR(out.longitudinal_after_composite,
              0.85 * 0.10 + out.longitudinal_disturbance, 1e-12);
  EXPECT_NEAR(out.yaw_native, native_yaw, 1e-12);
  EXPECT_NEAR(out.yaw_after_disturbance,
              native_yaw + out.yaw_disturbance, 1e-12);
}

TEST(CandidateBSpatialComposite, UsesPerRobotTrackAndIndependentState) {
  auto c = config();
  CandidateBSpatialCompositeDisturbance robot1;
  CandidateBSpatialCompositeDisturbance robot2;
  CandidateBSpatialCompositeDisturbance robot3;
  const auto one = robot1.evaluate(c, 1, 2.2, 10.0, 0.08, 0.12);
  const auto two = robot2.evaluate(c, 2, 1.9, 10.0, 0.08, 0.12);
  const auto three = robot3.evaluate(c, 3, 1.9, 10.0, 0.08, 0.12);
  EXPECT_TRUE(one.triggered);
  EXPECT_FALSE(two.triggered);
  EXPECT_FALSE(three.triggered);
  EXPECT_NEAR(one.yaw_native, 0.04 / 0.14, 1e-12);
  EXPECT_NEAR(two.yaw_native, 0.04 / 0.16, 1e-12);
  EXPECT_NEAR(three.yaw_native, 0.04 / 0.18, 1e-12);
}

TEST(CandidateBSpatialComposite, NoFixedTimeTermination) {
  auto c = config();
  CandidateBSpatialCompositeDisturbance d;
  d.evaluate(c, 1, 2.0, 0.0, 0.1, 0.1);
  const auto long_exposure = d.evaluate(c, 1, 2.4, 100.0, 0.1, 0.1);
  EXPECT_TRUE(long_exposure.active);
  EXPECT_FALSE(long_exposure.finished);
  const auto exited = d.evaluate(c, 1, 2.8, 101.0, 0.1, 0.1);
  EXPECT_TRUE(exited.finished);
}

}  // namespace
}  // namespace multi_agv_control
