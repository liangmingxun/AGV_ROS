#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "multi_agv_control/candidate_b_effectiveness_disturbance.hpp"

namespace multi_agv_control {
namespace {

TEST(CandidateB, IdentityWhenDisabled) {
  CandidateBConfig c;
  CandidateBEffectivenessDisturbance d;
  const auto out = d.evaluate(c, 2, 3.0, 1.0, .12, .08);
  EXPECT_DOUBLE_EQ(out.left_post_disturbance, .12);
  EXPECT_DOUBLE_EQ(out.right_post_disturbance, .08);
  EXPECT_DOUBLE_EQ(out.effectiveness, 1.0);
}

TEST(CandidateB, ScalesOnlyLongitudinalAndAddsYaw) {
  CandidateBConfig c;
  c.enabled = true;
  CandidateBEffectivenessDisturbance d;
  d.evaluate(c, 2, 2.0, 10.0, .12, .08);
  const auto out = d.evaluate(c, 2, 2.1, 11.0, .12, .08);
  EXPECT_DOUBLE_EQ(out.envelope, 1.0);
  EXPECT_DOUBLE_EQ(out.effectiveness, .85);
  EXPECT_NEAR(out.longitudinal_after_effectiveness, .085, 1e-12);
  EXPECT_NEAR(out.yaw_native, (.08 - .12) / c.wheel_separation, 1e-12);
  EXPECT_NEAR(out.yaw_after_disturbance,
              out.yaw_native + .2625 * std::sin(1.0 + .5 * std::acos(-1.0)),
              1e-12);
  EXPECT_NEAR((out.right_post_disturbance - out.left_post_disturbance) /
                  c.wheel_separation,
              out.yaw_after_disturbance, 1e-12);
}

TEST(CandidateB, QuinticRampAndExactRecovery) {
  CandidateBConfig c;
  c.enabled = true;
  CandidateBEffectivenessDisturbance d;
  d.evaluate(c, 2, 2.0, 1.0, .1, .1);
  const auto ramp = d.evaluate(c, 2, 2.0, 1.25, .1, .1);
  EXPECT_NEAR(ramp.envelope, .5, 1e-12);
  EXPECT_NEAR(ramp.effectiveness, .925, 1e-12);
  const auto out = d.evaluate(c, 2, 2.0, 8.75, .1, .1);
  EXPECT_NEAR(out.envelope, .5, 1e-12);
  const auto done = d.evaluate(c, 2, 5.0, 9.0, .1, .1);
  EXPECT_TRUE(done.finished);
  EXPECT_DOUBLE_EQ(done.effectiveness, 1.0);
  EXPECT_DOUBLE_EQ(done.left_post_disturbance, .1);
  EXPECT_DOUBLE_EQ(done.right_post_disturbance, .1);
}

TEST(CandidateB, OtherRobotsUnaffectedAndProfileFrozen) {
  CandidateBConfig c;
  c.enabled = true;
  CandidateBEffectivenessDisturbance d;
  const auto other = d.evaluate(c, 1, 3.0, 1.0, .12, .08);
  EXPECT_FALSE(other.triggered);
  EXPECT_DOUBLE_EQ(other.left_post_disturbance, .12);
  c.minimum_effectiveness = .90;
  EXPECT_THROW(validateCandidateBConfig(c), std::invalid_argument);
}

}  // namespace
}  // namespace multi_agv_control
