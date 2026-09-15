#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "multi_agv_control/classic_additive_disturbance.hpp"

namespace multi_agv_control {
namespace {

TEST(ClassicAdditiveDisturbance, FrozenProfileAndOneShotWallClock) {
  ClassicAdditiveConfig c;
  c.enabled = true;
  ClassicAdditiveDisturbance d;
  auto before = d.evaluate(c, 2, 1.9, 10.0, .11, .09);
  EXPECT_FALSE(before.triggered);
  EXPECT_DOUBLE_EQ(before.left_disturbed, .11);
  auto start = d.evaluate(c, 2, 2.0, 11.0, .11, .09);
  EXPECT_TRUE(start.triggered);
  EXPECT_DOUBLE_EQ(start.envelope, 0.0);
  auto ramp = d.evaluate(c, 2, 1.0, 11.25, .11, .09);
  EXPECT_NEAR(ramp.envelope, .5, 1e-12);
  auto hold = d.evaluate(c, 2, 0.0, 12.0, .11, .09);
  EXPECT_DOUBLE_EQ(hold.envelope, 1.0);
  EXPECT_NEAR(hold.linear_disturbance, .030 * std::sin(1.0), 1e-12);
  EXPECT_NEAR(hold.angular_disturbance,
              .350 * std::sin(1.0 + .5 * std::acos(-1.0)), 1e-12);
  auto ramp_out = d.evaluate(c, 2, 0.0, 18.75, .11, .09);
  EXPECT_NEAR(ramp_out.envelope, .5, 1e-12);
  auto done = d.evaluate(c, 2, 3.0, 19.0, .11, .09);
  EXPECT_TRUE(done.finished);
  EXPECT_FALSE(done.active);
  EXPECT_DOUBLE_EQ(done.left_disturbed, .11);
  EXPECT_DOUBLE_EQ(done.right_disturbed, .09);
  auto no_retrigger = d.evaluate(c, 2, 5.0, 20.0, .11, .09);
  EXPECT_TRUE(no_retrigger.finished);
}

TEST(ClassicAdditiveDisturbance, MappingPreservesSpecifiedComponents) {
  ClassicAdditiveConfig c;
  c.enabled = true;
  ClassicAdditiveDisturbance d;
  d.evaluate(c, 2, 2.0, 3.0, .12, .08);
  auto out = d.evaluate(c, 2, 2.0, 4.0, .12, .08);
  EXPECT_NEAR(.5 * ((out.left_disturbed + out.right_disturbed) - .20),
              out.linear_disturbance, 1e-12);
  EXPECT_NEAR(((out.right_disturbed - out.left_disturbed) - (.08 - .12)) /
                  c.wheel_separation,
              out.angular_disturbance, 1e-12);
}

TEST(ClassicAdditiveDisturbance, OtherRobotsUnchangedAndFiniteChecksFailClosed) {
  ClassicAdditiveConfig c;
  c.enabled = true;
  ClassicAdditiveDisturbance d;
  auto other = d.evaluate(c, 1, 3.0, 1.0, .12, .08);
  EXPECT_FALSE(other.triggered);
  EXPECT_DOUBLE_EQ(other.left_disturbed, .12);
  EXPECT_DOUBLE_EQ(other.right_disturbed, .08);
  EXPECT_THROW(d.evaluate(c, 2, 3.0, 2.0,
                          std::numeric_limits<double>::quiet_NaN(), .08),
               std::invalid_argument);
}

TEST(ClassicAdditiveDisturbance, RejectsAnyParameterSearch) {
  ClassicAdditiveConfig c;
  c.enabled = true;
  c.angular_amplitude = .45;
  EXPECT_THROW(validateClassicAdditiveConfig(c), std::invalid_argument);
}

}  // namespace
}  // namespace multi_agv_control
