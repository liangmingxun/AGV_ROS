#include <limits>
#include <gtest/gtest.h>
#include "multi_agv_control/transient_yaw_disturbance.hpp"
namespace mac = multi_agv_control;

TEST(TransientYaw, DisabledAndOtherRobotsAreExactIdentity) {
  mac::TransientYawPulse pulse;
  mac::TransientYawConfig cfg;
  auto v = pulse.evaluate(cfg, 2, 3., 10., .1, .12);
  EXPECT_FALSE(v.triggered);
  EXPECT_DOUBLE_EQ(v.left_disturbed, .1);
  cfg.enabled = true;
  v = pulse.evaluate(cfg, 1, 3., 10., .1, .12);
  EXPECT_FALSE(v.triggered);
  EXPECT_DOUBLE_EQ(v.right_disturbed, .12);
}

TEST(TransientYaw, TriggerIsActualProgressAndEndIsFixedWallClock) {
  mac::TransientYawPulse pulse;
  mac::TransientYawConfig cfg; cfg.enabled = true;
  EXPECT_FALSE(pulse.evaluate(cfg, 2, 1.999, 10., .1, .1).triggered);
  auto v = pulse.evaluate(cfg, 2, 2., 11., .1, .1);
  EXPECT_TRUE(v.triggered); EXPECT_DOUBLE_EQ(v.disturbance, 0.);
  v = pulse.evaluate(cfg, 2, 1.9, 11.75, .1, .1);
  EXPECT_DOUBLE_EQ(v.disturbance, .024);
  v = pulse.evaluate(cfg, 2, 2.001, 12.5, .1, .1);
  EXPECT_TRUE(v.finished); EXPECT_DOUBLE_EQ(v.disturbance, 0.);
  v = pulse.evaluate(cfg, 2, 20., 50., .1, .1);
  EXPECT_TRUE(v.finished); EXPECT_DOUBLE_EQ(v.trigger_wall_time, 11.);
  EXPECT_DOUBLE_EQ(v.left_disturbed, .1);
}

TEST(TransientYaw, PulseHasOneSignAndZeroLongitudinalInjection) {
  mac::TransientYawPulse pulse;
  mac::TransientYawConfig cfg; cfg.enabled = true;
  for (int k = 0; k <= 2000; ++k) {
    const auto v = pulse.evaluate(cfg, 2, 2., 10. + k * .001, .10, .11);
    EXPECT_GE(v.disturbance, 0.); EXPECT_LE(v.disturbance, cfg.amplitude);
    EXPECT_NEAR((v.left_disturbed-.10)+(v.right_disturbed-.11), 0., 1e-15);
    if (k >= 1500) { EXPECT_DOUBLE_EQ(v.disturbance, 0.); }
  }
}

TEST(TransientYaw, EntryPeakAndExitAreSmooth) {
  mac::TransientYawPulse pulse;
  mac::TransientYawConfig cfg; cfg.enabled = true;
  auto v = pulse.evaluate(cfg, 2, 2., 10., .1, .1);
  EXPECT_DOUBLE_EQ(v.disturbance, 0.);
  EXPECT_LT(pulse.evaluate(cfg, 2, 2., 10.00001, .1, .1).disturbance, 1e-12);
  const double near_peak = pulse.evaluate(cfg, 2, 2., 10.74999, .1, .1).disturbance;
  EXPECT_NEAR(near_peak, .024, 1e-12);
  EXPECT_NEAR(pulse.evaluate(cfg, 2, 2., 10.75001, .1, .1).disturbance, near_peak, 1e-12);
  EXPECT_LT(pulse.evaluate(cfg, 2, 2., 11.49999, .1, .1).disturbance, 1e-12);
  EXPECT_DOUBLE_EQ(pulse.evaluate(cfg, 2, 2., 11.5, .1, .1).disturbance, 0.);
}

TEST(TransientYaw, OnlyTwoFrozenCandidatesAreAllowed) {
  mac::TransientYawConfig cfg; cfg.enabled = true;
  EXPECT_NO_THROW(mac::validateTransientYawConfig(cfg));
  cfg.amplitude = .022; cfg.duration = 2.;
  EXPECT_NO_THROW(mac::validateTransientYawConfig(cfg));
  cfg.amplitude = .023;
  EXPECT_THROW(mac::validateTransientYawConfig(cfg), std::invalid_argument);
  cfg.amplitude = .024; cfg.duration = 2.;
  EXPECT_THROW(mac::validateTransientYawConfig(cfg), std::invalid_argument);
}

TEST(TransientYaw, NonfiniteOrBackwardTimeFailsClosed) {
  mac::TransientYawPulse pulse;
  mac::TransientYawConfig cfg; cfg.enabled = true;
  pulse.evaluate(cfg, 2, 2., 10., .1, .1);
  EXPECT_THROW(pulse.evaluate(cfg, 2, 2., 9., .1, .1), std::invalid_argument);
  EXPECT_THROW(pulse.evaluate(cfg, 2, 2., 11., std::numeric_limits<double>::quiet_NaN(), .1), std::invalid_argument);
}

TEST(TransientYaw, ExplorationCandidatesRequireExplicitOptIn) {
  mac::TransientYawConfig cfg; cfg.enabled = true;
  for (const auto pair : {std::pair<double,double>{.022,2.5}, {.022,3.0}, {.024,2.5}, {.026,2.5}}) {
    cfg.amplitude = pair.first; cfg.duration = pair.second;
    cfg.effect_exploration = false;
    EXPECT_THROW(mac::validateTransientYawConfig(cfg), std::invalid_argument);
    cfg.effect_exploration = true;
    EXPECT_NO_THROW(mac::validateTransientYawConfig(cfg));
  }
  cfg.amplitude = .028;
  EXPECT_THROW(mac::validateTransientYawConfig(cfg), std::invalid_argument);
}
