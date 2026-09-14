#include <gtest/gtest.h>
#include "multi_agv_control/yaw_effectiveness_degradation.hpp"
namespace mac=multi_agv_control;
TEST(YawEffectiveness, DisabledIdentityAndOtherRobots) {
  mac::YawEffectivenessDegradation model; mac::YawEffectivenessConfig c;
  auto v=model.evaluate(c,2,3.,10.,.1,.13);
  EXPECT_DOUBLE_EQ(v.gamma,1.); EXPECT_DOUBLE_EQ(v.left_degraded,.1);
  c.enabled=true;
  for (int i : {1,3}) {
    v=model.evaluate(c,i,3.,10.,.1,.13);
    EXPECT_FALSE(v.triggered); EXPECT_DOUBLE_EQ(v.right_degraded,.13);
  }
}
TEST(YawEffectiveness, MeanPreservedAndDifferentialScaled) {
  mac::YawEffectivenessDegradation model; mac::YawEffectivenessConfig c;c.enabled=true;
  model.evaluate(c,2,2.,10.,.10,.14);
  for (int i=0;i<=3500;++i) {
    const auto v=model.evaluate(c,2,2.,10.+i*.001,.10,.14);
    EXPECT_NEAR((v.left_degraded+v.right_degraded)/2.,.12,1e-15);
    EXPECT_NEAR((v.right_degraded-v.left_degraded)/2.,v.gamma*.02,1e-15);
    EXPECT_GE(v.gamma,c.gamma_min);EXPECT_LE(v.gamma,1.);
  }
}
TEST(YawEffectiveness, TriggerMidpointEndAndNoRetrigger) {
  mac::YawEffectivenessDegradation model;mac::YawEffectivenessConfig c;c.enabled=true;
  EXPECT_DOUBLE_EQ(model.evaluate(c,2,1.99,10.,.1,.14).gamma,1.);
  EXPECT_DOUBLE_EQ(model.evaluate(c,2,2.,11.,.1,.14).gamma,1.);
  EXPECT_DOUBLE_EQ(model.evaluate(c,2,1.8,12.75,.1,.14).gamma,c.gamma_min);
  auto v=model.evaluate(c,2,1.8,14.5,.1,.14);
  EXPECT_TRUE(v.finished);EXPECT_DOUBLE_EQ(v.gamma,1.);EXPECT_DOUBLE_EQ(v.left_degraded,.1);
  v=model.evaluate(c,2,20.,50.,.1,.14);
  EXPECT_DOUBLE_EQ(v.trigger_wall_time,11.);EXPECT_TRUE(v.finished);
}
TEST(YawEffectiveness, SmoothEdgesAndFiniteClockSafety) {
  mac::YawEffectivenessDegradation model;mac::YawEffectivenessConfig c;c.enabled=true;
  model.evaluate(c,2,2.,10.,.1,.14);
  EXPECT_NEAR(model.evaluate(c,2,2.,10.00001,.1,.14).gamma,1.,1e-12);
  EXPECT_NEAR(model.evaluate(c,2,2.,13.49999,.1,.14).gamma,1.,1e-12);
  EXPECT_THROW(model.evaluate(c,2,2.,9.,.1,.14),std::invalid_argument);
  EXPECT_THROW(model.evaluate(c,2,2.,14.,std::numeric_limits<double>::infinity(),.14),std::invalid_argument);
  EXPECT_THROW(model.evaluate(c,2,std::numeric_limits<double>::quiet_NaN(),14.,.1,.14),std::invalid_argument);
}
TEST(YawEffectiveness, OnlyAuthorizedExplorationCandidates) {
  mac::YawEffectivenessConfig c;c.enabled=true;
  for (double gamma : {.12,.10,.08}) {c.gamma_min=gamma;EXPECT_NO_THROW(mac::validateYawEffectivenessConfig(c));}
  c.gamma_min=.07;EXPECT_THROW(mac::validateYawEffectivenessConfig(c),std::invalid_argument);
  c.gamma_min=.12;c.duration=4.5;EXPECT_THROW(mac::validateYawEffectivenessConfig(c),std::invalid_argument);
}
