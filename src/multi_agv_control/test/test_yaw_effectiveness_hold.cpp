#include <gtest/gtest.h>
#include "multi_agv_control/yaw_effectiveness_hold_degradation.hpp"
using namespace multi_agv_control;
TEST(YawHold, DisabledProfileNeverLatches) {
  YawHoldConfig c;YawHoldDegradation m;
  EXPECT_FALSE(m.evaluate(c,2,5.,100.,.12,.08).triggered);
}
TEST(YawHold, BelowTriggerIsIdentity) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;
  EXPECT_DOUBLE_EQ(m.evaluate(c,2,1.999,0.,.12,.08).gamma,1.);
}
TEST(YawHold, TriggerStartsAtIdentity) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;
  auto x=m.evaluate(c,2,2.,5.,.12,.08);
  EXPECT_TRUE(x.triggered);EXPECT_DOUBLE_EQ(x.gamma,1.);
}
TEST(YawHold, DownEndpointMatchesHold) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;m.evaluate(c,2,2.,0.,.12,.08);
  EXPECT_DOUBLE_EQ(m.evaluate(c,2,2.,.6,.12,.08).gamma,.2);
}
TEST(YawHold, EntireHoldIsConstant) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;m.evaluate(c,2,2.,0.,.12,.08);
  for(double t:{.6,1.,2.,3.1}) EXPECT_DOUBLE_EQ(m.evaluate(c,2,2.,t,.12,.08).gamma,.2);
}
TEST(YawHold, UpEndpointRestoresIdentity) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;m.evaluate(c,2,2.,0.,.12,.08);
  auto x=m.evaluate(c,2,2.,3.7,.12,.08);EXPECT_TRUE(x.finished);EXPECT_DOUBLE_EQ(x.left_degraded,.12);
}
TEST(YawHold, EndCannotRetrigger) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;m.evaluate(c,2,2.,0.,.12,.08);
  m.evaluate(c,2,2.,4.,.12,.08);EXPECT_TRUE(m.evaluate(c,2,4.,5.,.12,.08).finished);
}
TEST(YawHold, ProgressRegressionDoesNotPauseWallPulse) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;m.evaluate(c,2,2.,0.,.12,.08);
  EXPECT_DOUBLE_EQ(m.evaluate(c,2,0.,1.,.12,.08).gamma,.2);
}
TEST(YawHold, InvalidRobotIsRejected) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;
  EXPECT_THROW(m.evaluate(c,0,2.,0.,.12,.08),std::invalid_argument);
}
TEST(YawHold, NonfiniteWallIsRejected) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;
  EXPECT_THROW(m.evaluate(c,2,2.,std::numeric_limits<double>::quiet_NaN(),.12,.08),std::invalid_argument);
}
TEST(YawHold, OldBellProfileRemainsIndependent) {
  YawEffectivenessConfig c;c.enabled=true;c.gamma_min=.12;c.duration=3.5;
  YawEffectivenessDegradation m;m.evaluate(c,2,2.,0.,.12,.08);
  EXPECT_NEAR(m.evaluate(c,2,2.,1.75,.12,.08).gamma,.12,1e-12);
}
TEST(YawHold, ProfileAndWallClockOneShot) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation model;
  auto run=[&](double t,double s=2.) {return model.evaluate(c,2,s,10.+t,.14,.06);};
  EXPECT_DOUBLE_EQ(run(-.1,1.9).gamma,1.);
  EXPECT_DOUBLE_EQ(run(0.).gamma,1.);
  EXPECT_NEAR(run(.3).gamma,.6,1e-12);
  EXPECT_DOUBLE_EQ(run(.6).gamma,.2);
  EXPECT_DOUBLE_EQ(run(1.5).gamma,.2);
  EXPECT_DOUBLE_EQ(run(3.1).gamma,.2);
  EXPECT_NEAR(run(3.4).gamma,.6,1e-12);
  EXPECT_DOUBLE_EQ(run(3.7,1.8).gamma,1.);
  EXPECT_TRUE(run(4.,1.8).finished);
  EXPECT_DOUBLE_EQ(run(9.,3.).gamma,1.);
}
TEST(YawHold, MeanDifferentialAndOtherRobots) {
  YawHoldConfig c;c.enabled=true;YawHoldDegradation m;
  for (int i=0;i<400;++i) {
    auto out=m.evaluate(c,2,2.,i*.01,.145,.067);
    EXPECT_NEAR(.5*(out.left_degraded+out.right_degraded),.106,3e-17);
    EXPECT_NEAR(out.right_degraded-out.left_degraded,out.gamma*(.067-.145),4e-17);
  }
  for (int robot:{1,3}) {
    YawHoldDegradation x;auto out=x.evaluate(c,robot,3.,0.,.14,.06);
    EXPECT_DOUBLE_EQ(out.left_degraded,.14);EXPECT_DOUBLE_EQ(out.right_degraded,.06);
    EXPECT_FALSE(out.triggered);
  }
}
TEST(YawHold, SmoothBoundaries) {
  YawHoldConfig c;c.enabled=true;
  for (double boundary:{.6,3.1,3.7}) {
    YawHoldDegradation m;m.evaluate(c,2,2.,0.,.14,.06);
    auto a=m.evaluate(c,2,2.,boundary-1e-5,.14,.06);
    auto b=m.evaluate(c,2,2.,boundary,.14,.06);
    auto d=m.evaluate(c,2,2.,boundary+1e-5,.14,.06);
    EXPECT_NEAR(a.gamma,b.gamma,1e-12);EXPECT_NEAR(d.gamma,b.gamma,1e-12);
  }
}
TEST(YawHold, DisabledIdentityAndNonfiniteSafety) {
  YawHoldConfig c;YawHoldDegradation m;
  EXPECT_DOUBLE_EQ(m.evaluate(c,2,2.,1.,.14,.06).left_degraded,.14);
  c.enabled=true;
  EXPECT_THROW(m.evaluate(c,2,2.,1.,std::numeric_limits<double>::infinity(),.06),std::invalid_argument);
  EXPECT_THROW(m.evaluate(c,2,std::numeric_limits<double>::quiet_NaN(),1.,.14,.06),std::invalid_argument);
  m.evaluate(c,2,2.,2.,.14,.06);
  EXPECT_THROW(m.evaluate(c,2,2.,1.,.14,.06),std::invalid_argument);
  c.hold=3.;EXPECT_THROW(validateYawHoldConfig(c),std::invalid_argument);
}
