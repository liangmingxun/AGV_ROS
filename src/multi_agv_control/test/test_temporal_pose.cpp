#include <gtest/gtest.h>
#include <limits>
#include "multi_agv_control/temporal_pose.hpp"
#include "multi_agv_control/recovery_slew.hpp"
#include "multi_agv_control/startup_tracking.hpp"
namespace m = multi_agv_control;
TEST(StartupTracking, EarlyRiseIsMonotoneSmoothAndHasConsistentDerivative) {
  EXPECT_DOUBLE_EQ(m::startupRampScale(0,.6),0);
  EXPECT_DOUBLE_EQ(m::startupRampScale(1,.6),1);
  EXPECT_DOUBLE_EQ(m::startupRampRate(0,.6,4),0);
  EXPECT_DOUBLE_EQ(m::startupRampRate(1,.6,4),0);
  EXPECT_GT(m::startupRampScale(.125,.6),m::startupSmoothStep(.125));
  double previous=0, peak=0;
  for(int i=1;i<1000;++i) {
    const double tau=i/1000.0, eps=1e-6;
    const double scale=m::startupRampScale(tau,.6);
    EXPECT_GE(scale,previous); EXPECT_LE(scale,1); previous=scale;
    EXPECT_DOUBLE_EQ(m::startupRampScale(tau,0),m::startupSmoothStep(tau));
    const double derivative=(m::startupRampScale(tau+eps,.6)-m::startupRampScale(tau-eps,.6))/(2*eps*4);
    EXPECT_NEAR(m::startupRampRate(tau,.6,4),derivative,1e-8);
    peak=std::max(peak,m::startupRampRate(tau,.6,4)*.1);
  }
  EXPECT_LT(peak,.034);
  EXPECT_NEAR(-.1*m::startupRampScale(.5,.6),-.05,1e-12);
}
TEST(TemporalPose, MeasuredTwistPropagationKeepsCurvedAndSignedMotion) {
  auto p=m::propagateMeasuredTwist({1,2,0},.1,0,0,.12);
  EXPECT_NEAR(p.x,1.012,1e-12); EXPECT_DOUBLE_EQ(p.y,2);
  p=m::propagateMeasuredTwist({0,0,0},.1,0,1,.1);
  EXPECT_NEAR(p.x,.1*std::sin(.1),1e-12);
  EXPECT_NEAR(p.y,.1*(1-std::cos(.1)),1e-12);
  p=m::propagateMeasuredTwist({0,0,0},-.1,0,0,.1);
  EXPECT_NEAR(p.x,-.01,1e-12);
}
TEST(StartupTracking, CandidateIsContinuousAndKeepsLegacyDefault) {
  EXPECT_DOUBLE_EQ(m::startupVelocityEnvelope(.04,.12,.4,.4,0),.04);
  EXPECT_DOUBLE_EQ(m::startupVelocityEnvelope(0,.12,0,0,.008),0);
  EXPECT_DOUBLE_EQ(m::startupFeedbackWeight(0,1.6,0),0);
  EXPECT_DOUBLE_EQ(m::startupFeedbackWeight(1.6,1.6,.5),1);
  EXPECT_DOUBLE_EQ(m::startupFeedbackWeight(1,0,.25),.25);
  EXPECT_NEAR(m::startupVelocityEnvelope(.05,.12,.5,.5,.008),.058,1e-12);
  double previous=0;
  for(int i=0;i<=320;++i) {
    const double tau=i/320.0, scale=m::startupSmoothStep(tau);
    const double v=m::startupVelocityEnvelope(.1*scale,.12,scale,tau,.008);
    EXPECT_GE(v,.1*scale); EXPECT_LE(v,.12);
    EXPECT_LT(std::abs(v-previous),.003); previous=v;
  }
  EXPECT_NEAR(previous,.12,1e-12);
}
TEST(TemporalPose, RecoverySlewIsSignedAndPreservesHardEnvelope) {
  EXPECT_NEAR(m::recoveryWheelSlew(.0855,.1234,.01),.1204,1e-12);
  EXPECT_NEAR(m::recoveryWheelSlew(.16,.1,.01),.103,1e-12);
  double v=.16;
  for(int i=0;i<120;++i) { v=m::recoveryWheelSlew(-.16,v,.01); EXPECT_LE(std::abs(v),.16); }
  EXPECT_DOUBLE_EQ(v,-.16);
  EXPECT_DOUBLE_EQ(m::recoveryWheelSlew(.12,.1,.2),.12);
}
TEST(TemporalPose, DelayedCameraDoesNotDoubleCountTranslation) {
  auto p=m::correctAtMeasurementTime({.12,0,0}, {.12,0,0}, {.02,0,0}, {.02,0,0}, .8,.8);
  EXPECT_NEAR(p.x,.12,1e-12);
}
TEST(TemporalPose, CameraCorrectionIsPropagatedToPresent) {
  auto p=m::correctAtMeasurementTime({.12,0,0}, {.12,0,0}, {.02,0,0}, {.03,0,0}, 1,1);
  EXPECT_NEAR(p.x,.13,1e-12);
}
TEST(TemporalPose, RotationAndTranslationRoundTrip) {
  m::Pose2 a{.2,-.3,1.2}, b{.7,.8,-2.8};
  auto p=m::compose(a,m::odometryDelta(a,b));
  EXPECT_NEAR(p.x,b.x,1e-12); EXPECT_NEAR(p.y,b.y,1e-12); EXPECT_NEAR(p.yaw,b.yaw,1e-12);
  auto corrected=m::correctAtMeasurementTime(b,b,a,a,1,1);
  EXPECT_NEAR(corrected.x,b.x,1e-12); EXPECT_NEAR(corrected.y,b.y,1e-12);
}
TEST(TemporalPose, InterpolationUsesShortAngle) {
  auto p=m::interpolatePose({0,0,3.1},{2,4,-3.1},.5);
  EXPECT_DOUBLE_EQ(p.x,1); EXPECT_DOUBLE_EQ(p.y,2); EXPECT_NEAR(std::abs(p.yaw),std::acos(-1),1e-12);
}
TEST(TemporalPose, SourceAgeNotArrivalAgeDefinesLiveInput) {
  EXPECT_FALSE(m::timelyPose(10,10.285,.05)); EXPECT_TRUE(m::timelyPose(10,10.01,.05));
  EXPECT_FALSE(m::timelyPose(0,0,.05)); EXPECT_FALSE(m::timelyPose(10.03,10,.05));
  EXPECT_FALSE(m::timelyPose(10,std::numeric_limits<double>::quiet_NaN(),.05));
}
