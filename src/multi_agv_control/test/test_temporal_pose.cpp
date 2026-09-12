#include <gtest/gtest.h>
#include <limits>
#include "multi_agv_control/temporal_pose.hpp"
#include "multi_agv_control/recovery_slew.hpp"
namespace m = multi_agv_control;
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
