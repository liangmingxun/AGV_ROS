#include <gtest/gtest.h>
#include <limits>
#include "multi_agv_control/rigid_fit_runtime_gate.hpp"
#include "multi_agv_control/v5_exploration_policy.hpp"

TEST(V5QualityPolicy, ExplicitFakeScopeAndComputability) {
  using namespace multi_agv_control;
  const std::string id="exp2c_v5_yaw_effectiveness_exploration";
  EXPECT_TRUE(v5QualityPolicyAuthorized(false,"","serial",true));
  EXPECT_TRUE(v5QualityPolicyAuthorized(true,id,"fake",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,id,"serial",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,"exp2c_v4_transient_yaw_recovery","fake",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,id,"fake",true));
  const std::string hold_id="exp2c_v5b_yaw_effectiveness_hold_exploration";
  EXPECT_TRUE(v5QualityPolicyAuthorized(true,hold_id,"fake",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,hold_id,"serial",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,hold_id,"fake",true));
  const std::string repeat_id="exp2c_v5b_s1_three_method_repeat_validation";
  EXPECT_TRUE(v5QualityPolicyAuthorized(true,repeat_id,"fake",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,repeat_id,"serial",false));
  EXPECT_FALSE(v5QualityPolicyAuthorized(true,repeat_id,"fake",true));
  std::array<Eigen::Vector2d,3> p{{{0.,0.},{.3,0.},{.15,.26}}};
  EXPECT_TRUE(v5FiniteQualityFit(true,.0105,1.,p));
  EXPECT_TRUE(v5FiniteQualityFit(true,.030,1.,p));
  EXPECT_FALSE(v5FiniteQualityFit(false,.0105,1.,p));
  EXPECT_FALSE(v5FiniteQualityFit(true,std::numeric_limits<double>::infinity(),1.,p));
  p[2]={.15,0.};
  EXPECT_FALSE(v5FiniteQualityFit(true,.0105,1.,p));
  RigidFitRuntimeGate normal;
  EXPECT_FALSE(normal.accept(.0105,1.,.010,0.));
}

using multi_agv_control::RigidFitRuntimeGate;
TEST(FormationRuntimeGate, ObservationRetainsRangeAndFiniteProtection) {
  multi_agv_control::FormationRuntimeGate gate;
  EXPECT_TRUE(gate.accept(.15, 1., .08, .2, true, .79, .8));
  EXPECT_TRUE(gate.accept(.20, 2., .08, .2, true, .80, .8));
  EXPECT_FALSE(gate.accept(.20, 3., .08, .2, true, .801, .8));
  EXPECT_FALSE(gate.accept(.01, 4., .08, .2, true, .3, .8));
  multi_agv_control::FormationRuntimeGate invalid;
  EXPECT_FALSE(invalid.accept(std::numeric_limits<double>::quiet_NaN(), 1., .08, .2, true, .3, .8));
  multi_agv_control::FormationRuntimeGate normal;
  EXPECT_FALSE(normal.accept(.081, 1., .08, 0., false, .3, .8));
}
TEST(RigidFitRuntimeGate, DeformationAndTransientRemainObservable) {
  RigidFitRuntimeGate gate;
  EXPECT_TRUE(gate.accept(.027, 1., .04, .2));
  EXPECT_TRUE(gate.accept(.041, 2., .04, .2));
  EXPECT_TRUE(gate.accept(.041, 2., .04, .2));
  EXPECT_TRUE(gate.accept(.039, 2.1, .04, .2));
  EXPECT_TRUE(gate.accept(.041, 3., .04, .2));
  EXPECT_TRUE(gate.accept(.041, 3.19, .04, .2));
  EXPECT_FALSE(gate.accept(.041, 3.21, .04, .2));
  EXPECT_FALSE(gate.accept(.020, 4., .04, .2));
}
TEST(RigidFitRuntimeGate, InvalidAndLegacyImmediateGate) {
  RigidFitRuntimeGate gate;
  EXPECT_FALSE(gate.accept(std::numeric_limits<double>::quiet_NaN(), 1., .04, .2));
  EXPECT_TRUE(gate.accept(.04, 1., .04, 0.));
  EXPECT_FALSE(gate.accept(.041, 2., .04, 0.));
}
