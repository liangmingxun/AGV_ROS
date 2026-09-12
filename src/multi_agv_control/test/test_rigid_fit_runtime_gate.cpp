#include <gtest/gtest.h>
#include <limits>
#include "multi_agv_control/rigid_fit_runtime_gate.hpp"

using multi_agv_control::RigidFitRuntimeGate;
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
