#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "multi_agv_control/m2c_native_governor.hpp"

namespace multi_agv_control {
namespace {

TEST(M2cNativeGovernor, LeavesDemandWithinLimitUnchanged) {
  const auto output = applyM2cNativeGovernor(0.12, -0.17, 0.18);
  EXPECT_FALSE(output.active);
  EXPECT_DOUBLE_EQ(output.scale, 1.0);
  EXPECT_DOUBLE_EQ(output.left_governed, 0.12);
  EXPECT_DOUBLE_EQ(output.right_governed, -0.17);
}

TEST(M2cNativeGovernor, UniformlyScalesBothWheelsBeforeDisturbance) {
  const auto output = applyM2cNativeGovernor(0.20, 0.10, 0.18);
  EXPECT_TRUE(output.active);
  EXPECT_NEAR(output.scale, 0.9, 1e-12);
  EXPECT_NEAR(output.left_governed, 0.18, 1e-12);
  EXPECT_NEAR(output.right_governed, 0.09, 1e-12);
  EXPECT_NEAR(output.left_governed / output.right_governed, 2.0, 1e-12);
}

TEST(M2cNativeGovernor, RejectsNonfiniteAndInvalidLimit) {
  EXPECT_THROW(applyM2cNativeGovernor(NAN, 0.1, 0.18),
               std::invalid_argument);
  EXPECT_THROW(applyM2cNativeGovernor(0.1, 0.1, 0.0),
               std::invalid_argument);
}

}  // namespace
}  // namespace multi_agv_control
