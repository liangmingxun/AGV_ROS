#include <gtest/gtest.h>

#include "multi_agv_control/dynamic_boundary.hpp"

namespace mac = multi_agv_control;

namespace {

mac::DynamicBoundarySet standardSet() {
  mac::DynamicBoundarySet set;
  set.physical_lower = -0.5;
  set.physical_upper = 0.5;
  set.physical_margin_lower = 0.02;
  set.physical_margin_upper = 0.02;
  set.zero_margin = 0.02;
  set.minimum_width = 0.08;
  return set;
}

}  // namespace

TEST(DynamicBoundaryProjector, LeavesInteriorPointUnchanged) {
  const mac::DynamicBoundaryProjector projector;
  const auto result = projector.project({-0.2, 0.2}, standardSet());
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.corrected);
  EXPECT_DOUBLE_EQ(result.projected.lower, -0.2);
  EXPECT_DOUBLE_EQ(result.projected.upper, 0.2);
  EXPECT_EQ(result.active_constraints, 0U);
}

TEST(DynamicBoundaryProjector, ProjectsToMinimumWidthExactly) {
  const mac::DynamicBoundaryProjector projector;
  const auto result = projector.project({0.1, -0.1}, standardSet());
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.corrected);
  EXPECT_NEAR(result.projected.lower, -0.04, 1.0e-12);
  EXPECT_NEAR(result.projected.upper, 0.04, 1.0e-12);
  EXPECT_NE(result.active_constraints &
                static_cast<std::uint32_t>(mac::kMinimumWidth),
            0U);
}

TEST(DynamicBoundaryProjector, EnforcesPhysicalMargins) {
  const mac::DynamicBoundaryProjector projector;
  const auto result = projector.project({-1.0, 1.0}, standardSet());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.projected.lower, -0.48, 1.0e-12);
  EXPECT_NEAR(result.projected.upper, 0.48, 1.0e-12);
  EXPECT_NE(result.active_constraints &
                static_cast<std::uint32_t>(mac::kLowerPhysical),
            0U);
  EXPECT_NE(result.active_constraints &
                static_cast<std::uint32_t>(mac::kUpperPhysical),
            0U);
}

TEST(DynamicBoundaryProjector, RejectsEmptyAdmissibleSet) {
  auto set = standardSet();
  set.minimum_width = 2.0;
  const mac::DynamicBoundaryProjector projector;
  EXPECT_FALSE(projector.project({-0.2, 0.2}, set).valid);
}
