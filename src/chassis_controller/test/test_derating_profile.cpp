#include <gtest/gtest.h>

#include "chassis_controller/derating_profile.hpp"

using chassis_controller::DeratingProfile;
using chassis_controller::DeratingRatios;

TEST(DeratingProfile, ReachesTargetWithoutJump) {
  DeratingProfile profile;
  const DeratingRatios reduced{0.7, 0.7, 0.72, 0.72, 0.72, 0.72};
  ASSERT_TRUE(profile.accept(1, reduced, 1.0));
  EXPECT_NEAR(profile.update(0.1).speed_left, 0.97, 1e-9);
  for (int i = 0; i < 9; ++i) profile.update(0.1);
  EXPECT_NEAR(profile.current().speed_left, 0.7, 1e-9);
}

TEST(DeratingProfile, DuplicateAndOlderSequencesAreRejected) {
  DeratingProfile profile;
  const DeratingRatios nominal{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const DeratingRatios reduced{0.7, 0.6, 0.72, 0.62, 0.72, 0.62};
  EXPECT_TRUE(profile.accept(10, nominal, 1.0));
  EXPECT_FALSE(profile.accept(10, reduced, 1.0));
  EXPECT_FALSE(profile.accept(9, reduced, 1.0));
}

TEST(DeratingProfile, RestoresFromCurrentValueAndPreservesAsymmetry) {
  DeratingProfile profile;
  const DeratingRatios reduced{0.6, 0.8, 0.5, 0.7, 0.4, 0.9};
  ASSERT_TRUE(profile.accept(1, reduced, 1.0));
  profile.update(0.5);
  ASSERT_TRUE(profile.accept(2, DeratingRatios::nominal(), 2.0));
  const auto restored = profile.update(1.0);
  EXPECT_NEAR(restored.speed_left, 0.9, 1e-12);
  EXPECT_NEAR(restored.speed_right, 0.95, 1e-12);
}

TEST(DeratingProfile, RejectsRatiosOutsideOpenClosedUnitInterval) {
  DeratingProfile profile;
  EXPECT_FALSE(profile.accept(1, {0.0, 1.0, 1.0, 1.0, 1.0, 1.0}, 1.0));
  EXPECT_FALSE(profile.accept(1, {1.1, 1.0, 1.0, 1.0, 1.0, 1.0}, 1.0));
}
