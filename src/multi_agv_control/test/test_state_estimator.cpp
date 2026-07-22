#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/s_curve_path.hpp"
#include "multi_agv_control/state_estimator.hpp"

using multi_agv_control::PathProjector;
using multi_agv_control::PathProjectorConfig;
using multi_agv_control::PlanarPose;
using multi_agv_control::ProjectionCurveSample;
using multi_agv_control::SCurvePath;
using multi_agv_control::StateEstimator;
using multi_agv_control::StateEstimatorConfig;
using multi_agv_control::SupportOffset;

namespace {

PathProjector makeProjector(const SCurvePath& path) {
  PathProjectorConfig config;
  config.maximum_projection_distance = 0.2;
  return PathProjector(
      path.length(),
      [&path](double s) {
        const auto value = path.sample(s);
        return ProjectionCurveSample{value.position, value.first_derivative,
                                     value.second_derivative};
      },
      config);
}

StateEstimatorConfig estimatorConfig() {
  StateEstimatorConfig config;
  config.filter_alpha = 0.75;
  config.projection_half_window = 0.3;
  config.initial_progress = 0.0;
  config.minimum_measurement_interval = 1e-4;
  config.maximum_measurement_interval = 0.2;
  config.maximum_absolute_speed = 0.5;
  return config;
}

}  // namespace

TEST(StateEstimator, RejectsInvalidConfiguration) {
  const SCurvePath path({0.12, 2.0, 20001});
  auto config = estimatorConfig();
  config.filter_alpha = 1.0;
  EXPECT_THROW(StateEstimator(makeProjector(path), config),
               std::invalid_argument);
}

TEST(StateEstimator, UsesCausalDifferenceAndOnePoleFilter) {
  const SCurvePath path({0.12, 2.0, 20001});
  StateEstimator estimator(makeProjector(path), estimatorConfig());
  const auto first = estimator.update(path.sample(0.10).position, 10.0);
  ASSERT_TRUE(first.valid);
  EXPECT_FALSE(first.speed_initialized);
  EXPECT_NEAR(first.progress, 0.10, 2e-6);
  EXPECT_DOUBLE_EQ(first.speed, 0.0);

  const auto second = estimator.update(path.sample(0.11).position, 10.1);
  ASSERT_TRUE(second.valid);
  EXPECT_TRUE(second.speed_initialized);
  EXPECT_NEAR(second.speed, 0.025, 2e-5);

  const auto third = estimator.update(path.sample(0.13).position, 10.2);
  ASSERT_TRUE(third.valid);
  EXPECT_NEAR(third.speed, 0.06875, 3e-5);
}

TEST(StateEstimator, RejectsNonIncreasingStampWithoutChangingHistory) {
  const SCurvePath path({0.12, 2.0, 20001});
  StateEstimator estimator(makeProjector(path), estimatorConfig());
  ASSERT_TRUE(estimator.update(path.sample(0.10).position, 5.0).valid);
  const auto stale = estimator.update(path.sample(0.15).position, 5.0);
  EXPECT_FALSE(stale.valid);
  EXPECT_NEAR(estimator.lastEstimate().progress, 0.10, 2e-6);
  const auto next = estimator.update(path.sample(0.11).position, 5.1);
  ASSERT_TRUE(next.valid);
  EXPECT_NEAR(next.progress, 0.11, 2e-6);
}

TEST(StateEstimator, InvalidProjectionDoesNotUseReferenceOrMutateHistory) {
  const SCurvePath path({0.12, 2.0, 20001});
  StateEstimator estimator(makeProjector(path), estimatorConfig());
  ASSERT_TRUE(estimator.update(path.sample(0.2).position, 1.0).valid);
  const auto invalid = estimator.update({100.0, 100.0}, 1.1);
  EXPECT_FALSE(invalid.valid);
  EXPECT_NEAR(estimator.lastEstimate().progress, 0.2, 2e-6);
}

TEST(StateEstimator, LargeGapResynchronizesWithoutInventingSpeed) {
  const SCurvePath path({0.12, 2.0, 20001});
  StateEstimator estimator(makeProjector(path), estimatorConfig());
  ASSERT_TRUE(estimator.update(path.sample(0.10).position, 1.0).valid);
  const auto gap = estimator.update(path.sample(0.20).position, 2.0);
  EXPECT_FALSE(gap.valid);
  const auto resumed = estimator.update(path.sample(0.21).position, 2.1);
  ASSERT_TRUE(resumed.valid);
  EXPECT_TRUE(resumed.speed_initialized);
  EXPECT_LT(resumed.speed, 0.03);
}

TEST(StateEstimator, ComposesWorldPoseAndFitsRigidLoad) {
  const double pi = 3.14159265358979323846;
  const PlanarPose world_to_odom{{1.0, 2.0}, pi / 2.0};
  const PlanarPose odom_to_base{{2.0, 0.0}, -pi / 4.0};
  const auto world_to_base =
      multi_agv_control::composePose(world_to_odom, odom_to_base);
  EXPECT_NEAR(world_to_base.position.x(), 1.0, 1e-12);
  EXPECT_NEAR(world_to_base.position.y(), 4.0, 1e-12);
  EXPECT_NEAR(world_to_base.yaw, pi / 4.0, 1e-12);

  const std::array<SupportOffset, 3> offsets{
      SupportOffset{0.2, 0.1}, SupportOffset{-0.2, 0.1},
      SupportOffset{0.0, -0.2}};
  const PlanarPose expected_load{{1.2, -0.7}, 0.4};
  std::array<Eigen::Vector2d, 3> measured;
  const double c = std::cos(expected_load.yaw);
  const double s = std::sin(expected_load.yaw);
  for (std::size_t index = 0; index < measured.size(); ++index) {
    const Eigen::Vector2d q{offsets[index].tangent, offsets[index].normal};
    measured[index] = expected_load.position +
        Eigen::Vector2d{c * q.x() - s * q.y(),
                        s * q.x() + c * q.y()};
  }
  const auto fitted = multi_agv_control::fitRigidLoadPose(measured, offsets);
  ASSERT_TRUE(fitted.valid);
  EXPECT_NEAR((fitted.pose.position - expected_load.position).norm(), 0.0,
              1e-12);
  EXPECT_NEAR(multi_agv_control::normalizeAngle(
                  fitted.pose.yaw - expected_load.yaw),
              0.0, 1e-12);
  EXPECT_NEAR(fitted.rms_residual, 0.0, 1e-12);
}
