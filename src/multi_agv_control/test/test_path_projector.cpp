#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/path_projector.hpp"
#include "multi_agv_control/s_curve_path.hpp"

using multi_agv_control::PathProjector;
using multi_agv_control::PathProjectorConfig;
using multi_agv_control::ProjectionCurveSample;
using multi_agv_control::SCurvePath;

namespace {

PathProjector centerProjector(const SCurvePath& path,
                              double maximum_distance = 0.5) {
  PathProjectorConfig config;
  config.maximum_projection_distance = maximum_distance;
  return PathProjector(
      path.length(),
      [&path](double s) {
        const auto value = path.sample(s);
        return ProjectionCurveSample{value.position, value.first_derivative,
                                     value.second_derivative};
      },
      config);
}

}  // namespace

TEST(PathProjector, RejectsInvalidConfigurationAndQuery) {
  const SCurvePath path({0.12, 2.0, 20001});
  EXPECT_THROW(PathProjector(0.0, [](double) { return ProjectionCurveSample{}; }),
               std::invalid_argument);
  auto config = PathProjectorConfig{};
  config.coarse_samples = 2;
  EXPECT_THROW(PathProjector(path.length(), [](double) {
                 return ProjectionCurveSample{};
               }, config), std::invalid_argument);
  const auto projector = centerProjector(path);
  EXPECT_FALSE(projector.project(
                            {std::numeric_limits<double>::quiet_NaN(), 0.0},
                            0.0, 0.2)
                   .valid);
  EXPECT_FALSE(projector.project({0.0, 0.0}, 0.0, 0.0).valid);
}

TEST(PathProjector, RecoversExactAndNormallyOffsetPoints) {
  const SCurvePath path({0.12, 2.0, 20001});
  const auto projector = centerProjector(path);
  for (double ratio : {0.05, 0.27, 0.51, 0.83, 0.95}) {
    const double expected_s = path.length() * ratio;
    const auto expected = path.sample(expected_s);
    const auto exact = projector.project(expected.position, expected_s, 0.25);
    ASSERT_TRUE(exact.valid);
    EXPECT_NEAR(exact.s, expected_s, 2e-7);
    EXPECT_NEAR(exact.distance, 0.0, 2e-7);

    const auto noisy = projector.project(
        expected.position + 0.03 * expected.normal, expected_s, 0.25);
    ASSERT_TRUE(noisy.valid);
    EXPECT_NEAR(noisy.s, expected_s, 2e-5);
    EXPECT_NEAR(noisy.distance, 0.03, 2e-5);
  }
}

TEST(PathProjector, ClampsOnlyAtPhysicalPathEndpoints) {
  const SCurvePath path({0.12, 2.0, 20001});
  const auto projector = centerProjector(path, 2.0);
  const auto start = path.sample(0.0);
  const auto end = path.sample(path.length());
  const auto before = projector.project(
      start.position - start.tangent, 0.0, path.length());
  const auto after = projector.project(
      end.position + end.tangent, path.length(), path.length());
  ASSERT_TRUE(before.valid);
  ASSERT_TRUE(after.valid);
  EXPECT_NEAR(before.s, 0.0, 1e-8);
  EXPECT_NEAR(after.s, path.length(), 1e-8);
  EXPECT_TRUE(before.hit_path_boundary);
  EXPECT_TRUE(after.hit_path_boundary);
}

TEST(PathProjector, RejectsPointsOutsideResidualGate) {
  const SCurvePath path({0.12, 2.0, 20001});
  const auto projector = centerProjector(path, 0.05);
  const auto expected = path.sample(path.length() * 0.5);
  const auto result = projector.project(
      expected.position + expected.normal, expected.s, 0.3);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.distance, 0.9);
}

TEST(PathProjector, WideUnloadedPilotKeepsUniqueCircularProgressAndFiniteGate) {
  constexpr double radius = 0.854400373;
  const auto sample = [](double s) {
    const double a = s / radius;
    return ProjectionCurveSample{
        {radius * std::cos(a), radius * std::sin(a)},
        {-std::sin(a), std::cos(a)},
        {-std::cos(a) / radius, -std::sin(a) / radius}};
  };
  PathProjectorConfig config;
  config.maximum_projection_distance = .150;
  const PathProjector projector(5.178, sample, config);
  for (double s : {1.5, 2.5, 3.5}) {
    const auto value = sample(s);
    const auto radial = value.position / radius;
    for (double distance : {-.149, -.081, .081, .149}) {
      const auto result = projector.project(value.position + distance * radial, s - .01, .35);
      ASSERT_TRUE(result.valid);
      EXPECT_NEAR(result.s, s, 1e-7);
      EXPECT_NEAR(result.distance, std::abs(distance), 1e-7);
      EXPECT_FALSE(result.hit_window_boundary);
    }
    EXPECT_FALSE(projector.project(value.position + .151 * radial, s, .35).valid);
  }
}

TEST(PathProjector, RejectsAnUnconvergedRefinement) {
  const SCurvePath path({0.12, 2.0, 20001});
  PathProjectorConfig config;
  config.coarse_samples = 3;
  config.maximum_refinement_iterations = 1;
  config.convergence_tolerance = 1e-15;
  config.maximum_projection_distance = 1.0;
  const PathProjector projector(
      path.length(),
      [&path](double s) {
        const auto value = path.sample(s);
        return ProjectionCurveSample{value.position, value.first_derivative,
                                     value.second_derivative};
      }, config);
  const auto expected = path.sample(0.37 * path.length());
  const auto result = projector.project(expected.position, expected.s, 0.3);
  EXPECT_FALSE(result.converged);
  EXPECT_FALSE(result.valid);
}

TEST(PathProjector, LocalWindowPreventsSCurveBranchJump) {
  const SCurvePath path({1.0, 0.4, 30001});
  const auto projector = centerProjector(path, 0.2);
  const double early_s = path.arcLengthForXi(0.05);
  const double late_s = path.arcLengthForXi(0.15);
  const auto point = path.sample(early_s).position;

  const auto global = projector.project(point, path.length() * 0.5,
                                        path.length());
  const auto local = projector.project(point, late_s, 0.08);
  ASSERT_TRUE(global.valid);
  ASSERT_TRUE(local.valid);
  EXPECT_NEAR(global.s, early_s, 2e-5);
  EXPECT_GT(local.s, early_s + 0.5);
  EXPECT_NEAR(local.s, late_s, 0.08);
}
