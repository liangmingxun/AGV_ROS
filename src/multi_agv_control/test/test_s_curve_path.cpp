#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/s_curve_path.hpp"

using multi_agv_control::SCurveConfig;
using multi_agv_control::SCurvePath;

namespace {

constexpr double kPi = 3.14159265358979323846;

SCurvePath testPath() {
  return SCurvePath({0.15, 2.0, 20001});
}

}  // namespace

TEST(SCurvePath, RejectsInvalidConfiguration) {
  EXPECT_THROW(SCurvePath({0.1, 0.0, 100}), std::invalid_argument);
  EXPECT_THROW(SCurvePath({0.1, 1.0, 100}), std::invalid_argument);
  EXPECT_THROW(SCurvePath(
      {std::numeric_limits<double>::quiet_NaN(), 1.0, 100}),
      std::invalid_argument);
}

TEST(SCurvePath, ClampsEndpointsAndRejectsNonFiniteQuery) {
  const auto path = testPath();
  const auto before = path.sample(-1.0);
  const auto after = path.sample(path.length() + 1.0);
  EXPECT_DOUBLE_EQ(before.s, 0.0);
  EXPECT_DOUBLE_EQ(before.xi, 0.0);
  EXPECT_NEAR(before.position.x(), 0.0, 1e-15);
  EXPECT_NEAR(before.position.y(), 0.0, 1e-15);
  EXPECT_DOUBLE_EQ(after.s, path.length());
  EXPECT_DOUBLE_EQ(after.xi, path.config().longitudinal_length);
  EXPECT_NEAR(after.position.x(), 2.0, 1e-15);
  EXPECT_NEAR(after.position.y(), 0.0, 1e-14);
  EXPECT_THROW(path.sample(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(SCurvePath, IsArcLengthParameterized) {
  const auto path = testPath();
  for (double s = 0.0; s <= path.length(); s += 0.01) {
    const auto value = path.sample(s);
    EXPECT_NEAR(value.first_derivative.norm(), 1.0, 1e-12);
    EXPECT_NEAR((value.first_derivative - value.tangent).norm(), 0.0, 1e-14);
    EXPECT_NEAR(value.first_derivative.dot(value.second_derivative), 0.0,
                1e-12);
    EXPECT_NEAR(value.second_derivative.dot(value.normal), value.curvature,
                1e-12);
  }
}

TEST(SCurvePath, ZeroAmplitudeIsAnExactStraightLine) {
  const SCurvePath path({0.0, 1.0, 1001});
  EXPECT_DOUBLE_EQ(path.length(), 1.0);
  EXPECT_DOUBLE_EQ(path.maximumAbsoluteCurvature(), 0.0);
  for (int index = 0; index <= 100; ++index) {
    const double s = static_cast<double>(index) / 100.0;
    const auto value = path.sample(s);
    EXPECT_NEAR(value.position.x(), s, 1e-14);
    EXPECT_DOUBLE_EQ(value.position.y(), 0.0);
    EXPECT_DOUBLE_EQ(value.heading, 0.0);
    EXPECT_DOUBLE_EQ(value.curvature, 0.0);
    EXPECT_NEAR((value.tangent - Eigen::Vector2d::UnitX()).norm(),
                0.0, 1e-14);
  }
}

TEST(SCurvePath, RadiusOneCircleIsArcLengthParameterizedAndClosed) {
  SCurveConfig config;
  config.amplitude = 0.0;
  config.longitudinal_length = 2.0 * kPi;
  config.lookup_samples = 20001;
  config.model = "circle";
  config.circle_radius = 1.0;
  const SCurvePath path(config);

  EXPECT_NEAR(path.length(), 2.0 * kPi, 1e-10);
  EXPECT_NEAR(path.maximumAbsoluteCurvature(), 1.0, 1e-12);
  const auto start = path.sample(0.0);
  const auto quarter = path.sample(0.5 * kPi);
  const auto finish = path.sample(2.0 * kPi);
  EXPECT_NEAR(start.position.x(), 0.0, 1e-12);
  EXPECT_NEAR(start.position.y(), 0.0, 1e-12);
  EXPECT_NEAR(quarter.position.x(), 1.0, 1e-12);
  EXPECT_NEAR(quarter.position.y(), 1.0, 1e-12);
  EXPECT_NEAR(finish.position.x(), 0.0, 1e-12);
  EXPECT_NEAR(finish.position.y(), 0.0, 1e-12);
  EXPECT_NEAR(quarter.curvature, 1.0, 1e-12);
  EXPECT_NEAR(quarter.first_derivative.norm(), 1.0, 1e-12);
}

TEST(SCurvePath, ClockwiseHalfMetreCircleHasSmoothCurvatureEntry) {
  SCurveConfig config;
  config.amplitude = 0.0;
  config.circle_radius = 0.5;
  config.entry_straight_length = 0.20;
  config.curvature_ramp_length = 0.40;
  config.circle_direction = -1.0;
  config.longitudinal_length = 0.60 + kPi;
  config.lookup_samples = 40001;
  config.model = "circle_smooth_entry";
  const SCurvePath path(config);

  EXPECT_NEAR(path.length(), 0.60 + kPi, 1e-10);
  EXPECT_NEAR(path.maximumAbsoluteCurvature(), 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(path.sample(0.0).heading, 0.0);
  EXPECT_DOUBLE_EQ(path.sample(0.20).curvature, 0.0);
  EXPECT_NEAR(path.sample(0.40).curvature, -1.0, 1e-12);
  EXPECT_NEAR(path.sample(0.60).curvature, -2.0, 1e-12);
  EXPECT_NEAR(path.sample(0.60).heading, -0.4, 1e-12);
  EXPECT_NEAR(path.sample(path.length()).heading,
              -0.4 - 2.0 * kPi, 1e-10);

  const double epsilon = 1e-6;
  EXPECT_NEAR(path.sample(0.20 - epsilon).curvature,
              path.sample(0.20 + epsilon).curvature, 1e-8);
  EXPECT_NEAR(path.sample(0.60 - epsilon).curvature,
              path.sample(0.60 + epsilon).curvature, 1e-8);
}

TEST(SCurvePath, ArcLengthLookupIsMonotonicAndInvertible) {
  const auto path = testPath();
  EXPECT_GT(path.length(), path.config().longitudinal_length);
  double previous_s = -1.0;
  for (int index = 0; index <= 197; ++index) {
    const double xi = path.config().longitudinal_length * index / 197.0;
    const double s = path.arcLengthForXi(xi);
    EXPECT_GT(s, previous_s);
    EXPECT_NEAR(path.xiForArcLength(s), xi, 2e-12);
    previous_s = s;
  }
}

TEST(SCurvePath, HeadingAndCurvatureRemainContinuousAndFinite) {
  const auto path = testPath();
  auto previous = path.sample(0.0);
  for (int index = 1; index <= 2000; ++index) {
    const auto value = path.sample(path.length() * index / 2000.0);
    EXPECT_TRUE(std::isfinite(value.heading));
    EXPECT_TRUE(std::isfinite(value.curvature));
    EXPECT_TRUE(std::isfinite(value.curvature_derivative));
    EXPECT_LT(std::abs(value.heading - previous.heading), 0.01);
    EXPECT_LT(std::abs(value.curvature - previous.curvature), 0.02);
    previous = value;
  }
}

TEST(SCurvePath, MatchesAnalyticCurvatureAtQuarterWave) {
  const auto path = testPath();
  const double xi = path.config().longitudinal_length / 4.0;
  const auto value = path.sample(path.arcLengthForXi(xi));
  const double wave_number = 2.0 * kPi / path.config().longitudinal_length;
  EXPECT_NEAR(value.xi, xi, 1e-12);
  EXPECT_NEAR(value.position.y(), path.config().amplitude, 1e-12);
  EXPECT_NEAR(value.heading, 0.0, 1e-12);
  EXPECT_NEAR(value.curvature,
              -path.config().amplitude * wave_number * wave_number, 1e-10);
  EXPECT_NEAR(path.maximumAbsoluteCurvature(), std::abs(value.curvature),
              1e-10);
}

TEST(SCurvePath, DerivativesAgreeWithFiniteDifferences) {
  const auto path = testPath();
  const double h = 1e-4;
  for (int index = 10; index < 90; index += 5) {
    const double s = path.length() * index / 100.0;
    const auto lower = path.sample(s - h);
    const auto value = path.sample(s);
    const auto upper = path.sample(s + h);
    const Eigen::Vector2d numerical_first =
        (upper.position - lower.position) / (2.0 * h);
    const Eigen::Vector2d numerical_second =
        (upper.first_derivative - lower.first_derivative) / (2.0 * h);
    EXPECT_NEAR((numerical_first - value.first_derivative).norm(), 0.0, 3e-5);
    EXPECT_NEAR((numerical_second - value.second_derivative).norm(), 0.0,
                3e-5);
    EXPECT_TRUE(std::isfinite(value.heading));
    EXPECT_TRUE(std::isfinite(value.curvature));
    EXPECT_TRUE(std::isfinite(value.curvature_derivative));
  }
}
