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
  SCurveConfig undeclared_exit;
  undeclared_exit.amplitude = 0.05;
  undeclared_exit.longitudinal_length = 1.0;
  undeclared_exit.lookup_samples = 1001;
  undeclared_exit.exit_straight_length = 0.18;
  EXPECT_THROW({
    const SCurvePath path(undeclared_exit);
    (void)path;
  }, std::invalid_argument);

  SCurveConfig impossible_circle_exit;
  impossible_circle_exit.amplitude = 0.0;
  impossible_circle_exit.circle_radius = 0.05;
  impossible_circle_exit.entry_straight_length = 0.20;
  impossible_circle_exit.curvature_ramp_length = 0.40;
  impossible_circle_exit.circle_direction = -1.0;
  impossible_circle_exit.longitudinal_length =
      0.20 + 0.40 + 2.0 * kPi * 0.05;
  impossible_circle_exit.exit_straight_length = 0.18;
  impossible_circle_exit.lookup_samples = 1001;
  impossible_circle_exit.model = "circle_smooth_entry_exit";
  EXPECT_THROW({
    const SCurvePath path(impossible_circle_exit);
    (void)path;
  }, std::invalid_argument);
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

TEST(SCurvePath, TerminalStraightContinuesTangentAtZeroCurvature) {
  SCurveConfig config;
  config.amplitude = 0.05;
  config.longitudinal_length = 1.0;
  config.lookup_samples = 20001;
  config.model = "sine_single_period_with_straight_exit";
  config.exit_straight_length = 0.18;
  const SCurvePath path(config);

  const double curve_end = path.arcLengthForXi(config.longitudinal_length);
  EXPECT_NEAR(path.length() - curve_end, 0.18, 1e-12);
  const auto before = path.sample(curve_end);
  const auto after = path.sample(curve_end + 1e-6);
  const auto finish = path.sample(path.length());
  EXPECT_NEAR(after.heading, before.heading, 1e-12);
  EXPECT_DOUBLE_EQ(after.curvature, 0.0);
  EXPECT_DOUBLE_EQ(finish.curvature, 0.0);
  EXPECT_NEAR((finish.position - before.position).norm(), 0.18, 1e-12);
  EXPECT_NEAR((finish.position - before.position).normalized().dot(
                  before.tangent), 1.0, 1e-12);
}

TEST(SCurvePath, LongPilotHasThreeMetresArcAndTwoPointThreeMetresX) {
  SCurveConfig config;
  config.amplitude = 0.4127396529000677;
  config.longitudinal_length = 2.184049318306566;
  config.lookup_samples = 60001;
  config.model = "sine_single_period_with_straight_exit";
  config.exit_straight_length = 0.18;
  const SCurvePath path(config);

  const auto finish = path.sample(path.length());
  EXPECT_GE(path.length(), 3.0);
  EXPECT_LT(path.length(), 3.0 + 1e-10);
  EXPECT_NEAR(path.length(), 3.0, 1e-10);
  EXPECT_NEAR(finish.position.x(), 2.3, 1e-10);
  EXPECT_NEAR(finish.position.y(), 0.1376787544061468, 1e-10);
  EXPECT_NEAR(path.maximumAbsoluteCurvature(), 3.415946414204, 1e-10);
  EXPECT_DOUBLE_EQ(finish.curvature, 0.0);
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

TEST(SCurvePath, ClockwiseR0p7PilotHasRequestedRadiusAndLength) {
  SCurveConfig config;
  config.amplitude = 0.0;
  config.circle_radius = 0.7;
  config.entry_straight_length = 0.20;
  config.curvature_ramp_length = 0.40;
  config.circle_direction = -1.0;
  config.longitudinal_length = 0.60 + 2.0 * kPi * 0.7;
  config.lookup_samples = 60001;
  config.model = "circle_smooth_entry";
  const SCurvePath path(config);

  EXPECT_NEAR(path.length(), 4.998229715025710, 1e-12);
  EXPECT_NEAR(path.maximumAbsoluteCurvature(), 1.0 / 0.7, 1e-12);
  EXPECT_DOUBLE_EQ(path.sample(0.0).curvature, 0.0);
  EXPECT_NEAR(path.sample(0.60).curvature, -1.0 / 0.7, 1e-12);
  EXPECT_NEAR(path.sample(path.length()).heading,
              -0.20 / 0.7 - 2.0 * kPi, 1e-10);
}

TEST(SCurvePath, ClockwiseR0p7CircleReturnsSmoothlyToStraight) {
  SCurveConfig config;
  config.amplitude = 0.0;
  config.circle_radius = 0.7;
  config.entry_straight_length = 0.20;
  config.curvature_ramp_length = 0.40;
  config.circle_direction = -1.0;
  // Entry straight + ramp-in + shortened constant arc + ramp-out.
  config.longitudinal_length = 0.60 + 2.0 * kPi * 0.7;
  config.exit_straight_length = 0.18;
  config.lookup_samples = 60001;
  config.model = "circle_smooth_entry_exit";
  const SCurvePath path(config);

  const double constant_arc_start = 0.60;
  const double exit_ramp_start = 0.20 + 2.0 * kPi * 0.7;
  const double curve_end = config.longitudinal_length;
  EXPECT_NEAR(path.length(), 5.178229715025710, 1e-12);
  EXPECT_NEAR(path.maximumAbsoluteCurvature(), 1.0 / 0.7, 1e-12);
  EXPECT_DOUBLE_EQ(path.sample(0.20).curvature, 0.0);
  EXPECT_NEAR(path.sample(constant_arc_start).curvature, -1.0 / 0.7,
              1e-12);
  EXPECT_NEAR(path.sample(exit_ramp_start).curvature, -1.0 / 0.7,
              1e-12);
  EXPECT_NEAR(path.sample(curve_end).curvature, 0.0, 1e-12);
  EXPECT_NEAR(path.sample(curve_end).heading, -2.0 * kPi, 1e-10);

  const auto curve_finish = path.sample(curve_end);
  const auto finish = path.sample(path.length());
  EXPECT_DOUBLE_EQ(finish.curvature, 0.0);
  EXPECT_NEAR(finish.heading, curve_finish.heading, 1e-12);
  EXPECT_NEAR((finish.position - curve_finish.position).norm(), 0.18,
              1e-12);
  EXPECT_NEAR((finish.position - curve_finish.position).normalized().dot(
                  curve_finish.tangent), 1.0, 1e-12);

  const double epsilon = 1e-6;
  for (const double boundary :
       {0.20, constant_arc_start, exit_ramp_start, curve_end}) {
    EXPECT_NEAR(path.sample(boundary - epsilon).curvature,
                path.sample(boundary + epsilon).curvature, 1e-8);
  }
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
