#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "multi_agv_control/support_geometry.hpp"

using multi_agv_control::SCurvePath;
using multi_agv_control::SupportGeometry;
using multi_agv_control::SupportGeometryConfig;
using multi_agv_control::SupportOffset;

namespace {

double angleDifference(double first, double second) {
  return std::atan2(std::sin(first - second), std::cos(first - second));
}

SCurvePath testPath() {
  return SCurvePath({0.12, 2.0, 20001});
}

SupportGeometryConfig testConfig() {
  return {{{0.18, 0.12}, {-0.18, 0.12}, {0.0, -0.16}},
          0.2,
          0.2,
          10001};
}

}  // namespace

TEST(SupportGeometry, ZeroOffsetsRecoverCenterPath) {
  SupportGeometry geometry(
      testPath(), {{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}, 0.2, 0.2, 2001});
  for (double s = 0.0; s <= geometry.path().length(); s += 0.02) {
    const auto center = geometry.path().sample(s);
    const auto support = geometry.sample(1, s);
    EXPECT_NEAR((support.position - center.position).norm(), 0.0, 1e-14);
    EXPECT_NEAR((support.first_derivative - center.first_derivative).norm(),
                0.0, 1e-13);
    EXPECT_NEAR(support.speed_scale, 1.0, 1e-13);
    EXPECT_NEAR(support.heading_rate, center.curvature, 1e-12);
    EXPECT_NEAR(support.curvature, center.curvature, 1e-12);
  }
}

TEST(SupportGeometry, UsesOneCommonArcLengthForDistinctSupports) {
  const SupportGeometry geometry(testPath(), testConfig());
  const double s = geometry.path().length() * 0.37;
  const auto center = geometry.path().sample(s);
  const auto first = geometry.sample(0, s);
  const auto second = geometry.sample(1, s);
  const auto third = geometry.sample(2, s);
  EXPECT_NEAR(first.s, s, 1e-15);
  EXPECT_NEAR(second.s, s, 1e-15);
  EXPECT_NEAR(third.s, s, 1e-15);
  EXPECT_GT((first.position - second.position).norm(), 0.1);
  EXPECT_GT((first.position - third.position).norm(), 0.1);
  const Eigen::Vector2d expected =
      center.position + first.offset.tangent * center.tangent +
      first.offset.normal * center.normal;
  EXPECT_NEAR((first.position - expected).norm(), 0.0, 1e-13);
}

TEST(SupportGeometry, DerivativesAgreeWithFiniteDifferences) {
  const SupportGeometry geometry(testPath(), testConfig());
  const double h = 1e-4;
  for (std::size_t robot = 0; robot < geometry.supportCount(); ++robot) {
    for (int index = 10; index < 90; index += 8) {
      const double s = geometry.path().length() * index / 100.0;
      const auto lower = geometry.sample(robot, s - h);
      const auto value = geometry.sample(robot, s);
      const auto upper = geometry.sample(robot, s + h);
      const Eigen::Vector2d numerical_first =
          (upper.position - lower.position) / (2.0 * h);
      const Eigen::Vector2d numerical_second =
          (upper.first_derivative - lower.first_derivative) / (2.0 * h);
      EXPECT_NEAR((numerical_first - value.first_derivative).norm(), 0.0,
                  4e-5);
      EXPECT_NEAR((numerical_second - value.second_derivative).norm(), 0.0,
                  5e-5);
      const double numerical_heading_rate =
          angleDifference(upper.heading, lower.heading) / (2.0 * h);
      EXPECT_NEAR(numerical_heading_rate, value.heading_rate, 5e-5);
      EXPECT_NEAR(value.first_derivative.norm(), value.speed_scale, 1e-13);
      EXPECT_NEAR(value.tangent.norm(), 1.0, 1e-13);
      EXPECT_NEAR(value.normal.norm(), 1.0, 1e-13);
    }
  }
}

TEST(SupportGeometry, ReportsNondegeneracyMetrics) {
  const SupportGeometry geometry(testPath(), testConfig());
  EXPECT_GT(geometry.minimumNondegeneracy(), 0.2);
  EXPECT_GT(geometry.minimumSpeedScale(), 0.2);
  EXPECT_GT(geometry.maxCurvatureNormalOffsetProduct(), 0.0);
  EXPECT_LT(geometry.maxCurvatureNormalOffsetProduct(), 1.0);
}

TEST(SupportGeometry, RejectsInvalidOrDegenerateConfiguration) {
  auto config = testConfig();
  config.offsets.pop_back();
  EXPECT_THROW(SupportGeometry(testPath(), config), std::invalid_argument);

  config = testConfig();
  config.offsets[0].normal = 10.0;
  config.validation_samples = 2;
  EXPECT_THROW(SupportGeometry(testPath(), config), std::invalid_argument);

  config = testConfig();
  config.offsets[0].tangent = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SupportGeometry(testPath(), config), std::invalid_argument);

  config = testConfig();
  config.validation_samples = 1;
  EXPECT_THROW(SupportGeometry(testPath(), config), std::invalid_argument);
}

TEST(SupportGeometry, RejectsInvalidQueries) {
  const SupportGeometry geometry(testPath(), testConfig());
  EXPECT_THROW(geometry.sample(3, 0.0), std::out_of_range);
  EXPECT_THROW(geometry.sample(
                   0, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}
