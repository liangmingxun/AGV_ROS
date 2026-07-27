#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "multi_agv_control/lower_channel_controller.hpp"

namespace mac = multi_agv_control;

namespace {

mac::LowerChannelConfig config(mac::LowerMode mode) {
  mac::LowerChannelConfig value;
  value.mode = mode;
  value.sustained_saturation_seconds = 0.03;
  return value;
}

mac::LowerChannelInput input() {
  mac::LowerChannelInput value;
  value.position_actual = 0.02;
  value.velocity_actual = 0.04;
  value.position_reference = 0.0;
  value.velocity_reference = 0.05;
  value.acceleration_reference = 0.0;
  value.velocity_lower_bound = -0.2;
  value.velocity_upper_bound = 0.2;
  value.available_acceleration = 1.0;
  value.available_deceleration = 2.0;
  value.regressor = {{0.4, -0.2}};
  value.dt_seconds = 0.01;
  return value;
}

using CsvRow = std::map<std::string, double>;

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    fields.push_back(field);
  }
  return fields;
}

std::vector<CsvRow> readGolden(const std::string& name) {
  std::ifstream input_file(
      std::string(MULTI_AGV_TEST_DATA_DIR) + "/" + name);
  if (!input_file) return {};
  std::string line;
  std::getline(input_file, line);
  const auto header = splitCsv(line);
  std::vector<CsvRow> rows;
  while (std::getline(input_file, line)) {
    const auto values = splitCsv(line);
    if (values.size() != header.size()) return {};
    CsvRow row;
    for (std::size_t i = 0; i < header.size(); ++i) {
      row[header[i]] = std::stod(values[i]);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace

TEST(LowerChannelController, RModeSwitchesAreOrthogonal) {
  EXPECT_TRUE(mac::lowerAdaptationEnabled(mac::LowerMode::kR1));
  EXPECT_TRUE(mac::lowerDisturbanceCompensationEnabled(mac::LowerMode::kR1));
  EXPECT_TRUE(mac::lowerAdaptationEnabled(mac::LowerMode::kR2));
  EXPECT_FALSE(mac::lowerDisturbanceCompensationEnabled(mac::LowerMode::kR2));
  EXPECT_FALSE(mac::lowerAdaptationEnabled(mac::LowerMode::kR3));
  EXPECT_TRUE(mac::lowerDisturbanceCompensationEnabled(mac::LowerMode::kR3));
  EXPECT_FALSE(mac::lowerAdaptationEnabled(mac::LowerMode::kR4));
  EXPECT_FALSE(mac::lowerDisturbanceCompensationEnabled(mac::LowerMode::kR4));
}

TEST(LowerChannelController, SharedTransformationIsIdenticalAcrossModes) {
  std::array<mac::LowerChannelOutput, 4> outputs;
  const std::array<mac::LowerMode, 4> modes{{
      mac::LowerMode::kR1, mac::LowerMode::kR2,
      mac::LowerMode::kR3, mac::LowerMode::kR4}};
  for (std::size_t index = 0; index < modes.size(); ++index) {
    mac::LowerChannelController controller(config(modes[index]));
    outputs[index] = controller.step(input());
    ASSERT_TRUE(outputs[index].valid);
  }
  for (std::size_t index = 1; index < outputs.size(); ++index) {
    EXPECT_DOUBLE_EQ(outputs[index].position_error,
                     outputs[0].position_error);
    EXPECT_DOUBLE_EQ(outputs[index].velocity_error,
                     outputs[0].velocity_error);
    EXPECT_DOUBLE_EQ(outputs[index].transformed_error,
                     outputs[0].transformed_error);
    EXPECT_DOUBLE_EQ(outputs[index].transformation_gain,
                     outputs[0].transformation_gain);
    EXPECT_DOUBLE_EQ(outputs[index].composite_error,
                     outputs[0].composite_error);
  }
}

TEST(LowerChannelController, AppliesAsymmetricAccelerationLimits) {
  mac::LowerChannelController controller(config(mac::LowerMode::kR4));
  auto value = input();
  value.acceleration_reference = 100.0;
  value.available_acceleration = 0.7;
  value.available_deceleration = 1.6;
  auto output = controller.step(value);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.input_limit_active);
  EXPECT_DOUBLE_EQ(output.input_limited, 0.7);

  controller.reset();
  value.acceleration_reference = -100.0;
  output = controller.step(value);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.input_limit_active);
  EXPECT_DOUBLE_EQ(output.input_limited, -1.6);
}

TEST(LowerChannelController, DetectsSustainedPhysicalSaturation) {
  mac::LowerChannelController controller(config(mac::LowerMode::kR1));
  auto value = input();
  value.physical_wheel_saturation = true;
  EXPECT_FALSE(controller.step(value).sustained_physical_saturation);
  EXPECT_FALSE(controller.step(value).sustained_physical_saturation);
  EXPECT_TRUE(controller.step(value).sustained_physical_saturation);
}

TEST(LowerChannelController, RejectsReferenceOnConstraintBoundary) {
  mac::LowerChannelController controller(config(mac::LowerMode::kR1));
  auto value = input();
  value.velocity_reference = value.velocity_upper_bound;
  EXPECT_FALSE(controller.step(value).valid);
}

TEST(LowerChannelController, FormalExecutionRequiresGoldenVectors) {
  mac::LowerChannelController controller(config(mac::LowerMode::kR1));
  EXPECT_FALSE(controller.formalExecutionReady());
}

TEST(LowerChannelController, CompleteR1MatchesApprovedExperiment1Golden) {
  const auto rows = readGolden("lower_m1_golden.csv");
  ASSERT_EQ(rows.size(), 100U);

  std::array<mac::LowerChannelController, 3> controllers{{
      mac::LowerChannelController(config(mac::LowerMode::kR1)),
      mac::LowerChannelController(config(mac::LowerMode::kR1)),
      mac::LowerChannelController(config(mac::LowerMode::kR1))}};
  for (std::size_t agent = 0; agent < controllers.size(); ++agent) {
    controllers[agent].reset(
        {{rows[0].at("theta_hat_" + std::to_string(agent) + "_0"),
          rows[0].at("theta_hat_" + std::to_string(agent) + "_1")}},
        rows[0].at("Dhat_" + std::to_string(agent)));
  }

  for (std::size_t k = 0; k + 1 < rows.size(); ++k) {
    for (std::size_t agent = 0; agent < controllers.size(); ++agent) {
      const std::string suffix = "_" + std::to_string(agent);
      mac::LowerChannelInput value;
      value.position_actual = rows[k].at("x1" + suffix);
      value.velocity_actual = rows[k].at("x2" + suffix);
      value.position_reference = rows[k].at("sL");
      value.velocity_reference = rows[k].at("vL");
      value.acceleration_reference = rows[k].at("vL_dot");
      value.velocity_lower_bound = -0.15;
      value.velocity_upper_bound = 0.58;
      value.available_acceleration = 2.8;
      value.available_deceleration = 2.8;
      value.regressor = {{
          std::sin(value.position_actual),
          value.velocity_actual * value.velocity_actual}};
      value.dt_seconds = 0.01;

      const auto output = controllers[agent].step(value);
      ASSERT_TRUE(output.valid) << "step=" << k << " agent=" << agent;
      EXPECT_NEAR(output.position_error, rows[k].at("es" + suffix), 1e-12);
      EXPECT_NEAR(output.velocity_error, rows[k].at("ev" + suffix), 1e-12);
      EXPECT_NEAR(
          output.transformed_error, rows[k].at("psi" + suffix), 1e-10);
      EXPECT_NEAR(
          output.transformation_gain, rows[k].at("Gamma" + suffix), 1e-10);
      EXPECT_NEAR(
          output.inverse_gain_term,
          rows[k].at("GammaInvUps" + suffix), 1e-10);
      EXPECT_NEAR(
          output.composite_error, rows[k].at("r" + suffix), 1e-10);
      EXPECT_NEAR(output.input_limited, rows[k].at("u" + suffix), 1e-8);
      EXPECT_NEAR(
          output.parameter_estimate[0],
          rows[k + 1].at(
              "theta_hat_" + std::to_string(agent) + "_0"),
          1e-10);
      EXPECT_NEAR(
          output.parameter_estimate[1],
          rows[k + 1].at(
              "theta_hat_" + std::to_string(agent) + "_1"),
          1e-10);
      EXPECT_NEAR(
          output.disturbance_estimate,
          rows[k + 1].at("Dhat_" + std::to_string(agent)),
          1e-10);
    }
  }
}
