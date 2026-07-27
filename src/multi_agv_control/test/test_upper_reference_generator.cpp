#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "multi_agv_control/upper_reference_generator.hpp"

namespace mac = multi_agv_control;

namespace {

mac::UpperReferenceConfig config(mac::UpperMode mode) {
  mac::UpperReferenceConfig value;
  value.mode = mode;
  value.controller_ticks_per_update = 4U;
  value.fixed_m4_speed = 0.03;
  for (auto& agent : value.agents) {
    agent.admissible_set = {-0.5, 0.5, 0.02, 0.02, 0.02, 0.08};
    agent.initial_boundary = {-0.20, 0.20};
    agent.nominal_boundary = {-0.20, 0.20};
    agent.inner_margin = 0.01;
    agent.restore_gain_lower = 1.0;
    agent.restore_gain_upper = 1.0;
    agent.risk_gain_lower = 0.10;
    agent.risk_gain_upper = 0.10;
    agent.risk_scale = 1.0;
    agent.risk_decay = 0.25;
  }
  return value;
}

std::array<mac::UpperAgentInput, 3> inputs(double capability = 0.5) {
  std::array<mac::UpperAgentInput, 3> value;
  value[0].candidate_velocity = 0.10;
  value[1].candidate_velocity = 0.20;
  value[2].candidate_velocity = 0.30;
  for (auto& agent : value) {
    agent.mapped_upper_capability = capability;
  }
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

mac::UpperReferenceConfig paperM1Config() {
  mac::UpperReferenceConfig value;
  value.mode = mac::UpperMode::kM1;
  value.controller_ticks_per_update = 1U;
  for (auto& agent : value.agents) {
    agent.admissible_set = {-0.15, 0.58, 0.02, 0.02, 0.03, 0.15};
    agent.initial_boundary = {-0.06, 0.52};
    agent.nominal_boundary = {-0.06, 0.52};
    agent.inner_margin = 0.01;
    agent.restore_gain_lower = 0.55;
    agent.restore_gain_upper = 0.65;
    agent.risk_gain_lower = 0.14;
    agent.risk_gain_upper = 0.385;
    agent.risk_scale = 0.95;
    agent.risk_decay = 0.42;
  }
  return value;
}

}  // namespace

TEST(UpperReferenceGenerator, UpdatesAtTwentyFiveHertzAndHoldsOutput) {
  mac::UpperReferenceGenerator generator(config(mac::UpperMode::kM1));
  const auto first = generator.step(inputs(), 0.01);
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.updated);
  for (int tick = 0; tick < 3; ++tick) {
    const auto held = generator.step(inputs(), 0.01);
    EXPECT_TRUE(held.valid);
    EXPECT_FALSE(held.updated);
    EXPECT_DOUBLE_EQ(held.common_velocity, first.common_velocity);
  }
  EXPECT_TRUE(generator.step(inputs(), 0.01).updated);
}

TEST(UpperReferenceGenerator, M1ContractsUnderLowCausalMargin) {
  mac::UpperReferenceGenerator generator(config(mac::UpperMode::kM1));
  auto value = inputs();
  for (auto& agent : value) {
    agent.normalised_causal_margins = {{0.0, 0.0, 0.0, 0.0, 0.0}};
  }
  const auto first = generator.step(value, 0.01);
  ASSERT_TRUE(first.valid);
  EXPECT_GT(first.agents[0].boundary.lower, -0.20);
  EXPECT_LT(first.agents[0].boundary.upper, 0.20);
  EXPECT_GT(first.agents[0].risk_factor, 0.99);
}

TEST(UpperReferenceGenerator, M2aLogsButDoesNotUseCapability) {
  mac::UpperReferenceGenerator high(config(mac::UpperMode::kM2a));
  mac::UpperReferenceGenerator low(config(mac::UpperMode::kM2a));
  const auto high_output = high.step(inputs(0.50), 0.01);
  const auto low_output = low.step(inputs(0.04), 0.01);
  ASSERT_TRUE(high_output.valid);
  ASSERT_TRUE(low_output.valid);
  EXPECT_DOUBLE_EQ(high_output.common_velocity, low_output.common_velocity);
  EXPECT_DOUBLE_EQ(high_output.common_lower, low_output.common_lower);
  EXPECT_DOUBLE_EQ(high_output.common_upper, low_output.common_upper);
  EXPECT_NE(high_output.agents[0].logged_mapped_capability,
            low_output.agents[0].logged_mapped_capability);
}

TEST(UpperReferenceGenerator, M4UsesPreregisteredFixedSpeed) {
  mac::UpperReferenceGenerator generator(config(mac::UpperMode::kM4));
  auto value = inputs();
  value[0].candidate_velocity = -0.1;
  value[1].candidate_velocity = 0.4;
  value[2].candidate_velocity = 0.0;
  const auto output = generator.step(value, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.common_velocity, 0.03);
  generator.setRunActive(true);
  EXPECT_FALSE(generator.setFixedM4Speed(0.04));
}

TEST(UpperReferenceGenerator, FormalExecutionRequiresGoldenVectors) {
  mac::UpperReferenceGenerator generator(config(mac::UpperMode::kM1));
  EXPECT_FALSE(generator.formalExecutionReady());
}

TEST(UpperReferenceGenerator, M1BoundaryMatchesApprovedExperiment2Golden) {
  const auto rows = readGolden("m1_golden.csv");
  ASSERT_EQ(rows.size(), 100U);
  mac::UpperReferenceGenerator generator(paperM1Config());
  for (std::size_t k = 0; k < rows.size(); ++k) {
    std::array<mac::UpperAgentInput, 3> value;
    for (std::size_t agent = 0; agent < value.size(); ++agent) {
      const std::string suffix = "_" + std::to_string(agent);
      value[agent].candidate_velocity = rows[k].at("ups" + suffix);
      value[agent].mapped_upper_capability =
          rows[k].at("capacity_reported" + suffix);
      const double margin = rows[k].at("rho" + suffix);
      value[agent].normalised_causal_margins =
          {{margin, margin, margin, margin, margin}};
    }
    const auto output = generator.step(value, 0.005);
    ASSERT_TRUE(output.valid) << "step=" << k;
    for (std::size_t agent = 0; agent < value.size(); ++agent) {
      const std::string suffix = "_" + std::to_string(agent);
      EXPECT_NEAR(
          output.agents[agent].risk_factor,
          rows[k].at("beta" + suffix), 1e-12);
      EXPECT_NEAR(
          output.agents[agent].boundary.lower,
          rows[k].at("lower" + suffix), 1e-10);
      EXPECT_NEAR(
          output.agents[agent].boundary.upper,
          rows[k].at("upper" + suffix), 1e-10);
    }
    EXPECT_NEAR(output.common_velocity, rows[k].at("v_ref"), 1e-10);
  }
}

TEST(UpperReferenceGenerator, DistributedStateStepMatchesApprovedGolden) {
  const auto rows = readGolden("m1_golden.csv");
  ASSERT_EQ(rows.size(), 100U);
  const mac::DistributedReferenceConfig config;
  for (std::size_t k = 0; k + 1 < rows.size(); ++k) {
    mac::DistributedReferenceState state;
    mac::DistributedReferenceInput input;
    input.leader_position = 0.435 * rows[k].at("t");
    input.leader_velocity = 0.435;
    input.leader_acceleration = 0.0;
    input.inner_margin = 0.01;
    input.dt_seconds = 0.005;
    for (std::size_t agent = 0; agent < 3; ++agent) {
      const std::string suffix = "_" + std::to_string(agent);
      state.position[agent] = rows[k].at("z" + suffix);
      state.velocity[agent] = rows[k].at("ups" + suffix);
      state.auxiliary[agent] = rows[k].at("phi" + suffix);
      input.boundary[agent] = {
          rows[k].at("lower" + suffix),
          rows[k].at("upper" + suffix)};
      if (k == 0) {
        input.boundary_derivative[agent] = {0.0, 0.0};
      } else {
        input.boundary_derivative[agent] = {
            (rows[k].at("lower" + suffix) -
             rows[k - 1].at("lower" + suffix)) / 0.005,
            (rows[k].at("upper" + suffix) -
             rows[k - 1].at("upper" + suffix)) / 0.005};
      }
    }
    const auto output =
        mac::stepDistributedReference(config, state, input);
    ASSERT_TRUE(output.valid) << "step=" << k;
    for (std::size_t agent = 0; agent < 3; ++agent) {
      const std::string suffix = "_" + std::to_string(agent);
      EXPECT_NEAR(
          output.position_disagreement[agent],
          rows[k].at("zeta_z" + suffix), 1e-10);
      EXPECT_NEAR(
          output.velocity_disagreement[agent],
          rows[k].at("zeta_v" + suffix), 1e-10);
      EXPECT_NEAR(
          output.auxiliary_disagreement[agent],
          rows[k].at("zeta_phi" + suffix), 1e-10);
      EXPECT_NEAR(
          output.acceleration[agent],
          rows[k].at("a_ref" + suffix), 1e-10);
      EXPECT_NEAR(
          output.next.position[agent],
          rows[k + 1].at("z" + suffix), 1e-10);
      EXPECT_NEAR(
          output.next.velocity[agent],
          rows[k + 1].at("ups" + suffix), 1e-10);
      EXPECT_NEAR(
          output.next.auxiliary[agent],
          rows[k + 1].at("phi" + suffix), 1e-10);
    }
    EXPECT_NEAR(
        output.common_velocity, rows[k + 1].at("v_ref"), 1e-10);
  }
}
