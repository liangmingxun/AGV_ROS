#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "multi_agv_control/m2b_controller.hpp"

namespace mac = multi_agv_control;
namespace {

using Row = std::map<std::string, double>;

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> result;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    if (!value.empty() && value.back() == '\r') value.pop_back();
    result.push_back(value);
  }
  return result;
}

std::vector<Row> golden() {
  std::ifstream input(
      std::string(MULTI_AGV_TEST_DATA_DIR) + "/m2b_golden.csv");
  std::string line;
  if (!std::getline(input, line)) return {};
  const auto header = split(line);
  std::vector<Row> rows;
  while (std::getline(input, line)) {
    const auto values = split(line);
    if (values.size() != header.size()) return {};
    Row row;
    for (std::size_t i = 0; i < header.size(); ++i) {
      row[header[i]] = std::stod(values[i]);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace

TEST(M2bController, RejectsGeneratorBoundary) {
  mac::M2bController controller(mac::M2bConfig{});
  mac::M2bGeneratorState state;
  state.velocity[0] = 0.52;
  EXPECT_THROW(controller.reset(state), std::invalid_argument);
}

TEST(M2bController, CapabilityIsLogOnly) {
  mac::M2bController high(mac::M2bConfig{});
  mac::M2bController low(mac::M2bConfig{});
  mac::M2bInput high_input;
  high_input.position_actual = {{0.018, -0.014, 0.010}};
  high_input.velocity_actual = {{
      0.4346666666666667, 0.4166666666666667, 0.4326666666666667}};
  mac::M2bInput low_input = high_input;
  high_input.reported_capability = {{0.57, 0.57, 0.57}};
  low_input.reported_capability = {{0.20, 0.20, 0.20}};
  const auto a = high.step(high_input);
  const auto b = low.step(low_input);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(a.next.position[i], b.next.position[i]);
    EXPECT_DOUBLE_EQ(a.next.velocity[i], b.next.velocity[i]);
    EXPECT_DOUBLE_EQ(a.input_raw[i], b.input_raw[i]);
    EXPECT_NE(a.reported_capability[i], b.reported_capability[i]);
  }
}

TEST(M2bController, PositiveBoundaryLayerIsContinuousNearZero) {
  mac::M2bConfig config;
  config.auxiliary_boundary_layer = 0.05;
  mac::M2bController positive(config);
  mac::M2bController negative(config);
  mac::M2bGeneratorState positive_state;
  mac::M2bGeneratorState negative_state;
  positive_state.auxiliary = {{1.0e-8, 1.0e-8, 1.0e-8}};
  negative_state.auxiliary = {{-1.0e-8, -1.0e-8, -1.0e-8}};
  positive.reset(positive_state);
  negative.reset(negative_state);
  mac::M2bInput input;
  const auto a = positive.step(input);
  const auto b = negative.step(input);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_LT(std::abs(a.next.auxiliary[i] -
                       b.next.auxiliary[i]), 1.0e-6);
  }
}

TEST(M2bController, MatchesApprovedExperiment2Vectors) {
  const auto rows = golden();
  ASSERT_EQ(rows.size(), 100U);
  mac::M2bConfig config;
  config.golden_vectors_verified = true;
  mac::M2bController controller(config);
  for (std::size_t k = 0; k < rows.size(); ++k) {
    mac::M2bInput input;
    input.dt_seconds = 0.005;
    input.leader_position = 0.435 * rows[k].at("t");
    input.leader_velocity = 0.435;
    for (std::size_t i = 0; i < 3; ++i) {
      const std::string suffix = "_" + std::to_string(i);
      input.position_actual[i] = rows[k].at("x1" + suffix);
      input.velocity_actual[i] = rows[k].at("x2" + suffix);
      input.reported_capability[i] =
          rows[k].at("capacity_reported" + suffix);
    }
    const auto output = controller.step(input);
    ASSERT_TRUE(output.valid) << "step=" << k;
    for (std::size_t i = 0; i < 3; ++i) {
      const std::string suffix = "_" + std::to_string(i);
      EXPECT_NEAR(output.current.position[i],
                  rows[k].at("z" + suffix), 2.0e-12);
      EXPECT_NEAR(output.current.velocity[i],
                  rows[k].at("ups" + suffix), 2.0e-12);
      EXPECT_NEAR(output.current.auxiliary[i],
                  rows[k].at("phi" + suffix), 2.0e-12);
      EXPECT_NEAR(output.position_error[i],
                  rows[k].at("es" + suffix), 2.0e-12);
      EXPECT_NEAR(output.velocity_error[i],
                  rows[k].at("ev" + suffix), 2.0e-12);
      EXPECT_NEAR(output.reference_acceleration[i],
                  rows[k].at("a_ref" + suffix), 2.0e-12);
      EXPECT_NEAR(output.transformed_error[i],
                  rows[k].at("psi" + suffix), 2.0e-12);
      EXPECT_NEAR(output.transformation_gain[i],
                  rows[k].at("Gamma" + suffix), 2.0e-12);
      EXPECT_NEAR(output.inverse_gain_term[i],
                  rows[k].at("GammaInvUps" + suffix), 2.0e-12);
      EXPECT_NEAR(output.composite_error[i],
                  rows[k].at("r" + suffix), 2.0e-12);
      EXPECT_NEAR(output.input_raw[i],
                  rows[k].at("u_raw" + suffix), 2.0e-11);
      EXPECT_NEAR(output.input_limited[i],
                  rows[k].at("u_act" + suffix), 2.0e-11);
      EXPECT_NEAR(output.delta_inverse[i],
                  rows[k].at("paper_delta_inv" + suffix), 2.0e-12);
      EXPECT_NEAR(output.mapping_zeta[i],
                  rows[k].at("paper_zeta" + suffix), 2.0e-12);
      EXPECT_DOUBLE_EQ(-config.alpha, rows[k].at("lower" + suffix));
      EXPECT_DOUBLE_EQ(config.beta, rows[k].at("upper" + suffix));
      EXPECT_DOUBLE_EQ(-config.alpha, rows[k].at("lower_s" + suffix));
      EXPECT_DOUBLE_EQ(config.beta, rows[k].at("upper_s" + suffix));
    }
    if (k + 1U < rows.size()) {
      EXPECT_NEAR(output.load_progress_reference,
                  rows[k + 1U].at("s_ref"), 2.0e-12);
      EXPECT_NEAR(output.load_velocity_reference,
                  rows[k + 1U].at("v_ref"), 2.0e-12);
    }
  }
}
