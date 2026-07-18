#include <cassert>
#include <cmath>
#include <limits>

#include "chassis_controller/chassis_core.hpp"
#include "chassis_controller/derating_profile.hpp"
#include "chassis_controller/odometry_integrator.hpp"
#include "chassis_controller/wheel_command_limiter.hpp"

namespace {
bool near(double a, double b, double tolerance = 1e-10) {
  return std::abs(a - b) <= tolerance;
}
}

int main() {
  using namespace chassis_controller;

  WheelCommandLimiter limiter({0.4, 0.4});
  const WheelLimits limits{0.9, 0.9, 1.0, 1.0, 2.0, 2.0};
  const auto reversed = limiter.update({-0.4, -0.4}, limits, 0.1);
  assert(reversed.valid && near(reversed.applied.left, 0.2));
  assert(reversed.applied.left > 0.0 && reversed.decel_limited_left);
  const auto invalid = limiter.update(
      {std::numeric_limits<double>::quiet_NaN(), 0.0}, limits, 0.1);
  assert(!invalid.valid && near(limiter.applied().left, 0.2));

  DeratingProfile profile;
  assert(profile.accept(1, {0.7, 0.7, 0.72, 0.72, 0.72, 0.72}, 1.0));
  assert(near(profile.update(0.1).speed_left, 0.97));
  for (int i = 0; i < 9; ++i) profile.update(0.1);
  assert(near(profile.current().speed_left, 0.7));
  assert(!profile.accept(1, DeratingRatios::nominal(), 1.0));

  OdometryIntegrator odom;
  assert(odom.update(1.0, 1.0, 1.0, 1.0));
  assert(near(odom.state().x, std::cos(0.5)));
  assert(near(odom.state().y, std::sin(0.5)));
  assert(near(odom.state().yaw, 1.0));
  const auto before = odom.state();
  assert(!odom.update(0.0, 0.0, 0.0, 0.0));
  assert(near(odom.state().x, before.x));

  ChassisConfig config;
  config.robot_index = 2;
  config.nominal_limits = {0.9, 0.9, 1.0, 1.0, 2.0, 2.0};
  config.control_loop_overrun_seconds = 0.01;
  ChassisCore core(config);
  assert(!core.acceptCommand({1, 1, {0.8, 0.8}}));
  assert(core.acceptCommand({2, 7, {0.8, 0.8}}));
  DeratingInput derating;
  derating.robot_id = 2;
  derating.sequence = 1;
  derating.active = true;
  derating.mode = 3;
  derating.ratios = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  assert(core.acceptDerating(derating));
  SensorInput sensor;
  sensor.wheel_left_mm_per_second = 200.0;
  sensor.wheel_right_mm_per_second = 400.0;
  core.updateSensors(sensor);
  core.step(0.02);
  assert(core.feedback().command_seq_applied == 7);
  assert(near(core.feedback().actual.left, 0.2));
  assert(near(core.capability().limits.max_velocity_left, 0.45));
  assert(core.feedback().control_loop_overrun);
  return 0;
}
