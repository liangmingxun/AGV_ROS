// Offline prototype only: never linked into the ROS command publisher.
// Uses the real R1 controller; the 1-D plant is a sensitivity model, not an
// identified robot or a camera/geometry/three-car closed-loop qualification.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include "multi_agv_control/lower_channel_controller.hpp"

class Anchor {
 public:
  double filtered{0.0};
  bool initialized{false};
  void reset() { initialized = false; filtered = 0.0; }
  double step(double measured, double acceleration, double dt,
              double tau, double lower, double upper) {
    if (!std::isfinite(measured) || !std::isfinite(acceleration) ||
        !std::isfinite(dt) || !std::isfinite(tau) ||
        !std::isfinite(lower) || !std::isfinite(upper) ||
        dt <= 0.0 || dt > 0.05 || tau <= 0.0 || lower > upper) {
      reset();
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (!initialized) { filtered = measured; initialized = true; }
    else filtered += -std::expm1(-dt / tau) * (measured - filtered);
    return std::clamp(filtered + dt * acceleration, lower, upper);
  }
};

void checks() {
  Anchor a;
  assert(std::abs(a.step(.08, 0., .01, .1, -.1, .2) - .08) < 1e-12);
  assert(a.step(.08, 100., .01, .1, -.1, .09) == .09);
  assert(a.step(.08, -100., .01, .1, -.01, .09) == -.01);
  assert(std::isnan(a.step(.08, 0., 0., .1, -.1, .2)));
  assert(!a.initialized);
  assert(std::isnan(a.step(std::numeric_limits<double>::infinity(),
                          0., .01, .1, -.1, .2)));
  assert(std::isnan(a.step(.08, 0., .06, .1, -.1, .2)));
  a.step(.2, 0., .01, .1, -.1, .2); a.reset();
  assert(std::abs(a.step(.03, 0., .01, .1, -.1, .2) - .03) < 1e-12);
}

void simulate(bool anchored, double tau, double gain, double lag) {
  namespace mac = multi_agv_control;
  mac::LowerChannelController r1{mac::LowerChannelConfig{}};
  r1.reset({{0., 0.}}, .08);
  Anchor filter;
  double position = 0., velocity = 0., reference = 0., command = 0., previous_ref = 0.;
  double low_sq = 0., low_mean = 0., peak = 0., change_sq = 0., previous = 0.;
  int count = 0, invalid = 0;
  constexpr double dt = .01;
  for (int tick = 0; tick < 6500; ++tick) {
    const double t = tick * dt;
    // Prescribed common reference, 1-s down/up transitions, equal for both
    // variants. Same 3.2-s quintic soft start. Not an M1 upper-loop model.
    double ref = (t >= 17. && t < 42.) ? .08 : .10;
    if (t >= 17. && t < 18.) ref = .10 - .02 * (t - 17.);
    if (t >= 42. && t < 43.) ref = .08 + .02 * (t - 42.);
    if (t < 3.2) {
      const double q = t / 3.2;
      ref *= q*q*q*(10. + q*(-15. + 6.*q));
    }
    reference += dt * ref;
    const double noise = .004 * std::sin(2.*3.141592653589793*9.*t)
                       + .002 * std::sin(2.*3.141592653589793*17.*t);
    mac::LowerChannelInput in;
    in.position_actual = position; in.velocity_actual = velocity + noise;
    in.position_reference = reference; in.velocity_reference = ref;
    in.acceleration_reference = (ref - previous_ref) / dt;
    previous_ref = ref;
    in.velocity_lower_bound = -.15; in.velocity_upper_bound = .58;
    in.available_acceleration = .3; in.available_deceleration = .3;
    in.regressor = {{std::sin(position), in.velocity_actual*in.velocity_actual}};
    const auto out = r1.step(in);
    if (!out.valid) { ++invalid; command = 0.; filter.reset(); }
    else {
      const double envelope = t < 3.2 ? ref : .125;
      command = anchored
          ? filter.step(in.velocity_actual, out.input_limited, dt, tau,
                        -envelope, envelope)
          : std::clamp(command + dt*out.input_limited, -envelope, envelope);
    }
    // Method-independent longitudinal tracking, then first-order plant.
    // Gains .85..1.25 and lag .08.. .25 are assumptions, not fitted hardware.
    const double body_demand = std::clamp(command - (position-reference), -.16, .16);
    velocity += -std::expm1(-dt/lag) * (gain*body_demand - velocity);
    position += dt * velocity;
    peak = std::max(peak, std::abs(velocity));
    if (t >= 20. && t < 41.) {
      const double error = position-reference;
      low_sq += error*error; low_mean += error;
      change_sq += (command-previous)*(command-previous); ++count;
    }
    previous = command;
  }
  std::cout << (anchored ? "anchor" : "persistent") << "," << tau << ","
            << gain << "," << lag << "," << std::sqrt(low_sq/count) << ","
            << low_mean/count << "," << peak << ","
            << std::sqrt(change_sq/count) << "," << invalid << "\n";
}

int main(int argc, char** argv) {
  checks();
  std::cout.precision(12);
  if (argc == 2 && std::string(argv[1]) == "--simulate") {
    std::cout << "mode,tau,gain,lag,low_progress_rms,low_progress_mean,peak_velocity,command_step_rms,invalid\n";
    for (double gain : {.85, 1., 1.13, 1.25})
      for (double lag : {.08, .15, .25}) {
        simulate(false, .1, gain, lag);
        for (double tau : {.04, .1, .2}) simulate(true, tau, gain, lag);
      }
    return 0;
  }
  if (argc != 3 || std::string(argv[1]) != "--replay") return 2;
  Anchor a;
  const double tau = std::stod(argv[2]);
  double measured, acceleration, dt, lower, upper;
  while (std::cin >> measured >> acceleration >> dt >> lower >> upper) {
    const double command = a.step(measured, acceleration, dt, tau, lower, upper);
    std::cout << command << "\n";
  }
  return 0;
}
