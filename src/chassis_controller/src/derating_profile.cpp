#include "chassis_controller/derating_profile.hpp"

#include <algorithm>
#include <cmath>

namespace chassis_controller {
namespace {

bool validRatio(double value) {
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

bool valid(const DeratingRatios& r) {
  return validRatio(r.speed_left) && validRatio(r.speed_right) &&
         validRatio(r.acceleration_left) && validRatio(r.acceleration_right) &&
         validRatio(r.deceleration_left) && validRatio(r.deceleration_right);
}

bool equal(const DeratingRatios& a, const DeratingRatios& b) {
  return a.speed_left == b.speed_left && a.speed_right == b.speed_right &&
         a.acceleration_left == b.acceleration_left &&
         a.acceleration_right == b.acceleration_right &&
         a.deceleration_left == b.deceleration_left &&
         a.deceleration_right == b.deceleration_right;
}

DeratingRatios interpolate(const DeratingRatios& a, const DeratingRatios& b,
                           double fraction) {
  const auto mix = [fraction](double from, double to) {
    return from + fraction * (to - from);
  };
  return {mix(a.speed_left, b.speed_left),
          mix(a.speed_right, b.speed_right),
          mix(a.acceleration_left, b.acceleration_left),
          mix(a.acceleration_right, b.acceleration_right),
          mix(a.deceleration_left, b.deceleration_left),
          mix(a.deceleration_right, b.deceleration_right)};
}

}  // namespace

bool DeratingProfile::accept(std::uint32_t sequence,
                             const DeratingRatios& ratios,
                             double ramp_seconds) {
  if ((has_command_ && sequence <= last_sequence_) || !valid(ratios) ||
      !std::isfinite(ramp_seconds) || ramp_seconds < 0.0) {
    return false;
  }
  last_sequence_ = sequence;
  ++revision_;
  has_command_ = true;
  if (equal(ratios, target_)) return true;

  start_ = current_;
  target_ = ratios;
  elapsed_ = 0.0;
  ramp_seconds_ = ramp_seconds;
  if (ramp_seconds_ == 0.0) current_ = target_;
  return true;
}

DeratingRatios DeratingProfile::update(double dt_seconds) {
  if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0 ||
      equal(current_, target_)) {
    return current_;
  }
  elapsed_ += dt_seconds;
  const double fraction = ramp_seconds_ <= 0.0
                              ? 1.0
                              : std::max(0.0, std::min(elapsed_ / ramp_seconds_, 1.0));
  current_ = interpolate(start_, target_, fraction);
  return current_;
}

}  // namespace chassis_controller
