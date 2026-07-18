#pragma once

#include <cstdint>

namespace chassis_controller {

struct DeratingRatios {
  double speed_left{1.0};
  double speed_right{1.0};
  double acceleration_left{1.0};
  double acceleration_right{1.0};
  double deceleration_left{1.0};
  double deceleration_right{1.0};

  static DeratingRatios nominal() { return {}; }
};

struct DeratingTarget {
  std::uint32_t sequence{0};
  DeratingRatios ratios{};
  double ramp_seconds{0.0};
};

class DeratingProfile {
 public:
  bool accept(std::uint32_t sequence, const DeratingRatios& ratios,
              double ramp_seconds);
  DeratingRatios update(double dt_seconds);
  const DeratingRatios& current() const noexcept { return current_; }
  std::uint64_t revision() const noexcept { return revision_; }
  std::uint32_t sequence() const noexcept { return last_sequence_; }

 private:
  DeratingRatios current_{};
  DeratingRatios start_{};
  DeratingRatios target_{};
  double elapsed_{0.0};
  double ramp_seconds_{0.0};
  std::uint32_t last_sequence_{0};
  std::uint64_t revision_{0};
  bool has_command_{false};
};

}  // namespace chassis_controller
