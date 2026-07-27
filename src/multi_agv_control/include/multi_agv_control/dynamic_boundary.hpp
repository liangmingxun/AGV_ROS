#pragma once

#include <array>
#include <cstdint>

namespace multi_agv_control {

enum BoundaryConstraint : std::uint32_t {
  kLowerPhysical = 1U << 0U,
  kLowerZero = 1U << 1U,
  kUpperZero = 1U << 2U,
  kUpperPhysical = 1U << 3U,
  kMinimumWidth = 1U << 4U,
};

struct DynamicBoundarySet {
  double physical_lower{-0.5};
  double physical_upper{0.5};
  double physical_margin_lower{0.02};
  double physical_margin_upper{0.02};
  double zero_margin{0.02};
  double minimum_width{0.08};
};

struct DynamicBoundaryState {
  double lower{-0.2};
  double upper{0.2};
};

struct BoundaryProjection {
  DynamicBoundaryState input;
  DynamicBoundaryState projected;
  DynamicBoundaryState correction;
  std::uint32_t active_constraints{0U};
  bool corrected{false};
  bool valid{false};
};

class DynamicBoundaryProjector {
 public:
  explicit DynamicBoundaryProjector(double active_tolerance = 1.0e-10);

  BoundaryProjection project(
      const DynamicBoundaryState& input,
      const DynamicBoundarySet& set) const;

  static bool feasible(
      const DynamicBoundaryState& value,
      const DynamicBoundarySet& set,
      double tolerance = 1.0e-10);

 private:
  double active_tolerance_;
};

}  // namespace multi_agv_control

