#include "multi_agv_control/dynamic_boundary.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace multi_agv_control {
namespace {

struct Point {
  double x;
  double y;
};

struct Halfspace {
  double a;
  double b;
  double c;
};

bool finite(double value) {
  return std::isfinite(value);
}

std::array<Halfspace, 5> constraints(const DynamicBoundarySet& set) {
  const double lower_min =
      set.physical_lower + set.physical_margin_lower;
  const double lower_max = -set.zero_margin;
  const double upper_min = set.zero_margin;
  const double upper_max =
      set.physical_upper - set.physical_margin_upper;
  return {{{-1.0, 0.0, -lower_min},
           {1.0, 0.0, lower_max},
           {0.0, -1.0, -upper_min},
           {0.0, 1.0, upper_max},
           {1.0, -1.0, -set.minimum_width}}};
}

bool inside(const Point& point, const std::array<Halfspace, 5>& halfspaces,
            double tolerance) {
  for (const auto& constraint : halfspaces) {
    if (constraint.a * point.x + constraint.b * point.y >
        constraint.c + tolerance) {
      return false;
    }
  }
  return true;
}

double squaredDistance(const Point& a, const Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

DynamicBoundaryProjector::DynamicBoundaryProjector(double active_tolerance)
    : active_tolerance_(active_tolerance) {
  if (!finite(active_tolerance_) || active_tolerance_ <= 0.0) {
    throw std::invalid_argument("active tolerance must be positive");
  }
}

bool DynamicBoundaryProjector::feasible(
    const DynamicBoundaryState& value, const DynamicBoundarySet& set,
    double tolerance) {
  if (!finite(value.lower) || !finite(value.upper) ||
      !finite(set.physical_lower) || !finite(set.physical_upper) ||
      !finite(set.physical_margin_lower) ||
      !finite(set.physical_margin_upper) ||
      !finite(set.zero_margin) || !finite(set.minimum_width) ||
      set.physical_lower >= 0.0 || set.physical_upper <= 0.0 ||
      set.physical_margin_lower < 0.0 ||
      set.physical_margin_upper < 0.0 ||
      set.zero_margin <= 0.0 || set.minimum_width <= 0.0 ||
      tolerance < 0.0) {
    return false;
  }
  return inside({value.lower, value.upper}, constraints(set), tolerance);
}

BoundaryProjection DynamicBoundaryProjector::project(
    const DynamicBoundaryState& input,
    const DynamicBoundarySet& set) const {
  BoundaryProjection result;
  result.input = input;
  if (!finite(input.lower) || !finite(input.upper)) {
    return result;
  }

  const auto halfspaces = constraints(set);
  std::vector<Point> vertices;
  for (std::size_t first = 0; first < halfspaces.size(); ++first) {
    for (std::size_t second = first + 1; second < halfspaces.size(); ++second) {
      const auto& a = halfspaces[first];
      const auto& b = halfspaces[second];
      const double determinant = a.a * b.b - b.a * a.b;
      if (std::abs(determinant) <= 1.0e-14) continue;
      const Point point{
          (a.c * b.b - b.c * a.b) / determinant,
          (a.a * b.c - b.a * a.c) / determinant};
      if (inside(point, halfspaces, active_tolerance_)) {
        vertices.push_back(point);
      }
    }
  }
  if (vertices.size() < 3U) {
    return result;
  }

  const Point query{input.lower, input.upper};
  Point best = query;
  double best_distance = std::numeric_limits<double>::infinity();
  if (inside(query, halfspaces, active_tolerance_)) {
    best_distance = 0.0;
  }

  for (const auto& constraint : halfspaces) {
    std::vector<Point> edge_vertices;
    for (const auto& vertex : vertices) {
      if (std::abs(constraint.a * vertex.x +
                   constraint.b * vertex.y - constraint.c) <=
          10.0 * active_tolerance_) {
        edge_vertices.push_back(vertex);
      }
    }
    if (edge_vertices.size() < 2U) continue;
    for (std::size_t first = 0; first < edge_vertices.size(); ++first) {
      for (std::size_t second = first + 1;
           second < edge_vertices.size(); ++second) {
        const Point start = edge_vertices[first];
        const Point end = edge_vertices[second];
        const Point direction{end.x - start.x, end.y - start.y};
        const double norm_squared =
            direction.x * direction.x + direction.y * direction.y;
        if (norm_squared <= 1.0e-20) continue;
        const double parameter = std::clamp(
            ((query.x - start.x) * direction.x +
             (query.y - start.y) * direction.y) / norm_squared,
            0.0, 1.0);
        const Point candidate{
            start.x + parameter * direction.x,
            start.y + parameter * direction.y};
        if (!inside(candidate, halfspaces, 10.0 * active_tolerance_)) {
          continue;
        }
        const double distance = squaredDistance(query, candidate);
        if (distance < best_distance) {
          best = candidate;
          best_distance = distance;
        }
      }
    }
  }
  if (!finite(best_distance)) return result;

  result.projected = {best.x, best.y};
  result.correction = {
      result.projected.lower - input.lower,
      result.projected.upper - input.upper};
  result.corrected = best_distance >
      active_tolerance_ * active_tolerance_;
  result.valid = feasible(result.projected, set, 20.0 * active_tolerance_);

  const auto mark = [&](std::size_t index, BoundaryConstraint flag) {
    const auto& constraint = halfspaces[index];
    if (std::abs(constraint.a * best.x + constraint.b * best.y -
                 constraint.c) <= 20.0 * active_tolerance_) {
      result.active_constraints |= static_cast<std::uint32_t>(flag);
    }
  };
  mark(0U, kLowerPhysical);
  mark(1U, kLowerZero);
  mark(2U, kUpperZero);
  mark(3U, kUpperPhysical);
  mark(4U, kMinimumWidth);
  return result;
}

}  // namespace multi_agv_control

