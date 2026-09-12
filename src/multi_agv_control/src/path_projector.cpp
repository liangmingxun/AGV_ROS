#include "multi_agv_control/path_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace multi_agv_control {
double PathProjector::projectedVelocity(const Eigen::Vector2d& point,
    const ProjectionResult& projection, const Eigen::Vector2d& velocity) const {
  if (!projection.valid || !point.allFinite() || !velocity.allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  const auto curve = sample_(projection.s);
  // Differentiate the nearest-point stationarity equation. This retains
  // speed-scale and curvature effects, including slightly off-path motion.
  const double denominator = curve.first_derivative.squaredNorm() +
      (curve.position-point).dot(curve.second_derivative);
  if (!std::isfinite(denominator) || denominator <= 1e-12)
    return std::numeric_limits<double>::quiet_NaN();
  return curve.first_derivative.dot(velocity)/denominator;
}
namespace {

bool finiteVector(const Eigen::Vector2d& value) {
  return std::isfinite(value.x()) && std::isfinite(value.y());
}

}  // namespace

PathProjector::PathProjector(double path_length, SampleFunction sample,
                             const PathProjectorConfig& config)
    : path_length_(path_length), sample_(std::move(sample)), config_(config) {
  if (!std::isfinite(path_length_) || path_length_ <= 0.0 || !sample_) {
    throw std::invalid_argument("path projector requires a finite positive path and sampler");
  }
  if (config_.coarse_samples < 3U ||
      config_.maximum_refinement_iterations == 0U ||
      !std::isfinite(config_.convergence_tolerance) ||
      config_.convergence_tolerance <= 0.0 ||
      !std::isfinite(config_.maximum_projection_distance) ||
      config_.maximum_projection_distance <= 0.0) {
    throw std::invalid_argument("invalid path projector configuration");
  }
}

double PathProjector::squaredDistance(const Eigen::Vector2d& point,
                                      double s) const {
  const auto value = sample_(std::clamp(s, 0.0, path_length_));
  if (!finiteVector(value.position)) {
    return std::numeric_limits<double>::infinity();
  }
  return (value.position - point).squaredNorm();
}

ProjectionResult PathProjector::project(const Eigen::Vector2d& point,
                                        double previous_s,
                                        double half_window) const {
  ProjectionResult result;
  if (!finiteVector(point) || !std::isfinite(previous_s) ||
      !std::isfinite(half_window) || half_window <= 0.0) {
    return result;
  }

  const double center = std::clamp(previous_s, 0.0, path_length_);
  const double lower = std::max(0.0, center - half_window);
  const double upper = std::min(path_length_, center + half_window);
  if (!(upper > lower)) {
    return result;
  }

  const double step = (upper - lower) /
                      static_cast<double>(config_.coarse_samples - 1U);
  std::size_t best_index = 0U;
  double best_s = lower;
  double best_cost = squaredDistance(point, best_s);
  for (std::size_t index = 1U; index < config_.coarse_samples; ++index) {
    const double candidate = lower + step * static_cast<double>(index);
    const double cost = squaredDistance(point, candidate);
    if (cost < best_cost) {
      best_index = index;
      best_s = candidate;
      best_cost = cost;
    }
  }
  if (!std::isfinite(best_cost)) {
    return result;
  }

  double left = best_index == 0U ? lower : best_s - step;
  double right = best_index + 1U == config_.coarse_samples ? upper
                                                           : best_s + step;
  constexpr double inverse_phi = 0.6180339887498948482;
  double x1 = right - inverse_phi * (right - left);
  double x2 = left + inverse_phi * (right - left);
  double f1 = squaredDistance(point, x1);
  double f2 = squaredDistance(point, x2);
  std::size_t iterations = 0U;
  for (; iterations < config_.maximum_refinement_iterations; ++iterations) {
    if (right - left <= config_.convergence_tolerance) {
      break;
    }
    if (f1 <= f2) {
      right = x2;
      x2 = x1;
      f2 = f1;
      x1 = right - inverse_phi * (right - left);
      f1 = squaredDistance(point, x1);
    } else {
      left = x1;
      x1 = x2;
      f1 = f2;
      x2 = left + inverse_phi * (right - left);
      f2 = squaredDistance(point, x2);
    }
  }

  best_s = (left + right) * 0.5;
  // A bounded Newton polish uses the analytical curve derivatives while the
  // golden section above supplies a robust bracket for strongly curved paths.
  for (std::size_t polish = 0U; polish < 4U; ++polish) {
    const auto value = sample_(best_s);
    if (!finiteVector(value.position) || !finiteVector(value.first_derivative) ||
        !finiteVector(value.second_derivative)) {
      return result;
    }
    const Eigen::Vector2d residual = value.position - point;
    const double first = residual.dot(value.first_derivative);
    const double second = value.first_derivative.squaredNorm() +
                          residual.dot(value.second_derivative);
    if (!std::isfinite(second) || std::abs(second) < 1e-14) {
      break;
    }
    const double candidate = std::clamp(best_s - first / second, left, right);
    if (std::abs(candidate - best_s) <= config_.convergence_tolerance) {
      best_s = candidate;
      break;
    }
    best_s = candidate;
  }

  const double lower_cost = squaredDistance(point, lower);
  const double upper_cost = squaredDistance(point, upper);
  best_cost = squaredDistance(point, best_s);
  if (lower_cost <= best_cost) {
    best_s = lower;
    best_cost = lower_cost;
  }
  if (upper_cost < best_cost) {
    best_s = upper;
    best_cost = upper_cost;
  }

  const double boundary_tolerance =
      std::max(config_.convergence_tolerance * 10.0, 1e-8);
  result.converged = std::isfinite(best_cost) &&
                     right - left <= config_.convergence_tolerance;
  result.s = best_s;
  result.distance = std::sqrt(std::max(0.0, best_cost));
  result.refinement_iterations = iterations;
  result.hit_path_boundary = best_s <= boundary_tolerance ||
                             best_s >= path_length_ - boundary_tolerance;
  result.hit_window_boundary =
      (!result.hit_path_boundary) &&
      (best_s <= lower + boundary_tolerance ||
       best_s >= upper - boundary_tolerance);
  result.valid = result.converged && std::isfinite(result.distance) &&
                 result.distance <= config_.maximum_projection_distance;
  return result;
}

}  // namespace multi_agv_control
