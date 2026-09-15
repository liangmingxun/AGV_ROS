#pragma once
#include <cmath>
#include <string>
#include <Eigen/Core>
#include <array>

namespace multi_agv_control {
inline bool v5QualityPolicyAuthorized(bool requested, const std::string& id,
                                     const std::string& transport, bool hardware) {
  return !requested || ((id == "exp2c_v5_yaw_effectiveness_exploration" ||
                        id == "exp2c_v5b_yaw_effectiveness_hold_exploration" ||
                        id == "exp2c_v5b_s1_three_method_repeat_validation") &&
                        transport == "fake" && !hardware);
}
// Numerical degeneracy, not a formation-error or gross-divergence threshold.
inline bool v5TriangleComputable(const std::array<Eigen::Vector2d, 3>& p) {
  for (const auto& v : p) if (!v.allFinite()) return false;
  const Eigen::Vector2d a=p[1]-p[0], b=p[2]-p[0];
  const double scale=a.norm()*b.norm();
  return std::isfinite(scale) && scale>1e-12 &&
      std::abs(a.x()*b.y()-a.y()*b.x())>1e-12*scale;
}
inline bool v5FiniteQualityFit(bool valid, double residual, double stamp,
    const std::array<Eigen::Vector2d, 3>& supports) {
  return valid && std::isfinite(residual) && residual>=0. &&
      std::isfinite(stamp) && v5TriangleComputable(supports);
}
}  // namespace multi_agv_control
