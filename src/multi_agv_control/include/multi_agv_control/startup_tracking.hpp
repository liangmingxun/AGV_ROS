#pragma once
#include <algorithm>
#include <cmath>

namespace multi_agv_control {
inline double startupSmoothStep(double tau) {
  tau = std::clamp(tau, 0.0, 1.0);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}
// Symmetric monotone time warp: earlier departure from near-zero speed,
// gentler middle, and unchanged zero endpoint acceleration/jerk.
inline double startupRampScale(double tau, double early_rise) {
  tau = std::clamp(tau, 0.0, 1.0);
  const double warped = tau + early_rise * tau * (1.0-tau) * (1.0-2.0*tau);
  return startupSmoothStep(warped);
}
inline double startupRampRate(double tau, double early_rise, double seconds) {
  tau = std::clamp(tau, 0.0, 1.0);
  const double warped = tau + early_rise * tau * (1.0-tau) * (1.0-2.0*tau);
  const double warp_rate = 1.0 + early_rise * (1.0-6.0*tau+6.0*tau*tau);
  return 30.0 * warped * warped * (1.0-warped) * (1.0-warped) * warp_rate / seconds;
}
inline double startupFeedbackWeight(double elapsed, double seconds, double legacy) {
  return seconds > 0.0 ? startupSmoothStep(elapsed / seconds) : legacy;
}
inline double startupVelocityEnvelope(double reference, double capability,
                                      double scale, double tau, double margin) {
  const double base = std::abs(reference);
  if (margin <= 0.0) return base;
  // Extra catch-up authority is zero at rest. Release the temporary envelope
  // smoothly into the existing capability bound, rather than jumping at T.
  const double catchup = margin * 4.0 * scale * (1.0 - scale);
  const double release = startupSmoothStep((tau - 0.75) / 0.25);
  const double bounded = std::min(capability, base + catchup);
  return bounded + release * (capability - bounded);
}
}  // namespace multi_agv_control
