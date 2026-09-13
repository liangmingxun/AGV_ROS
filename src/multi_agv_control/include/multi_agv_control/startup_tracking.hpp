#pragma once
#include <algorithm>
#include <cmath>

namespace multi_agv_control {
inline double startupSmoothStep(double tau) {
  tau = std::clamp(tau, 0.0, 1.0);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
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
