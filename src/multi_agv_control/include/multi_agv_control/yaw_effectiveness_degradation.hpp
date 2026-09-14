#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multi_agv_control {
// Abstract fake yaw-execution degradation, not a particular mechanical fault.
struct YawEffectivenessConfig {
  bool enabled{false};
  double trigger_progress{2.0};
  double gamma_min{.12};
  double duration{3.5};
};
inline void validateYawEffectivenessConfig(const YawEffectivenessConfig& c) {
  if (!c.enabled) return;
  if (!std::isfinite(c.trigger_progress) || !std::isfinite(c.gamma_min) ||
      !std::isfinite(c.duration) || std::abs(c.trigger_progress-2.)>1e-12 ||
      std::abs(c.duration-3.5)>1e-12 ||
      !(std::abs(c.gamma_min-.12)<1e-12 || std::abs(c.gamma_min-.10)<1e-12 || std::abs(c.gamma_min-.08)<1e-12))
    throw std::invalid_argument("fake yaw effectiveness permits only gamma=.12/.10/.08, T=3.5, trigger=2");
}
struct YawEffectivenessOutput {
  bool triggered{false}, active{false}, finished{false};
  double wall_time{0.}, trigger_wall_time{std::numeric_limits<double>::quiet_NaN()};
  double elapsed{0.}, progress{0.}, envelope{0.}, gamma{1.};
  double left_raw{0.}, right_raw{0.}, left_degraded{0.}, right_degraded{0.};
};
class YawEffectivenessDegradation {
 public:
  YawEffectivenessOutput evaluate(const YawEffectivenessConfig& c, int robot,
      double progress, double wall, double left, double right) {
    validateYawEffectivenessConfig(c);
    YawEffectivenessOutput out;
    out.wall_time=wall; out.progress=progress;
    out.left_raw=out.left_degraded=left; out.right_raw=out.right_degraded=right;
    if (!c.enabled) return out;
    if (robot<1 || robot>3 || !std::isfinite(progress) || !std::isfinite(wall) ||
        !std::isfinite(left) || !std::isfinite(right) || wall<last_wall_)
      throw std::invalid_argument("nonfinite/backward fake yaw effectiveness input");
    last_wall_=wall;
    if (robot!=2) return out;
    if (!triggered_) { if (progress<c.trigger_progress) return out; triggered_=true; trigger_wall_=wall; }
    out.triggered=true; out.trigger_wall_time=trigger_wall_; out.elapsed=wall-trigger_wall_;
    out.finished=out.elapsed>=c.duration; out.active=!out.finished;
    if (out.elapsed>0. && !out.finished) {
      const double u=std::clamp(2.*std::min(out.elapsed,c.duration-out.elapsed)/c.duration,0.,1.);
      out.envelope=u*u*u*(10.+u*(-15.+6.*u));
      out.gamma=1.-(1.-c.gamma_min)*out.envelope;
      if (out.envelope==1.) out.gamma=c.gamma_min;
      const double delta=.5*(right-left);
      const double shift=(1.-out.gamma)*delta;
      out.left_degraded=left+shift; out.right_degraded=right-shift;
    }
    return out;
  }
 private:
  bool triggered_{false};
  double trigger_wall_{0.}, last_wall_{-std::numeric_limits<double>::infinity()};
};
}  // namespace multi_agv_control
