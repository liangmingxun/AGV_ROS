#pragma once
#include "multi_agv_control/yaw_effectiveness_degradation.hpp"

namespace multi_agv_control {
struct YawHoldConfig {
  bool enabled{false};
  double trigger_progress{2.}, gamma_min{.20}, duration{3.7};
  double down{.6}, hold{2.5}, up{.6};
};
inline void validateYawHoldConfig(const YawHoldConfig& c) {
  if (!c.enabled) return;
  for (double v : {c.trigger_progress,c.gamma_min,c.duration,c.down,c.hold,c.up})
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite v5b profile");
  if (std::abs(c.trigger_progress-2.)>1e-12 || std::abs(c.gamma_min-.20)>1e-12 ||
      std::abs(c.down-.6)>1e-12 || std::abs(c.hold-2.5)>1e-12 ||
      std::abs(c.up-.6)>1e-12 || std::abs(c.duration-3.7)>1e-12)
    throw std::invalid_argument("v5b permits only gamma=.20 down=.6 hold=2.5 up=.6 trigger=2");
}
inline double yawHoldSmoothstep(double u) {
  u=std::clamp(u,0.,1.);return u*u*u*(10.+u*(-15.+6.*u));
}
class YawHoldDegradation {
 public:
  YawEffectivenessOutput evaluate(const YawHoldConfig& c,int robot,double progress,
      double wall,double left,double right) {
    validateYawHoldConfig(c);
    YawEffectivenessOutput out;
    out.wall_time=wall;out.progress=progress;
    out.left_raw=out.left_degraded=left;out.right_raw=out.right_degraded=right;
    if (!c.enabled) return out;
    if (robot<1 || robot>3 || !std::isfinite(progress) || !std::isfinite(wall) ||
        !std::isfinite(left) || !std::isfinite(right) || wall<last_wall_)
      throw std::invalid_argument("nonfinite/backward v5b input");
    last_wall_=wall;
    if (robot!=2) return out;
    if (!triggered_) {
      if (progress<c.trigger_progress) return out;
      triggered_=true;trigger_wall_=wall;
    }
    out.triggered=true;out.trigger_wall_time=trigger_wall_;out.elapsed=wall-trigger_wall_;
    out.finished=out.elapsed>=c.duration;out.active=!out.finished;
    if (out.active) {
      if (out.elapsed<c.down) out.envelope=yawHoldSmoothstep(out.elapsed/c.down);
      else if (out.elapsed<c.down+c.hold) out.envelope=1.;
      else out.envelope=1.-yawHoldSmoothstep((out.elapsed-c.down-c.hold)/c.up);
      out.envelope=std::clamp(out.envelope,0.,1.);
      out.gamma=out.envelope==1.?c.gamma_min:1.-(1.-c.gamma_min)*out.envelope;
      const double shift=(1.-out.gamma)*(.5*right-.5*left);
      out.left_degraded=left+shift;out.right_degraded=right-shift;
      if (!std::isfinite(out.left_degraded) || !std::isfinite(out.right_degraded))
        throw std::invalid_argument("nonfinite v5b output");
    }
    return out;
  }
 private:
  bool triggered_{false};
  double trigger_wall_{0.},last_wall_{-std::numeric_limits<double>::infinity()};
};
}  // namespace multi_agv_control
