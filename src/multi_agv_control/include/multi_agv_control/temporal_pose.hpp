#pragma once
#include <cmath>

namespace multi_agv_control {
struct Pose2 { double x{0.0}; double y{0.0}; double yaw{0.0}; };
inline double wrapAngle(double v) { return std::atan2(std::sin(v), std::cos(v)); }
inline Pose2 odometryDelta(const Pose2& a, const Pose2& b) {
  const double c=std::cos(a.yaw), s=std::sin(a.yaw);
  return {c*(b.x-a.x)+s*(b.y-a.y), -s*(b.x-a.x)+c*(b.y-a.y), wrapAngle(b.yaw-a.yaw)};
}
inline Pose2 compose(const Pose2& a, const Pose2& b) {
  const double c=std::cos(a.yaw), s=std::sin(a.yaw);
  return {a.x+c*b.x-s*b.y, a.y+s*b.x+c*b.y, wrapAngle(a.yaw+b.yaw)};
}
inline Pose2 interpolatePose(const Pose2& a, const Pose2& b, double q) {
  return {a.x+q*(b.x-a.x), a.y+q*(b.y-a.y), wrapAngle(a.yaw+q*wrapAngle(b.yaw-a.yaw))};
}
// Correct at the camera measurement time, then propagate to the latest odom.
inline Pose2 correctAtMeasurementTime(const Pose2& fused_now,
    const Pose2& odom_now, const Pose2& odom_camera, const Pose2& camera,
    double position_weight, double heading_weight) {
  Pose2 at_camera=compose(fused_now, odometryDelta(odom_now, odom_camera));
  at_camera.x+=position_weight*(camera.x-at_camera.x);
  at_camera.y+=position_weight*(camera.y-at_camera.y);
  at_camera.yaw=wrapAngle(at_camera.yaw+heading_weight*wrapAngle(camera.yaw-at_camera.yaw));
  return compose(at_camera, odometryDelta(odom_camera, odom_now));
}
inline bool timelyPose(double source, double now, double maximum_age) {
  const double age=now-source;
  return source>0.0 && std::isfinite(age) && age>=-.02 && age<=maximum_age;
}
}  // namespace multi_agv_control
