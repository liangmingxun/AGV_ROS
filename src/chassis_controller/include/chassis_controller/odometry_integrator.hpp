#pragma once

namespace chassis_controller {

struct Pose2DState {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class OdometryIntegrator {
 public:
  bool update(double actual_left, double actual_right, double imu_yaw_rate,
              double dt_seconds);
  void reset(const Pose2DState& pose = {});
  const Pose2DState& state() const noexcept { return state_; }
  double linearVelocity() const noexcept { return linear_velocity_; }
  double yawRate() const noexcept { return yaw_rate_; }

 private:
  static double normalizeYaw(double yaw);
  Pose2DState state_{};
  double linear_velocity_{0.0};
  double yaw_rate_{0.0};
};

}  // namespace chassis_controller
