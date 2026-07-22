#pragma once

#include <array>
#include <cstdint>

#include <ros/time.h>

namespace multi_agv_control {

struct WheelCapability {
  double maximum_velocity_left{0.0};
  double maximum_velocity_right{0.0};
  double maximum_acceleration_left{0.0};
  double maximum_acceleration_right{0.0};
  double maximum_deceleration_left{0.0};
  double maximum_deceleration_right{0.0};
  ros::Time source_stamp;
  std::uint32_t source_sequence{0U};
};

struct CapabilityGeometry {
  double speed_scale{1.0};
  double heading_rate{0.0};
  double wheel_separation{0.0};
};

struct CapabilityReserve {
  double body_linear_velocity{0.0};
  double body_angular_velocity{0.0};
  double wheel_linear_velocity{0.0};
  double path_acceleration{0.0};
  double path_deceleration{0.0};
  double heading_rate_epsilon{1e-6};
};

struct PathCapability {
  double lower_velocity{0.0};
  double upper_velocity{0.0};
  double available_acceleration{0.0};
  double available_deceleration{0.0};
  double actuator_upper_velocity{0.0};
  ros::Time source_stamp;
  std::uint32_t source_sequence{0U};
  bool valid{false};
};

struct FleetCapability {
  std::array<PathCapability, 3> robots;
  double public_upper_velocity{0.0};
  bool valid{false};
};

class CapabilityMapper {
 public:
  explicit CapabilityMapper(const CapabilityReserve& reserve);

  PathCapability map(const WheelCapability& wheel,
                     const CapabilityGeometry& geometry) const;
  FleetCapability mapFleet(
      const std::array<WheelCapability, 3>& wheels,
      const std::array<CapabilityGeometry, 3>& geometries) const;

 private:
  CapabilityReserve reserve_;
};

}  // namespace multi_agv_control
