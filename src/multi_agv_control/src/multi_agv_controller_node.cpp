#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <XmlRpcValue.h>
#include <agv_msgs/ChassisCommand.h>
#include <agv_msgs/ControllerState.h>
#include <agv_msgs/CooperativeState.h>
#include <agv_msgs/PathReference.h>
#include <ros/ros.h>

#include "multi_agv_control/planar_support_tracker.hpp"
#include "multi_agv_control/s_curve_path.hpp"
#include "multi_agv_control/support_geometry.hpp"

namespace multi_agv_control {
namespace {

constexpr std::size_t kRobotCount = 3U;

double xmlNumber(const XmlRpc::XmlRpcValue& value, const char* key) {
  if (!value.hasMember(key)) {
    throw std::runtime_error(std::string("missing numeric field: ") + key);
  }
  const auto& member = value[key];
  if (member.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(member);
  }
  if (member.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(member);
  }
  throw std::runtime_error(std::string("field is not numeric: ") + key);
}

geometry_msgs::Pose2D poseMessage(const PlanarPose& pose) {
  geometry_msgs::Pose2D message;
  message.x = pose.position.x();
  message.y = pose.position.y();
  message.theta = pose.yaw;
  return message;
}

PlanarPose poseValue(const geometry_msgs::Pose2D& pose) {
  return {{pose.x, pose.y}, pose.theta};
}

}  // namespace

class MultiAgvControllerNode {
 public:
  MultiAgvControllerNode()
      : private_node_("~"), path_(loadPathConfig()),
        geometry_(path_, loadGeometryConfig()),
        tracker_(geometry_, loadTrackerConfig()) {
    private_node_.param("pretest_constant_reference/publish_rate",
                        publish_rate_, 100.0);
    private_node_.param("pretest_constant_reference/maximum_state_age",
                        maximum_state_age_, 0.15);
    private_node_.param("pretest_constant_reference/initial_progress",
                        reference_progress_, 0.0);
    private_node_.param("pretest_constant_reference/constant_load_velocity",
                        configured_velocity_, 0.05);
    private_node_.param<std::string>(
        "pretest_constant_reference/experiment_id", experiment_id_,
        "odom_pretest");
    private_node_.param<std::string>(
        "pretest_constant_reference/method_id", method_id_, "CONSTANT");
    private_node_.param("command_publication_authorized",
                        command_publication_authorized_, false);
    private_node_.param<std::string>("platform_transport_type",
                                     platform_transport_type_, "fake");
    bool path_authorized = false;
    bool geometry_authorized = false;
    bool pretest_authorized = false;
    private_node_.param("path_s_curve/hardware_execution_authorized",
                        path_authorized, false);
    private_node_.param("support_geometry/hardware_execution_authorized",
                        geometry_authorized, false);
    private_node_.param(
        "pretest_constant_reference/hardware_execution_authorized",
        pretest_authorized, false);
    if (!(publish_rate_ > 0.0) || !(maximum_state_age_ > 0.0) ||
        !std::isfinite(reference_progress_) || reference_progress_ < 0.0 ||
        reference_progress_ > path_.length() ||
        !std::isfinite(configured_velocity_) || configured_velocity_ < 0.0) {
      throw std::runtime_error("invalid constant-reference configuration");
    }
    if (command_publication_authorized_ && platform_transport_type_ != "fake" &&
        !(path_authorized && geometry_authorized && pretest_authorized)) {
      throw std::runtime_error(
          "physical command publication refused by hardware authorization gates");
    }

    state_subscriber_ = node_.subscribe(
        "/multi_agv/cooperative_state", 5,
        &MultiAgvControllerNode::receiveState, this);
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      command_publishers_[index] = node_.advertise<agv_msgs::ChassisCommand>(
          "/agv" + std::to_string(index + 1U) + "/chassis_command", 1, false);
    }
    reference_publisher_ = node_.advertise<agv_msgs::PathReference>(
        "/multi_agv/path_reference", 5, false);
    controller_publisher_ = node_.advertise<agv_msgs::ControllerState>(
        "/multi_agv/controller_state", 10, false);
    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate_),
                              &MultiAgvControllerNode::step, this);
  }

 private:
  SCurveConfig loadPathConfig() {
    SCurveConfig config;
    private_node_.param("path_s_curve/amplitude", config.amplitude, 0.12);
    private_node_.param("path_s_curve/longitudinal_length",
                        config.longitudinal_length, 2.0);
    int samples = 20001;
    private_node_.param("path_s_curve/lookup_samples", samples, 20001);
    config.lookup_samples = static_cast<std::size_t>(samples);
    return config;
  }

  SupportGeometryConfig loadGeometryConfig() {
    SupportGeometryConfig config;
    private_node_.param("support_geometry/minimum_nondegeneracy",
                        config.minimum_nondegeneracy, 0.2);
    private_node_.param("support_geometry/minimum_speed_scale",
                        config.minimum_speed_scale, 0.2);
    int samples = 10001;
    private_node_.param("support_geometry/validation_samples", samples, 10001);
    config.validation_samples = static_cast<std::size_t>(samples);
    XmlRpc::XmlRpcValue supports;
    if (!private_node_.getParam("support_geometry/supports", supports) ||
        supports.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        supports.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error("support geometry must contain three supports");
    }
    for (int index = 0; index < supports.size(); ++index) {
      config.offsets.push_back({xmlNumber(supports[index], "q_tangent"),
                                xmlNumber(supports[index], "q_normal")});
    }
    return config;
  }

  PlanarTrackerConfig loadTrackerConfig() {
    PlanarTrackerConfig config;
    private_node_.param(
        "pretest_constant_reference/tracker/longitudinal_gain",
        config.longitudinal_gain, config.longitudinal_gain);
    private_node_.param("pretest_constant_reference/tracker/lateral_gain",
                        config.lateral_gain, config.lateral_gain);
    private_node_.param("pretest_constant_reference/tracker/heading_gain",
                        config.heading_gain, config.heading_gain);
    int reference_samples =
        static_cast<int>(config.chassis_reference_samples);
    private_node_.param(
        "pretest_constant_reference/tracker/chassis_reference_samples",
        reference_samples, reference_samples);
    if (reference_samples < 2) {
      throw std::runtime_error(
          "tracker chassis_reference_samples must be at least two");
    }
    config.chassis_reference_samples =
        static_cast<std::size_t>(reference_samples);
    XmlRpc::XmlRpcValue separation;
    if (!private_node_.getParam(
            "pretest_constant_reference/tracker/wheel_separation",
            separation) ||
        separation.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        separation.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error("tracker wheel_separation must have three values");
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const auto& value = separation[static_cast<int>(index)];
      config.wheel_separation[index] =
          value.getType() == XmlRpc::XmlRpcValue::TypeInt
              ? static_cast<int>(value)
              : static_cast<double>(value);
    }
    XmlRpc::XmlRpcValue offsets;
    if (!private_node_.getParam(
            "pretest_constant_reference/tracker/base_to_support", offsets) ||
        offsets.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        offsets.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error(
          "tracker base_to_support must contain three offsets");
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const auto& value = offsets[static_cast<int>(index)];
      config.base_to_support[index] = {
          xmlNumber(value, "x"), xmlNumber(value, "y")};
    }
    return config;
  }

  void receiveState(const agv_msgs::CooperativeState::ConstPtr& message) {
    state_ = *message;
    state_receive_time_ = ros::Time::now();
    has_state_ = true;
  }

  bool stateUsable(const ros::Time& now) const {
    if (!has_state_ || (now - state_receive_time_).toSec() > maximum_state_age_) {
      return false;
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      if (!state_.robot_pose_valid[index] ||
          !state_.support_pose_valid[index] ||
          !state_.path_state_valid[index]) {
        return false;
      }
    }
    return true;
  }

  void publishCommands(const ros::Time& stamp,
                       const std::array<PlanarTrackingResult, 3>* tracking) {
    if (!command_publication_authorized_) {
      return;
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      agv_msgs::ChassisCommand command;
      command.header.stamp = stamp;
      command.robot_id = static_cast<std::uint8_t>(index + 1U);
      command.command_seq = ++command_sequence_[index];
      command.control_mode = 1U;
      command.experiment_id = experiment_id_;
      command.method_id = method_id_;
      if (tracking != nullptr && (*tracking)[index].valid) {
        const auto& value = (*tracking)[index];
        command.linear_velocity_reference = value.linear_velocity_raw;
        command.angular_velocity_reference = value.angular_velocity_raw;
        command.wheel_linear_velocity_left_raw =
            value.wheel_linear_velocity_left_raw;
        command.wheel_linear_velocity_right_raw =
            value.wheel_linear_velocity_right_raw;
      }
      command_publishers_[index].publish(command);
    }
  }

  void step(const ros::TimerEvent& event) {
    const ros::Time now = ros::Time::now();
    double dt = (event.current_real - event.last_real).toSec();
    if (!std::isfinite(dt) || dt < 0.0 || dt > 0.1) {
      dt = 0.0;
    }
    if (!stateUsable(now)) {
      publishCommands(now, nullptr);
      return;
    }

    const double remaining = path_.length() - reference_progress_;
    const double velocity = remaining > 0.0 ? configured_velocity_ : 0.0;
    reference_progress_ = std::min(
        path_.length(), reference_progress_ + velocity * dt);
    std::array<PlanarPose, 3> robot_poses;
    std::array<PlanarPose, 3> support_poses;
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      robot_poses[index] = poseValue(state_.robot_pose[index]);
      support_poses[index] = poseValue(state_.support_pose[index]);
    }
    const auto tracking = tracker_.trackFleet(
        reference_progress_, velocity, robot_poses, support_poses);

    agv_msgs::PathReference reference;
    reference.header.stamp = now;
    reference.header.frame_id = "world";
    reference.path_id = "s_curve";
    reference.path_version = 1U;
    reference.load_path_progress_reference = reference_progress_;
    reference.load_path_velocity_reference = velocity;
    const auto load = path_.sample(reference_progress_);
    reference.load_pose_reference =
        poseMessage({load.position, load.heading});
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      reference.support_pose_reference[index] =
          poseMessage(tracking[index].support_pose_reference);
      reference.chassis_linear_velocity_feedforward[index] =
          tracking[index].linear_velocity_feedforward;
      reference.chassis_angular_velocity_feedforward[index] =
          tracking[index].angular_velocity_feedforward;
    }
    reference_publisher_.publish(reference);

    agv_msgs::ControllerState controller;
    controller.header.stamp = now;
    controller.header.frame_id = "world";
    controller.experiment_id = experiment_id_;
    controller.method_id = method_id_;
    controller.common_load_velocity_reference = velocity;
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      controller.path_progress_actual[index] = state_.s_actual[index];
      controller.path_velocity_actual[index] = state_.s_dot_actual[index];
      controller.path_progress_execute_reference[index] = reference_progress_;
      controller.path_velocity_execute_reference[index] = velocity;
    }
    controller_publisher_.publish(controller);
    publishCommands(now, &tracking);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  SCurvePath path_;
  SupportGeometry geometry_;
  PlanarSupportTracker tracker_;
  ros::Subscriber state_subscriber_;
  std::array<ros::Publisher, kRobotCount> command_publishers_;
  ros::Publisher reference_publisher_;
  ros::Publisher controller_publisher_;
  ros::Timer timer_;
  agv_msgs::CooperativeState state_;
  ros::Time state_receive_time_;
  std::array<std::uint32_t, kRobotCount> command_sequence_{{0U, 0U, 0U}};
  bool has_state_{false};
  bool command_publication_authorized_{false};
  double publish_rate_{100.0};
  double maximum_state_age_{0.15};
  double reference_progress_{0.0};
  double configured_velocity_{0.05};
  std::string experiment_id_;
  std::string method_id_;
  std::string platform_transport_type_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_agv_controller");
  try {
    multi_agv_control::MultiAgvControllerNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start multi_agv_controller: %s", error.what());
    return 1;
  }
  return 0;
}
