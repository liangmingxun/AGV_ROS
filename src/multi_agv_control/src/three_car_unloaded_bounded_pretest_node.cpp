#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <agv_msgs/ChassisCommand.h>
#include <agv_msgs/ChassisFeedback.h>
#include <agv_msgs/ControllerState.h>
#include <agv_msgs/CooperativeState.h>
#include <agv_msgs/PathReference.h>
#include <ros/master.h>
#include <ros/ros.h>

#include "multi_agv_control/planar_support_tracker.hpp"

namespace multi_agv_control {
namespace {

constexpr std::size_t kRobotCount = 3U;
constexpr double kPi = 3.14159265358979323846;
std::atomic<bool> stop_requested{false};

void requestStop(int) {
  stop_requested.store(true);
}

double finiteParam(ros::NodeHandle& node, const std::string& name,
                   double fallback) {
  const double value = node.param(name, fallback);
  if (!std::isfinite(value)) {
    throw std::runtime_error("non-finite parameter: " + name);
  }
  return value;
}

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

std::array<double, kRobotCount> numericArrayParam(
    ros::NodeHandle& node, const std::string& name,
    const std::array<double, kRobotCount>& fallback) {
  XmlRpc::XmlRpcValue values;
  if (!node.getParam(name, values)) return fallback;
  if (values.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      values.size() != static_cast<int>(kRobotCount)) {
    throw std::runtime_error(name + " must contain exactly three numbers");
  }
  std::array<double, kRobotCount> result{};
  for (std::size_t index = 0; index < kRobotCount; ++index) {
    const auto& value = values[static_cast<int>(index)];
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result[index] = static_cast<int>(value);
    } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      result[index] = static_cast<double>(value);
    } else {
      throw std::runtime_error(name + " contains a non-numeric value");
    }
    if (!std::isfinite(result[index])) {
      throw std::runtime_error(name + " contains a non-finite value");
    }
  }
  return result;
}

PlanarPose poseValue(const geometry_msgs::Pose2D& pose) {
  return {{pose.x, pose.y}, pose.theta};
}

geometry_msgs::Pose2D poseMessage(const PlanarPose& pose) {
  geometry_msgs::Pose2D value;
  value.x = pose.position.x();
  value.y = pose.position.y();
  value.theta = pose.yaw;
  return value;
}

std::vector<std::string> topicPublishers(const std::string& topic) {
  XmlRpc::XmlRpcValue request;
  request.setSize(1);
  request[0] = ros::this_node::getName();
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  if (!ros::master::execute("getSystemState", request, response, payload,
                            true)) {
    throw std::runtime_error("failed to query ROS master system state");
  }
  std::vector<std::string> publishers;
  if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      payload.size() < 1) {
    return publishers;
  }
  const auto& published_topics = payload[0];
  for (int index = 0; index < published_topics.size(); ++index) {
    const auto& entry = published_topics[index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        entry.size() != 2 ||
        static_cast<std::string>(entry[0]) != topic) {
      continue;
    }
    const auto& nodes = entry[1];
    for (int node_index = 0; node_index < nodes.size(); ++node_index) {
      publishers.push_back(static_cast<std::string>(nodes[node_index]));
    }
  }
  return publishers;
}

std::string join(const std::vector<std::string>& values) {
  std::string result;
  for (const auto& value : values) {
    if (!result.empty()) result += ", ";
    result += value;
  }
  return result;
}

}  // namespace

class ThreeCarUnloadedBoundedPretestNode {
 public:
  ThreeCarUnloadedBoundedPretestNode() : private_("~") {
    loadConfiguration();
    validateAuthorization();
    command_sequence_.fill(initial_command_sequence_);
    initialiseGeometry();
    rejectCompetingPublishers();

    state_subscriber_ = node_.subscribe(
        "/multi_agv/cooperative_state", 10,
        &ThreeCarUnloadedBoundedPretestNode::receiveState, this,
        ros::TransportHints().tcpNoDelay());
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const auto robot = std::to_string(index + 1U);
      // Chassis feedback carries packet progression, battery voltage and
      // safety flags. Unlike a latest-state camera pose, dropping it can
      // directly trip the motion gate, so use reliable TCPROS on hardware as
      // well as in fake tests. Camera-fused poses retain their separate
      // UDPROS-preferred subscription in path_state_estimator_node.
      ros::TransportHints feedback_transport_hints;
      feedback_transport_hints.reliable().tcpNoDelay();
      feedback_subscribers_[index] = node_.subscribe<agv_msgs::ChassisFeedback>(
          "/agv" + robot + "/chassis_feedback", 10,
          [this, index](const agv_msgs::ChassisFeedback::ConstPtr& message) {
            receiveFeedback(index, message);
          },
          ros::VoidConstPtr(), feedback_transport_hints);
      command_publishers_[index] = node_.advertise<agv_msgs::ChassisCommand>(
          "/agv" + robot + "/chassis_command", 1, false);
    }
    reference_publisher_ = node_.advertise<agv_msgs::PathReference>(
        "/multi_agv/bounded_pretest/path_reference", 5, false);
    controller_publisher_ = node_.advertise<agv_msgs::ControllerState>(
        "/multi_agv/bounded_pretest/controller_state", 10, false);
  }

  ~ThreeCarUnloadedBoundedPretestNode() {
    if (command_publishers_[0]) {
      publishRepeatedStop();
    }
  }

  int run() {
    ros::Rate rate(publish_rate_);
    if (!waitForReady(rate)) {
      publishRepeatedStop();
      return stop_requested.load() ? 130 : 2;
    }
    synchroniseCommandSequences();

    std::string reason;
    if (!safeToMove(&reason, true)) {
      ROS_ERROR("Three-car bounded pretest refused before start: %s",
                reason.c_str());
      publishRepeatedStop();
      return 3;
    }

    ROS_WARN("Three-car unloaded bounded pretest starts in %.1f seconds: "
             "fixture=0.40 m, speed=%.3f m/s, progress=%.3f m",
             start_delay_, speed_, target_progress_);
    const ros::WallTime delay_start = ros::WallTime::now();
    while (ros::ok() && !stop_requested.load() &&
           (ros::WallTime::now() - delay_start).toSec() < start_delay_) {
      ros::spinOnce();
      if (!safeToMove(&reason, true)) {
        ROS_ERROR("Three-car bounded pretest aborted during start delay: %s",
                  reason.c_str());
        publishRepeatedStop();
        return 4;
      }
      publishStop();
      rate.sleep();
    }
    if (!ros::ok() || stop_requested.load()) {
      publishRepeatedStop();
      return 130;
    }

    const ros::WallTime motion_start = ros::WallTime::now();
    ros::WallTime previous = motion_start;
    double progress = 0.0;
    while (ros::ok() && !stop_requested.load() &&
           progress < target_progress_) {
      ros::spinOnce();
      const ros::WallTime now = ros::WallTime::now();
      const double dt = (now - previous).toSec();
      previous = now;
      if (!safeToMove(&reason, false)) {
        ROS_ERROR("Three-car bounded pretest aborted: %s", reason.c_str());
        publishRepeatedStop();
        return 5;
      }
      const double elapsed = (now - motion_start).toSec();
      const double ramp_scale = startup_ramp_seconds_ > 0.0
          ? std::clamp(elapsed / startup_ramp_seconds_, 0.0, 1.0)
          : 1.0;
      const double requested_velocity = speed_ * ramp_scale;
      auto tracking = trackingAt(progress, requested_velocity);
      double command_scale = 1.0;
      if (!limitTrackingCommands(&tracking, &command_scale, &reason)) {
        ROS_ERROR("Three-car bounded pretest aborted: %s", reason.c_str());
        publishRepeatedStop();
        return 6;
      }
      if (!trackingSafe(tracking, progress, &reason)) {
        ROS_ERROR("Three-car bounded pretest aborted: %s", reason.c_str());
        publishRepeatedStop();
        return 6;
      }
      const double effective_velocity = requested_velocity * command_scale;
      publishTracking(tracking);
      publishReferenceAndController(tracking, progress, effective_velocity);
      if (dt > 0.0 && dt <= maximum_step_) {
        progress = std::min(
            target_progress_, progress + effective_velocity * dt);
      }

      if ((now - motion_start).toSec() > maximum_motion_time_) {
        ROS_ERROR("Three-car bounded pretest aborted: motion timeout");
        publishRepeatedStop();
        return 7;
      }
      rate.sleep();
    }

    publishRepeatedStop();
    publishReferenceAndController(trackingAt(target_progress_, 0.0),
                                  target_progress_, 0.0);
    if (!waitForStopped(rate)) {
      ROS_ERROR("Three-car stop was commanded but all-wheel zero feedback "
                "was not confirmed before timeout");
      return 8;
    }
    const auto stopped_tracking = trackingAt(target_progress_, 0.0);
    for (int repeat = 0; repeat < 3; ++repeat) {
      publishReferenceAndController(
          stopped_tracking, target_progress_, 0.0);
      ros::spinOnce();
      ros::WallDuration(0.02).sleep();
    }
    ROS_INFO("Three-car unloaded bounded pretest completed: %.3f m reference "
             "progress and all-wheel zero feedback confirmed",
             target_progress_);
    return 0;
  }

 private:
  void loadConfiguration() {
    const std::string root = "three_car_unloaded_bounded_pretest/";
    hardware_authorized_ =
        private_.param(root + "hardware_execution_authorized", false);
    command_authorized_ =
        private_.param("command_publication_authorized", false);
    confirm_readonly_gate_ =
        private_.param("confirm_readonly_gate_passed", false);
    confirm_area_clear_ =
        private_.param("confirm_test_area_clear", false);
    confirm_wheels_on_floor_ =
        private_.param("confirm_wheels_on_floor", false);
    confirm_unloaded_fixture_ =
        private_.param("confirm_unloaded_40cm_fixture", false);
    transport_type_ =
        private_.param<std::string>("platform_transport_type", "serial");

    publish_rate_ = finiteParam(
        private_, root + "publish_rate", 100.0);
    state_timeout_ = finiteParam(
        private_, root + "maximum_state_age", 0.15);
    subscriber_wait_ = finiteParam(
        private_, root + "subscriber_wait_seconds", 15.0);
    start_delay_ = finiteParam(
        private_, root + "start_delay_seconds", 5.0);
    stop_confirmation_timeout_ = finiteParam(
        private_, root + "stop_confirmation_timeout_seconds", 4.0);
    stopped_wheel_tolerance_ = finiteParam(
        private_, root + "stopped_wheel_tolerance", 0.01);
    maximum_step_ = finiteParam(
        private_, root + "maximum_integration_step", 0.05);
    maximum_motion_time_ = finiteParam(
        private_, root + "maximum_motion_time", 25.0);
    speed_ = finiteParam(
        private_, root + "motion/load_path_speed", 0.05);
    target_progress_ = finiteParam(
        private_, root + "motion/target_progress", 1.00);
    startup_ramp_seconds_ = finiteParam(
        private_, root + "motion/startup_ramp_seconds", 1.0);
    minimum_battery_voltage_ = finiteParam(
        private_, root + "abort/minimum_battery_voltage", 10.0);
    maximum_feedback_receive_age_ = finiteParam(
        private_, root + "abort/maximum_feedback_receive_age", 0.25);
    maximum_serial_feedback_age_ = finiteParam(
        private_, root + "abort/maximum_serial_feedback_age", 0.25);
    maximum_stamp_spread_ = finiteParam(
        private_, root + "abort/maximum_stamp_spread", 0.02);
    maximum_initial_progress_ = finiteParam(
        private_, root + "abort/maximum_initial_load_progress", 0.03);
    maximum_progress_spread_ = finiteParam(
        private_, root + "abort/maximum_robot_progress_spread", 0.04);
    maximum_reference_error_ = finiteParam(
        private_, root + "abort/maximum_load_progress_error", 0.05);
    maximum_longitudinal_error_ = finiteParam(
        private_, root + "abort/maximum_longitudinal_error", 0.05);
    maximum_lateral_error_ = finiteParam(
        private_, root + "abort/maximum_lateral_error", 0.03);
    maximum_heading_error_ = finiteParam(
        private_, root + "abort/maximum_heading_error", 0.12);
    maximum_wheel_command_ = finiteParam(
        private_, root + "abort/maximum_wheel_linear_velocity", 0.08);
    minimum_wheel_command_scale_ = finiteParam(
        private_, root + "abort/minimum_wheel_command_scale", 0.75);
    required_subscribers_ =
        private_.param(root + "required_command_subscribers", 2);
    readiness_stable_samples_ =
        private_.param(root + "readiness_stable_samples", 20);
    const int initial_sequence =
        private_.param(root + "initial_command_sequence", 41000);
    initial_command_sequence_ =
        static_cast<std::uint32_t>(std::max(initial_sequence, 0));
    experiment_id_ = private_.param<std::string>(
        root + "experiment_id", "three_car_unloaded_s_1m");
    method_id_ = private_.param<std::string>(
        root + "method_id", "ODOM_BOUNDED_COMMON");
    path_id_ = private_.param<std::string>(
        root + "path_id", "s_curve_1m_bounded");
    validation_profile_ = private_.param<std::string>(
        root + "validation_profile", "legacy_unloaded_bounded");
    wheel_separation_ = numericArrayParam(
        private_, root + "tracker/wheel_separation",
        {{0.114, 0.114, 0.114}});
    longitudinal_gain_per_robot_ = numericArrayParam(
        private_, root + "tracker/longitudinal_gain_per_robot",
        {{1.0, 1.0, 1.0}});
    lateral_gain_per_robot_ = numericArrayParam(
        private_, root + "tracker/lateral_gain_per_robot",
        {{2.0, 2.0, 2.0}});
    heading_gain_per_robot_ = numericArrayParam(
        private_, root + "tracker/heading_gain_per_robot",
        {{2.0, 2.0, 2.0}});
    angular_feedforward_scale_positive_ = numericArrayParam(
        private_, root + "tracker/angular_feedforward_scale_positive",
        {{1.0, 1.0, 1.0}});
    angular_feedforward_scale_negative_ = numericArrayParam(
        private_, root + "tracker/angular_feedforward_scale_negative",
        {{1.0, 1.0, 1.0}});
    formation_longitudinal_gain_ = finiteParam(
        private_, root + "tracker/formation_longitudinal_gain", 0.0);
    formation_lateral_gain_ = finiteParam(
        private_, root + "tracker/formation_lateral_gain", 0.0);
    formation_heading_gain_ = finiteParam(
        private_, root + "tracker/formation_heading_gain", 0.0);
    const auto legacy_curvature_preview_seconds = numericArrayParam(
        private_, root + "tracker/curvature_preview_seconds",
        {{0.0, 0.0, 0.0}});
    curvature_preview_seconds_positive_ = numericArrayParam(
        private_, root + "tracker/curvature_preview_seconds_positive",
        legacy_curvature_preview_seconds);
    curvature_preview_seconds_negative_ = numericArrayParam(
        private_, root + "tracker/curvature_preview_seconds_negative",
        legacy_curvature_preview_seconds);
  }

  SCurveConfig loadPathConfig() {
    SCurveConfig config;
    private_.param("path_s_curve/model", config.model,
                   std::string("sine_single_period"));
    private_.param("path_s_curve/amplitude", config.amplitude, 0.05);
    private_.param("path_s_curve/longitudinal_length",
                   config.longitudinal_length, 1.0);
    private_.param("path_s_curve/circle_radius", config.circle_radius, 1.0);
    private_.param("path_s_curve/entry_straight_length",
                   config.entry_straight_length, 0.0);
    private_.param("path_s_curve/curvature_ramp_length",
                   config.curvature_ramp_length, 0.0);
    private_.param("path_s_curve/circle_direction",
                   config.circle_direction, 1.0);
    int samples = private_.param("path_s_curve/lookup_samples", 20001);
    config.lookup_samples =
        static_cast<std::size_t>(std::max(samples, 0));
    return config;
  }

  SupportGeometryConfig loadGeometryConfig() {
    SupportGeometryConfig config;
    private_.param("support_geometry/minimum_nondegeneracy",
                   config.minimum_nondegeneracy, 0.2);
    private_.param("support_geometry/minimum_speed_scale",
                   config.minimum_speed_scale, 0.2);
    int samples = private_.param(
        "support_geometry/validation_samples", 10001);
    config.validation_samples =
        static_cast<std::size_t>(std::max(samples, 0));
    XmlRpc::XmlRpcValue supports;
    if (!private_.getParam("support_geometry/supports", supports) ||
        supports.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        supports.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error("support geometry must contain three supports");
    }
    for (int index = 0; index < supports.size(); ++index) {
      config.offsets.push_back(
          {xmlNumber(supports[index], "q_tangent"),
           xmlNumber(supports[index], "q_normal")});
    }
    return config;
  }

  PlanarTrackerConfig loadTrackerConfig() {
    const std::string root =
        "three_car_unloaded_bounded_pretest/tracker/";
    PlanarTrackerConfig config;
    config.longitudinal_gain = finiteParam(
        private_, root + "longitudinal_gain", 1.0);
    config.lateral_gain = finiteParam(
        private_, root + "lateral_gain", 2.0);
    config.heading_gain = finiteParam(
        private_, root + "heading_gain", 2.0);
    int samples = private_.param(
        root + "chassis_reference_samples", 10001);
    config.chassis_reference_samples =
        static_cast<std::size_t>(std::max(samples, 0));

    XmlRpc::XmlRpcValue offsets;
    if (!private_.getParam(root + "base_to_support", offsets) ||
        offsets.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        offsets.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error(
          "tracker base_to_support must contain three offsets");
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      config.wheel_separation[index] = wheel_separation_[index];
      const auto& offset = offsets[static_cast<int>(index)];
      config.base_to_support[index] =
          {xmlNumber(offset, "x"), xmlNumber(offset, "y")};
    }
    return config;
  }

  void validateAuthorization() const {
    const bool authorized_path_speed =
        (validation_profile_ == "camera_fused_s_1p00" &&
         path_id_ == "s_curve_1m_bounded" &&
         std::abs(speed_ - 0.05) <= 1e-12 &&
         std::abs(target_progress_ - 1.00) <= 1e-12) ||
        (validation_profile_ == "camera_fused_straight_0p30" &&
         path_id_ == "straight_1m_bounded" &&
         std::abs(speed_ - 0.03) <= 1e-12 &&
         std::abs(target_progress_ - 0.30) <= 1e-12) ||
        (validation_profile_ == "camera_fused_circle_r0p5_cw_smooth" &&
         path_id_ == "circle_r0p5_cw_smooth_entry_bounded" &&
         std::abs(speed_ - 0.05) <= 1e-12 &&
         std::abs(target_progress_ - (0.60 + kPi)) <= 1e-9) ||
        (validation_profile_ == "legacy_unloaded_bounded" &&
         std::abs(target_progress_ - 1.00) <= 1e-12 &&
         ((path_id_ == "s_curve_1m_bounded" &&
           std::abs(speed_ - 0.05) <= 1e-12) ||
          (path_id_ == "straight_1m_bounded" &&
           std::abs(speed_ - 0.08) <= 1e-12)));
    if (!command_authorized_) {
      throw std::runtime_error(
          "command publication is disabled; this dedicated entry requires "
          "enable_commands:=true");
    }
    if (!hardware_authorized_) {
      throw std::runtime_error(
          "dedicated three-car bounded-pretest configuration is not authorized");
    }
    if (transport_type_ == "serial" &&
        (!confirm_readonly_gate_ || !confirm_area_clear_ ||
         !confirm_wheels_on_floor_ || !confirm_unloaded_fixture_)) {
      throw std::runtime_error(
          "serial execution requires all four explicit physical confirmations");
    }
    if (transport_type_ != "serial" && transport_type_ != "fake") {
      throw std::runtime_error("platform transport must be serial or fake");
    }
    if (!(publish_rate_ > 0.0) || !(state_timeout_ > 0.0) ||
        !(subscriber_wait_ > 0.0) || start_delay_ < 0.0 ||
        !(stop_confirmation_timeout_ > 0.0) ||
        !(stopped_wheel_tolerance_ >= 0.0) ||
        !(maximum_step_ > 0.0) || !(maximum_motion_time_ > 0.0) ||
        !authorized_path_speed || !(target_progress_ > 0.0) ||
        !(minimum_battery_voltage_ >= 10.0) ||
        !(maximum_feedback_receive_age_ > 0.0) ||
        maximum_feedback_receive_age_ > 0.25 ||
        !(maximum_serial_feedback_age_ > 0.0) ||
        maximum_serial_feedback_age_ > 0.25 ||
        !(maximum_stamp_spread_ > 0.0) ||
        !(maximum_initial_progress_ > 0.0) ||
        !(maximum_progress_spread_ > 0.0) ||
        !(maximum_reference_error_ > 0.0) ||
        !(maximum_longitudinal_error_ > 0.0) ||
        !(maximum_lateral_error_ > 0.0) ||
        !(maximum_heading_error_ > 0.0) ||
        !(maximum_wheel_command_ > speed_) ||
        startup_ramp_seconds_ < 0.0 || startup_ramp_seconds_ > 5.0 ||
        !(minimum_wheel_command_scale_ > 0.0) ||
        minimum_wheel_command_scale_ > 1.0 ||
        formation_longitudinal_gain_ < 0.0 ||
        formation_longitudinal_gain_ > 2.0 ||
        formation_lateral_gain_ < 0.0 ||
        formation_lateral_gain_ > 2.0 ||
        formation_heading_gain_ < 0.0 ||
        formation_heading_gain_ > 2.0 ||
        required_subscribers_ < 1 || required_subscribers_ > 8 ||
        readiness_stable_samples_ < 1 ||
        initial_command_sequence_ == 0U ||
        initial_command_sequence_ > 0xFFFFFF00U ||
        experiment_id_.empty() || method_id_.empty() || path_id_.empty()) {
      throw std::runtime_error(
          "invalid three-car unloaded bounded-pretest configuration");
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!(wheel_separation_[index] > 0.0) ||
          longitudinal_gain_per_robot_[index] < 0.0 ||
          lateral_gain_per_robot_[index] < 0.0 ||
          heading_gain_per_robot_[index] < 0.0 ||
          angular_feedforward_scale_positive_[index] < 0.75 ||
          angular_feedforward_scale_positive_[index] > 1.25 ||
          angular_feedforward_scale_negative_[index] < 0.75 ||
          angular_feedforward_scale_negative_[index] > 1.25 ||
          curvature_preview_seconds_positive_[index] < 0.0 ||
          curvature_preview_seconds_positive_[index] > 0.20 ||
          curvature_preview_seconds_negative_[index] < 0.0 ||
          curvature_preview_seconds_negative_[index] > 0.20) {
        throw std::runtime_error(
            "invalid per-robot tracker parameters for agv" +
            std::to_string(index + 1U));
      }
    }
  }

  void initialiseGeometry() {
    path_.reset(new SCurvePath(loadPathConfig()));
    geometry_.reset(new SupportGeometry(
        *path_, loadGeometryConfig()));
    tracker_.reset(new PlanarSupportTracker(
        *geometry_, loadTrackerConfig()));
    if (target_progress_ > path_->length() ||
        maximum_motion_time_ < target_progress_ / speed_ + 1.0) {
      throw std::runtime_error(
          "bounded motion target or timeout is inconsistent with the path");
    }
  }

  void rejectCompetingPublishers() const {
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const std::string topic =
          "/agv" + std::to_string(index + 1U) + "/chassis_command";
      const auto publishers = topicPublishers(topic);
      if (!publishers.empty()) {
        throw std::runtime_error(
            "refusing bounded pretest because " + topic +
            " already has publisher(s): " + join(publishers));
      }
    }
    for (const std::string topic :
         {"/multi_agv/path_reference", "/multi_agv/controller_state"}) {
      const auto publishers = topicPublishers(topic);
      if (!publishers.empty()) {
        throw std::runtime_error(
            "refusing bounded pretest because the old central controller "
            "is still active on " + topic + ": " + join(publishers));
      }
    }
  }

  void receiveState(const agv_msgs::CooperativeState::ConstPtr& message) {
    state_ = *message;
    state_receive_time_ = ros::WallTime::now();
    has_state_ = true;
  }

  void receiveFeedback(
      std::size_t index,
      const agv_msgs::ChassisFeedback::ConstPtr& message) {
    if (message->robot_id != index + 1U) {
      return;
    }
    feedback_[index] = *message;
    const ros::WallTime now = ros::WallTime::now();
    feedback_receive_time_[index] = now;
    if (!has_feedback_packet_[index] ||
        message->packet_seq != last_feedback_packet_sequence_[index]) {
      last_feedback_packet_sequence_[index] = message->packet_seq;
      feedback_packet_progress_time_[index] = now;
      has_feedback_packet_[index] = true;
    }
    has_feedback_[index] = true;
  }

  bool inputsFresh(std::string* reason = nullptr) const {
    const ros::WallTime now = ros::WallTime::now();
    if (!has_state_) {
      if (reason != nullptr) {
        *reason = "cooperative state has not been received";
      }
      return false;
    }
    const double state_age = (now - state_receive_time_).toSec();
    if (state_age > state_timeout_) {
      if (reason != nullptr) {
        *reason = "cooperative state receive age " +
                  std::to_string(state_age) + " s exceeds " +
                  std::to_string(state_timeout_) + " s";
      }
      return false;
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const std::string robot = "agv" + std::to_string(index + 1U);
      if (!has_feedback_[index]) {
        if (reason != nullptr) {
          *reason = robot + " chassis feedback has not been received";
        }
        return false;
      }
      const double feedback_age =
          (now - feedback_receive_time_[index]).toSec();
      if (feedback_age > maximum_feedback_receive_age_) {
        if (reason != nullptr) {
          *reason = robot + " chassis feedback receive age " +
                    std::to_string(feedback_age) + " s exceeds " +
                    std::to_string(maximum_feedback_receive_age_) + " s";
        }
        return false;
      }
      if (transport_type_ == "serial") {
        if (!has_feedback_packet_[index]) {
          if (reason != nullptr) {
            *reason = robot + " STM32 packet sequence has not been received";
          }
          return false;
        }
        const double packet_age =
            (now - feedback_packet_progress_time_[index]).toSec();
        if (packet_age > maximum_serial_feedback_age_) {
          if (reason != nullptr) {
            *reason = robot + " STM32 packet sequence age " +
                      std::to_string(packet_age) + " s exceeds " +
                      std::to_string(maximum_serial_feedback_age_) + " s";
          }
          return false;
        }
      }
    }
    return true;
  }

  bool commandSubscribersReady() const {
    return std::all_of(
        command_publishers_.begin(), command_publishers_.end(),
        [this](const ros::Publisher& publisher) {
          return publisher.getNumSubscribers() >=
              static_cast<std::uint32_t>(required_subscribers_);
        });
  }

  bool oneStatePublisher() const {
    return topicPublishers("/multi_agv/cooperative_state").size() == 1U;
  }

  bool waitForReady(ros::Rate& rate) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(subscriber_wait_);
    int stable_samples = 0;
    std::string reason = "waiting for initial inputs";
    while (ros::ok() && !stop_requested.load() &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      if (inputsFresh(&reason) && commandSubscribersReady() &&
          oneStatePublisher() && safeToMove(&reason, true)) {
        ++stable_samples;
      } else {
        stable_samples = 0;
      }
      if (stable_samples >= readiness_stable_samples_) {
        ROS_INFO("Three-car bounded pretest ready: command subscribers "
                 "agv1=%u agv2=%u agv3=%u, stable_samples=%d",
                 command_publishers_[0].getNumSubscribers(),
                 command_publishers_[1].getNumSubscribers(),
                 command_publishers_[2].getNumSubscribers(),
                 stable_samples);
        return true;
      }
      rate.sleep();
    }
    ROS_ERROR("Three-car bounded pretest readiness timeout: fresh=%s, "
              "subscribers=[%u,%u,%u], cooperative_state_publishers=%zu",
              inputsFresh() ? "true" : "false",
              command_publishers_[0].getNumSubscribers(),
              command_publishers_[1].getNumSubscribers(),
              command_publishers_[2].getNumSubscribers(),
              topicPublishers("/multi_agv/cooperative_state").size());
    ROS_ERROR("Last readiness rejection: %s; stable_samples=%d/%d",
              reason.c_str(), stable_samples, readiness_stable_samples_);
    return false;
  }

  double stampSpread() const {
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (const auto& stamp : state_.robot_pose_stamp) {
      if (stamp.isZero()) return std::numeric_limits<double>::infinity();
      minimum = std::min(minimum, stamp.toSec());
      maximum = std::max(maximum, stamp.toSec());
    }
    return maximum - minimum;
  }

  bool allStopped() const {
    for (const auto& value : feedback_) {
      if (std::abs(value.wheel_linear_velocity_left_raw) >
              stopped_wheel_tolerance_ ||
          std::abs(value.wheel_linear_velocity_right_raw) >
              stopped_wheel_tolerance_ ||
          std::abs(value.wheel_linear_velocity_left_applied) >
              stopped_wheel_tolerance_ ||
          std::abs(value.wheel_linear_velocity_right_applied) >
              stopped_wheel_tolerance_ ||
          std::abs(value.wheel_linear_velocity_left_actual) >
              stopped_wheel_tolerance_ ||
          std::abs(value.wheel_linear_velocity_right_actual) >
              stopped_wheel_tolerance_) {
        return false;
      }
    }
    return true;
  }

  bool safeToMove(std::string* reason, bool require_stopped) const {
    if (!inputsFresh(reason)) {
      return false;
    }
    if (!oneStatePublisher()) {
      *reason = "cooperative state must have exactly one publisher";
      return false;
    }
    if (!state_.load_pose_valid || !state_.load_path_state_valid) {
      *reason = "virtual load pose or path state is invalid";
      return false;
    }
    const bool camera_fused_profile =
        validation_profile_ == "camera_fused_s_1p00" ||
        validation_profile_ == "camera_fused_straight_0p30" ||
        validation_profile_ == "camera_fused_circle_r0p5_cw_smooth";
    if (camera_fused_profile && transport_type_ == "serial" &&
        (state_.header.frame_id != "three_car_path" ||
         state_.load_localization_source !=
             agv_msgs::CooperativeState::SOURCE_FUSED)) {
      *reason = "camera-fused validation requires a fused virtual load in three_car_path";
      return false;
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!state_.robot_pose_valid[index] ||
          !state_.support_pose_valid[index] ||
          !state_.path_state_valid[index]) {
        *reason = "robot/support/path validity dropped for agv" +
                  std::to_string(index + 1U);
        return false;
      }
      if (camera_fused_profile && transport_type_ == "serial" &&
          state_.robot_localization_source[index] !=
              agv_msgs::CooperativeState::SOURCE_FUSED) {
        *reason = "camera-fused localization authority dropped for agv" +
                  std::to_string(index + 1U);
        return false;
      }
      if (transport_type_ == "serial" &&
          (!std::isfinite(feedback_[index].battery_voltage) ||
           feedback_[index].battery_voltage < minimum_battery_voltage_)) {
        *reason = "battery voltage below bound for agv" +
                  std::to_string(index + 1U);
        return false;
      }
      if (transport_type_ == "serial" &&
          feedback_[index].control_loop_overrun) {
        *reason = "control loop overrun reported by agv" +
                  std::to_string(index + 1U);
        return false;
      }
      if (transport_type_ == "serial") {
        const double serial_age =
            (ros::Time::now() -
             feedback_[index].serial_receive_stamp).toSec();
        // This stamp is generated on another computer. Local receive age and
        // packet-sequence progress are already enforced by inputsFresh() and
        // are the authoritative safety checks. Keep the cross-host stamp as
        // a clock/transport diagnostic so NTP correction or packet reordering
        // cannot independently abort an otherwise fresh feedback stream.
        if (feedback_[index].serial_receive_stamp.isZero() ||
            !std::isfinite(serial_age) ||
            serial_age < -maximum_stamp_spread_ ||
            serial_age > maximum_serial_feedback_age_) {
          ROS_WARN_THROTTLE(
              1.0,
              "agv%zu remote STM32 timestamp diagnostic only: serial_age=%.6f s, allowed=[-%.6f, %.6f] s, zero=%s; local receive and packet-progress gates remain authoritative",
              index + 1U, serial_age, maximum_stamp_spread_,
              maximum_serial_feedback_age_,
              feedback_[index].serial_receive_stamp.isZero() ? "true" :
                                                                "false");
        }
      }
    }
    if (stampSpread() > maximum_stamp_spread_) {
      *reason = "three-car localization timestamp spread exceeds bound";
      return false;
    }
    if (require_stopped && !allStopped()) {
      *reason = "all six wheels must be stopped before motion";
      return false;
    }
    if (require_stopped &&
        std::abs(state_.load_s_actual) > maximum_initial_progress_) {
      *reason = "virtual load is not at the frozen path start";
      return false;
    }
    return true;
  }

  void synchroniseCommandSequences() {
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (feedback_[index].command_seq_applied >= 0xFFFFFF00U) {
        throw std::runtime_error(
            "command sequence is too close to uint32 exhaustion for agv" +
            std::to_string(index + 1U));
      }
      command_sequence_[index] = std::max(
          initial_command_sequence_,
          feedback_[index].command_seq_applied + 1U);
    }
  }

  std::array<PlanarTrackingResult, kRobotCount> trackingAt(
      double progress, double velocity) const {
    std::array<PlanarPose, kRobotCount> robot;
    std::array<PlanarPose, kRobotCount> support;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      robot[index] = poseValue(state_.robot_pose[index]);
      support[index] = poseValue(state_.support_pose[index]);
    }
    auto result = tracker_->trackFleet(progress, velocity, robot, support);

    // Separate rigid fleet motion from deformation of the triangle.  The
    // ordinary per-robot tracker remains responsible for following the
    // virtual-load path.  These centred errors therefore cannot translate or
    // rotate the fleet as a whole; they only tighten relative formation.
    Eigen::Vector2d mean_world_error = Eigen::Vector2d::Zero();
    double heading_error_sine_sum = 0.0;
    double heading_error_cosine_sum = 0.0;
    std::size_t valid_count = 0U;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!result[index].valid) continue;
      mean_world_error +=
          result[index].chassis_pose_reference.position -
          robot[index].position;
      heading_error_sine_sum += std::sin(result[index].heading_error);
      heading_error_cosine_sum += std::cos(result[index].heading_error);
      ++valid_count;
    }
    if (valid_count > 0U) {
      mean_world_error /= static_cast<double>(valid_count);
    }
    const double mean_heading_error =
        std::atan2(heading_error_sine_sum, heading_error_cosine_sum);

    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!result[index].valid) continue;
      double feedforward = result[index].angular_velocity_feedforward;
      const double preview_seconds =
          feedforward > 0.0
              ? curvature_preview_seconds_positive_[index]
              : (feedforward < 0.0
                     ? curvature_preview_seconds_negative_[index]
                     : 0.0);
      if (velocity != 0.0 && preview_seconds > 0.0) {
        const double preview_progress = std::min(
            path_->length(), progress +
                std::abs(velocity) * preview_seconds);
        const auto preview = tracker_->trackFleet(
            preview_progress, velocity, robot, support);
        if (preview[index].valid) {
          feedforward = preview[index].angular_velocity_feedforward;
        }
      }
      const double directional_scale =
          feedforward > 0.0
              ? angular_feedforward_scale_positive_[index]
              : (feedforward < 0.0
                     ? angular_feedforward_scale_negative_[index]
                     : 1.0);
      result[index].angular_velocity_feedforward =
          directional_scale * feedforward;

      const Eigen::Vector2d relative_world_error =
          result[index].chassis_pose_reference.position -
          robot[index].position - mean_world_error;
      const double c = std::cos(robot[index].yaw);
      const double s = std::sin(robot[index].yaw);
      const double relative_longitudinal_error =
          c * relative_world_error.x() + s * relative_world_error.y();
      const double relative_lateral_error =
          -s * relative_world_error.x() + c * relative_world_error.y();
      const double relative_heading_error = std::atan2(
          std::sin(result[index].heading_error - mean_heading_error),
          std::cos(result[index].heading_error - mean_heading_error));

      result[index].linear_velocity_raw =
          result[index].linear_velocity_feedforward *
              std::cos(result[index].heading_error) +
          longitudinal_gain_per_robot_[index] *
              result[index].longitudinal_error +
          formation_longitudinal_gain_ * relative_longitudinal_error;
      result[index].angular_velocity_raw =
          result[index].angular_velocity_feedforward +
          lateral_gain_per_robot_[index] * result[index].lateral_error +
          heading_gain_per_robot_[index] *
              std::sin(result[index].heading_error) +
          formation_lateral_gain_ * relative_lateral_error +
          formation_heading_gain_ * std::sin(relative_heading_error);
      const double half_track = 0.5 * wheel_separation_[index];
      result[index].wheel_linear_velocity_left_raw =
          result[index].linear_velocity_raw -
          half_track * result[index].angular_velocity_raw;
      result[index].wheel_linear_velocity_right_raw =
          result[index].linear_velocity_raw +
          half_track * result[index].angular_velocity_raw;
      result[index].valid =
          std::isfinite(result[index].wheel_linear_velocity_left_raw) &&
          std::isfinite(result[index].wheel_linear_velocity_right_raw);
    }
    return result;
  }

  bool limitTrackingCommands(
      std::array<PlanarTrackingResult, kRobotCount>* tracking,
      double* applied_scale, std::string* reason) const {
    double peak_wheel_command = 0.0;
    for (const auto& value : *tracking) {
      if (!value.valid) continue;
      peak_wheel_command = std::max(
          peak_wheel_command,
          std::max(std::abs(value.wheel_linear_velocity_left_raw),
                   std::abs(value.wheel_linear_velocity_right_raw)));
    }
    *applied_scale = 1.0;
    if (peak_wheel_command <= maximum_wheel_command_) return true;

    // Use one factor for all six wheels. Independent clipping would change
    // wheel curvature and relative robot speeds, deforming the support
    // triangle. Keep a small numerical margin below the hard limit.
    const double scale =
        0.999 * maximum_wheel_command_ / peak_wheel_command;
    if (!std::isfinite(scale) || scale < minimum_wheel_command_scale_) {
      std::ostringstream message;
      message << "wheel command requires excessive fleet scaling"
              << " (peak=" << peak_wheel_command
              << ", scale=" << scale << ")";
      *reason = message.str();
      return false;
    }
    for (auto& value : *tracking) {
      value.linear_velocity_feedforward *= scale;
      value.angular_velocity_feedforward *= scale;
      value.linear_velocity_raw *= scale;
      value.angular_velocity_raw *= scale;
      value.wheel_linear_velocity_left_raw *= scale;
      value.wheel_linear_velocity_right_raw *= scale;
    }
    *applied_scale = scale;
    ROS_WARN_THROTTLE(
        1.0,
        "Uniform three-car wheel-command scaling active: raw_peak=%.6f m/s scale=%.4f hard_limit=%.6f m/s",
        peak_wheel_command, scale, maximum_wheel_command_);
    return true;
  }

  bool trackingSafe(
      const std::array<PlanarTrackingResult, kRobotCount>& tracking,
      double progress, std::string* reason) const {
    const auto minimum_actual =
        *std::min_element(state_.s_actual.begin(), state_.s_actual.end());
    const auto maximum_actual =
        *std::max_element(state_.s_actual.begin(), state_.s_actual.end());
    if (maximum_actual - minimum_actual > maximum_progress_spread_) {
      *reason = "robot path-progress spread exceeds bound";
      return false;
    }
    if (std::abs(state_.load_s_actual - progress) >
        maximum_reference_error_) {
      *reason = "virtual load progress error exceeds bound";
      return false;
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const auto& value = tracking[index];
      if (!value.valid) {
        *reason = "invalid planar tracking result for agv" +
                  std::to_string(index + 1U);
        return false;
      }
      if (std::abs(value.longitudinal_error) >
              maximum_longitudinal_error_ ||
          std::abs(value.lateral_error) > maximum_lateral_error_ ||
          std::abs(value.heading_error) > maximum_heading_error_) {
        std::ostringstream message;
        message << "tracking error exceeds bound for agv" << index + 1U
                << " (longitudinal=" << value.longitudinal_error
                << ", lateral=" << value.lateral_error
                << ", heading=" << value.heading_error << ")";
        *reason = message.str();
        return false;
      }
      if (std::abs(value.wheel_linear_velocity_left_raw) >
              maximum_wheel_command_ ||
          std::abs(value.wheel_linear_velocity_right_raw) >
              maximum_wheel_command_) {
        *reason = "wheel command exceeds bound for agv" +
                  std::to_string(index + 1U);
        return false;
      }
    }
    return true;
  }

  agv_msgs::ChassisCommand commandMessage(
      std::size_t index, const ros::Time& stamp,
      const PlanarTrackingResult* tracking) {
    agv_msgs::ChassisCommand command;
    command.header.stamp = stamp;
    command.robot_id = static_cast<std::uint8_t>(index + 1U);
    command.command_seq = ++command_sequence_[index];
    command.control_mode = 1U;
    command.experiment_id = experiment_id_;
    command.method_id =
        tracking == nullptr ? method_id_ + "_STOP" : method_id_;
    if (tracking != nullptr) {
      command.linear_velocity_reference = tracking->linear_velocity_raw;
      command.angular_velocity_reference = tracking->angular_velocity_raw;
      command.wheel_linear_velocity_left_raw =
          tracking->wheel_linear_velocity_left_raw;
      command.wheel_linear_velocity_right_raw =
          tracking->wheel_linear_velocity_right_raw;
    }
    return command;
  }

  void publishTracking(
      const std::array<PlanarTrackingResult, kRobotCount>& tracking) {
    const ros::Time stamp = ros::Time::now();
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      command_publishers_[index].publish(
          commandMessage(index, stamp, &tracking[index]));
    }
  }

  void publishStop() {
    const ros::Time stamp = ros::Time::now();
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      command_publishers_[index].publish(
          commandMessage(index, stamp, nullptr));
    }
  }

  void publishRepeatedStop() {
    for (int repeat = 0; repeat < 10; ++repeat) {
      if (!ros::master::check()) break;
      publishStop();
      ros::spinOnce();
      ros::WallDuration(0.05).sleep();
    }
  }

  void publishReferenceAndController(
      const std::array<PlanarTrackingResult, kRobotCount>& tracking,
      double progress, double velocity) {
    const ros::Time stamp = ros::Time::now();
    agv_msgs::PathReference reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = state_.header.frame_id;
    reference.path_id = path_id_;
    reference.path_version = 1U;
    reference.load_path_progress_reference = progress;
    reference.load_path_velocity_reference = velocity;
    const auto load = path_->sample(progress);
    reference.load_pose_reference =
        poseMessage({load.position, load.heading});
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      reference.support_pose_reference[index] =
          poseMessage(tracking[index].support_pose_reference);
      reference.chassis_linear_velocity_feedforward[index] =
          tracking[index].linear_velocity_feedforward;
      reference.chassis_angular_velocity_feedforward[index] =
          tracking[index].angular_velocity_feedforward;
    }
    reference_publisher_.publish(reference);

    agv_msgs::ControllerState controller;
    controller.header.stamp = stamp;
    controller.header.frame_id = state_.header.frame_id;
    controller.experiment_id = experiment_id_;
    controller.method_id = method_id_;
    controller.common_load_velocity_reference = velocity;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      controller.path_progress_actual[index] = state_.s_actual[index];
      controller.path_velocity_actual[index] = state_.s_dot_actual[index];
      controller.path_progress_execute_reference[index] = progress;
      controller.path_velocity_execute_reference[index] = velocity;
    }
    controller_publisher_.publish(controller);
  }

  bool waitForStopped(ros::Rate& rate) {
    const ros::WallTime deadline =
        ros::WallTime::now() +
        ros::WallDuration(stop_confirmation_timeout_);
    int consecutive = 0;
    while (ros::ok() && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      publishStop();
      if (inputsFresh() && allStopped()) {
        ++consecutive;
        if (consecutive >= 5) return true;
      } else {
        consecutive = 0;
      }
      rate.sleep();
    }
    return false;
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  ros::Subscriber state_subscriber_;
  std::array<ros::Subscriber, kRobotCount> feedback_subscribers_;
  std::array<ros::Publisher, kRobotCount> command_publishers_;
  ros::Publisher reference_publisher_;
  ros::Publisher controller_publisher_;
  std::unique_ptr<SCurvePath> path_;
  std::unique_ptr<SupportGeometry> geometry_;
  std::unique_ptr<PlanarSupportTracker> tracker_;
  agv_msgs::CooperativeState state_;
  std::array<agv_msgs::ChassisFeedback, kRobotCount> feedback_;
  ros::WallTime state_receive_time_;
  std::array<ros::WallTime, kRobotCount> feedback_receive_time_;
  std::array<ros::WallTime, kRobotCount> feedback_packet_progress_time_;
  std::array<bool, kRobotCount> has_feedback_{{false, false, false}};
  std::array<bool, kRobotCount> has_feedback_packet_{{false, false, false}};
  std::array<std::uint32_t, kRobotCount>
      last_feedback_packet_sequence_{{0U, 0U, 0U}};
  std::array<std::uint32_t, kRobotCount> command_sequence_{{0U, 0U, 0U}};
  bool has_state_{false};
  bool hardware_authorized_{false};
  bool command_authorized_{false};
  bool confirm_readonly_gate_{false};
  bool confirm_area_clear_{false};
  bool confirm_wheels_on_floor_{false};
  bool confirm_unloaded_fixture_{false};
  std::string transport_type_;
  std::string experiment_id_;
  std::string method_id_;
  double publish_rate_{100.0};
  double state_timeout_{0.15};
  double subscriber_wait_{15.0};
  double start_delay_{5.0};
  double stop_confirmation_timeout_{4.0};
  double stopped_wheel_tolerance_{0.01};
  double maximum_step_{0.05};
  double maximum_motion_time_{25.0};
  double speed_{0.05};
  double target_progress_{1.00};
  double startup_ramp_seconds_{1.0};
  double minimum_battery_voltage_{10.0};
  double maximum_feedback_receive_age_{0.25};
  double maximum_serial_feedback_age_{0.25};
  double maximum_stamp_spread_{0.02};
  double maximum_initial_progress_{0.03};
  double maximum_progress_spread_{0.04};
  double maximum_reference_error_{0.05};
  double maximum_longitudinal_error_{0.05};
  double maximum_lateral_error_{0.03};
  double maximum_heading_error_{0.12};
  double maximum_wheel_command_{0.08};
  double minimum_wheel_command_scale_{0.75};
  int required_subscribers_{2};
  int readiness_stable_samples_{20};
  std::uint32_t initial_command_sequence_{41000U};
  std::string path_id_{"s_curve_1m_bounded"};
  std::string validation_profile_{"legacy_unloaded_bounded"};
  std::array<double, kRobotCount> wheel_separation_{{0.114, 0.114, 0.114}};
  std::array<double, kRobotCount> longitudinal_gain_per_robot_{{1.0, 1.0, 1.0}};
  std::array<double, kRobotCount> lateral_gain_per_robot_{{2.0, 2.0, 2.0}};
  std::array<double, kRobotCount> heading_gain_per_robot_{{2.0, 2.0, 2.0}};
  std::array<double, kRobotCount> angular_feedforward_scale_positive_{{1.0, 1.0, 1.0}};
  std::array<double, kRobotCount> angular_feedforward_scale_negative_{{1.0, 1.0, 1.0}};
  double formation_longitudinal_gain_{0.0};
  double formation_lateral_gain_{0.0};
  double formation_heading_gain_{0.0};
  std::array<double, kRobotCount> curvature_preview_seconds_positive_{{
      0.0, 0.0, 0.0}};
  std::array<double, kRobotCount> curvature_preview_seconds_negative_{{
      0.0, 0.0, 0.0}};
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "three_car_unloaded_bounded_pretest",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle result_node;
  const std::string result_parameter =
      "/multi_agv/three_car_unloaded_pretest_result_code";
  result_node.setParam(result_parameter, -1);
  std::signal(SIGINT, multi_agv_control::requestStop);
  std::signal(SIGTERM, multi_agv_control::requestStop);
  std::signal(SIGHUP, multi_agv_control::requestStop);
  int result = 1;
  try {
    multi_agv_control::ThreeCarUnloadedBoundedPretestNode node;
    result = node.run();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to run three-car bounded pretest: %s", error.what());
  }
  result_node.setParam(result_parameter, result);
  ros::shutdown();
  return result;
}
