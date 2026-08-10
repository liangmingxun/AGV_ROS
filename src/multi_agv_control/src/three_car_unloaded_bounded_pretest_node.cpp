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
      ros::TransportHints feedback_transport_hints;
      if (transport_type_ == "fake") {
        // Loopback fake tests validate control behavior, not UDPROS. TCP keeps
        // their startup deterministic while the physical path remains
        // UDP-preferred with TCP fallback.
        feedback_transport_hints.reliable().tcpNoDelay();
      } else {
        feedback_transport_hints
            .unreliable()
            .reliable()
            .maxDatagramSize(1400)
            .tcpNoDelay();
      }
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
      if (dt > 0.0 && dt <= maximum_step_) {
        progress = std::min(target_progress_, progress + speed_ * dt);
      }

      const auto tracking = trackingAt(progress, speed_);
      if (!trackingSafe(tracking, progress, &reason)) {
        ROS_ERROR("Three-car bounded pretest aborted: %s", reason.c_str());
        publishRepeatedStop();
        return 6;
      }
      publishTracking(tracking);
      publishReferenceAndController(tracking, progress, speed_);

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
    minimum_battery_voltage_ = finiteParam(
        private_, root + "abort/minimum_battery_voltage", 10.5);
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
  }

  SCurveConfig loadPathConfig() {
    SCurveConfig config;
    private_.param("path_s_curve/amplitude", config.amplitude, 0.05);
    private_.param("path_s_curve/longitudinal_length",
                   config.longitudinal_length, 1.0);
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

    XmlRpc::XmlRpcValue separation;
    if (!private_.getParam(root + "wheel_separation", separation) ||
        separation.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        separation.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error("tracker wheel_separation must have three values");
    }
    XmlRpc::XmlRpcValue offsets;
    if (!private_.getParam(root + "base_to_support", offsets) ||
        offsets.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        offsets.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error(
          "tracker base_to_support must contain three offsets");
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const auto& wheel = separation[static_cast<int>(index)];
      config.wheel_separation[index] =
          wheel.getType() == XmlRpc::XmlRpcValue::TypeInt
              ? static_cast<int>(wheel)
              : static_cast<double>(wheel);
      const auto& offset = offsets[static_cast<int>(index)];
      config.base_to_support[index] =
          {xmlNumber(offset, "x"), xmlNumber(offset, "y")};
    }
    return config;
  }

  void validateAuthorization() const {
    const bool authorized_path_speed =
        (path_id_ == "s_curve_1m_bounded" &&
         std::abs(speed_ - 0.05) <= 1e-12) ||
        (path_id_ == "straight_1m_bounded" &&
         std::abs(speed_ - 0.08) <= 1e-12);
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
        !authorized_path_speed ||
        std::abs(target_progress_ - 1.00) > 1e-12 ||
        !(minimum_battery_voltage_ >= 10.5) ||
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
        required_subscribers_ < 1 || required_subscribers_ > 8 ||
        readiness_stable_samples_ < 1 ||
        initial_command_sequence_ == 0U ||
        initial_command_sequence_ > 0xFFFFFF00U ||
        experiment_id_.empty() || method_id_.empty() || path_id_.empty()) {
      throw std::runtime_error(
          "invalid three-car unloaded bounded-pretest configuration");
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
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!state_.robot_pose_valid[index] ||
          !state_.support_pose_valid[index] ||
          !state_.path_state_valid[index]) {
        *reason = "robot/support/path validity dropped for agv" +
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
        if (feedback_[index].serial_receive_stamp.isZero() ||
            !std::isfinite(serial_age) ||
            serial_age < -maximum_stamp_spread_ ||
            serial_age > maximum_serial_feedback_age_) {
          *reason = "STM32 serial feedback is stale or future-dated for agv" +
                    std::to_string(index + 1U);
          return false;
        }
      }
    }
    if (stampSpread() > maximum_stamp_spread_) {
      *reason = "three-car odometry timestamp spread exceeds bound";
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
    return tracker_->trackFleet(progress, velocity, robot, support);
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
    reference.header.frame_id = "world";
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
    controller.header.frame_id = "world";
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
  double minimum_battery_voltage_{10.5};
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
  int required_subscribers_{2};
  int readiness_stable_samples_{20};
  std::uint32_t initial_command_sequence_{41000U};
  std::string path_id_{"s_curve_1m_bounded"};
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
