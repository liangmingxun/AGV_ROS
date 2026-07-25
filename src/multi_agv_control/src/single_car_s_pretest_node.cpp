#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <agv_msgs/ChassisCommand.h>
#include <agv_msgs/ChassisFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "multi_agv_control/planar_support_tracker.hpp"
#include "multi_agv_control/s_curve_path.hpp"
#include "multi_agv_control/support_geometry.hpp"

namespace multi_agv_control {
namespace {

std::atomic<bool> stop_requested{false};

void requestStop(int) {
  stop_requested.store(true);
}

double finiteParam(const ros::NodeHandle& node, const std::string& name,
                   double fallback) {
  const double value = node.param(name, fallback);
  if (!std::isfinite(value)) {
    throw std::runtime_error(name + " must be finite");
  }
  return value;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& message) {
  tf2::Quaternion quaternion(message.x, message.y, message.z, message.w);
  if (quaternion.length2() <= 1e-12) {
    throw std::runtime_error("odometry orientation quaternion is invalid");
  }
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

Eigen::Vector2d rotate(const Eigen::Vector2d& value, double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * value.x() - s * value.y(),
          s * value.x() + c * value.y()};
}

bool topicHasPublisher(const std::string& topic,
                       std::vector<std::string>* publishers) {
  XmlRpc::XmlRpcValue request;
  request.setSize(1);
  request[0] = ros::this_node::getName();
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  if (!ros::master::execute("getSystemState", request, response, payload,
                            true)) {
    throw std::runtime_error("failed to query ROS master system state");
  }
  if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      payload.size() < 1) {
    throw std::runtime_error("ROS master returned malformed system state");
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
      publishers->push_back(static_cast<std::string>(nodes[node_index]));
    }
  }
  return !publishers->empty();
}

}  // namespace

class SingleCarSPretestNode {
 public:
  SingleCarSPretestNode() : private_("~") {
    loadConfiguration();
    validateAuthorization();
    initialiseGeometry();
    rejectCompetingPublisher();

    odom_subscriber_ = node_.subscribe(
        "/agv1/odom", 10, &SingleCarSPretestNode::receiveOdometry, this);
    feedback_subscriber_ = node_.subscribe(
        "/agv1/chassis_feedback", 10,
        &SingleCarSPretestNode::receiveFeedback, this);
    command_publisher_ = node_.advertise<agv_msgs::ChassisCommand>(
        "/agv1/chassis_command", 1, false);
    path_publisher_ = node_.advertise<nav_msgs::Path>(
        "/agv1/s_pretest/reference_path", 1, true);
    reference_publisher_ = node_.advertise<geometry_msgs::PoseStamped>(
        "/agv1/s_pretest/chassis_reference", 5, false);
  }

  int run() {
    ros::Rate rate(publish_rate_);
    if (!waitForInputsAndSubscribers(rate)) {
      return stop_requested.load() ? 130 : 2;
    }
    synchroniseCommandSequence();
    initialiseAnchor();
    publishReferencePath();

    ROS_WARN("Robot1 bounded S pretest starts in %.1f seconds: "
             "A=%.3f m, longitudinal=%.3f m, arc_speed=%.3f m/s",
             start_delay_, path_->config().amplitude,
             path_->config().longitudinal_length, speed_);
    const ros::WallTime delay_start = ros::WallTime::now();
    while (ros::ok() && !stop_requested.load() &&
           (ros::WallTime::now() - delay_start).toSec() < start_delay_) {
      ros::spinOnce();
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
           progress < path_->length()) {
      ros::spinOnce();
      const ros::WallTime now = ros::WallTime::now();
      const double dt = (now - previous).toSec();
      previous = now;
      if (!stateFresh()) {
        ROS_ERROR("Robot1 S pretest aborted: odometry or chassis feedback stale");
        publishRepeatedStop();
        return 3;
      }
      if (dt > 0.0 && dt <= maximum_step_) {
        progress = std::min(path_->length(), progress + speed_ * dt);
      }
      const auto tracking = trackingAt(progress);
      if (!tracking.valid || trackingErrorExceeded(tracking)) {
        ROS_ERROR("Robot1 S pretest aborted: invalid or excessive tracking "
                  "error (longitudinal=%+.3f, lateral=%+.3f, heading=%+.3f)",
                  tracking.longitudinal_error, tracking.lateral_error,
                  tracking.heading_error);
        publishRepeatedStop();
        return 4;
      }
      if (std::abs(tracking.wheel_linear_velocity_left_raw) >
              maximum_wheel_command_ ||
          std::abs(tracking.wheel_linear_velocity_right_raw) >
              maximum_wheel_command_) {
        ROS_ERROR("Robot1 S pretest aborted: requested wheel speed exceeds "
                  "the dedicated %.3f m/s pretest bound (left=%+.3f right=%+.3f)",
                  maximum_wheel_command_,
                  tracking.wheel_linear_velocity_left_raw,
                  tracking.wheel_linear_velocity_right_raw);
        publishRepeatedStop();
        return 5;
      }
      publishTracking(tracking);
      publishReference(tracking.chassis_pose_reference);
      if ((now - motion_start).toSec() > maximum_motion_time_) {
        ROS_ERROR("Robot1 S pretest aborted: motion timeout");
        publishRepeatedStop();
        return 6;
      }
      rate.sleep();
    }

    publishRepeatedStop();
    if (!waitForStopped(rate)) {
      ROS_ERROR("Robot1 S pretest stop was commanded, but stopped feedback "
                "was not confirmed before timeout");
      return 7;
    }
    ROS_INFO("Robot1 bounded S pretest completed; zero wheel speed confirmed");
    return 0;
  }

 private:
  void loadConfiguration() {
    const std::string root = "single_car_s_pretest/";
    hardware_authorized_ =
        private_.param(root + "hardware_execution_authorized", false);
    command_authorized_ =
        private_.param("command_publication_authorized", false);
    confirm_floor_clear_ = private_.param("confirm_test_area_clear", false);
    confirm_wheels_on_floor_ =
        private_.param("confirm_wheels_on_floor", false);
    transport_type_ =
        private_.param<std::string>("platform_transport_type", "serial");

    amplitude_ = finiteParam(private_, root + "path/amplitude", 0.05);
    longitudinal_length_ = finiteParam(
        private_, root + "path/longitudinal_length", 1.0);
    speed_ = finiteParam(private_, root + "path/arc_length_speed", 0.05);
    int path_samples =
        private_.param(root + "path/lookup_samples", 20001);
    path_samples_ = static_cast<std::size_t>(std::max(path_samples, 0));

    publish_rate_ = finiteParam(private_, root + "publish_rate", 100.0);
    state_timeout_ =
        finiteParam(private_, root + "maximum_state_age", 0.15);
    subscriber_wait_ =
        finiteParam(private_, root + "subscriber_wait_seconds", 10.0);
    start_delay_ = finiteParam(private_, root + "start_delay_seconds", 3.0);
    stop_confirmation_timeout_ = finiteParam(
        private_, root + "stop_confirmation_timeout_seconds", 3.0);
    stopped_wheel_tolerance_ = finiteParam(
        private_, root + "stopped_wheel_tolerance", 0.01);
    maximum_step_ =
        finiteParam(private_, root + "maximum_integration_step", 0.1);
    maximum_motion_time_ =
        finiteParam(private_, root + "maximum_motion_time", 30.0);
    maximum_longitudinal_error_ = finiteParam(
        private_, root + "abort/maximum_longitudinal_error", 0.20);
    maximum_lateral_error_ = finiteParam(
        private_, root + "abort/maximum_lateral_error", 0.12);
    maximum_heading_error_ = finiteParam(
        private_, root + "abort/maximum_heading_error", 0.50);
    maximum_wheel_command_ = finiteParam(
        private_, root + "abort/maximum_wheel_linear_velocity", 0.08);
    wheel_separation_ =
        finiteParam(private_, root + "geometry/wheel_separation", 0.114);
    support_offset_ = {
        finiteParam(private_, root + "geometry/base_to_support_x", -0.01783),
        finiteParam(private_, root + "geometry/base_to_support_y", 0.0)};
    longitudinal_gain_ =
        finiteParam(private_, root + "tracker/longitudinal_gain", 1.0);
    lateral_gain_ =
        finiteParam(private_, root + "tracker/lateral_gain", 2.0);
    heading_gain_ =
        finiteParam(private_, root + "tracker/heading_gain", 2.0);
    int reference_samples =
        private_.param(root + "tracker/chassis_reference_samples", 10001);
    reference_samples_ =
        static_cast<std::size_t>(std::max(reference_samples, 0));
    required_subscribers_ =
        private_.param(root + "required_command_subscribers", 2);
    command_sequence_ =
        static_cast<std::uint32_t>(
            private_.param(root + "initial_command_sequence", 31000));
  }

  void validateAuthorization() const {
    if (!command_authorized_) {
      throw std::runtime_error(
          "command publication is disabled; pass enable_commands:=true only "
          "for the supervised Robot1 floor pretest");
    }
    if (!hardware_authorized_) {
      throw std::runtime_error(
          "dedicated Robot1 S-pretest configuration is not authorized");
    }
    if (transport_type_ == "serial" &&
        (!confirm_floor_clear_ || !confirm_wheels_on_floor_)) {
      throw std::runtime_error(
          "serial execution requires confirm_test_area_clear:=true and "
          "confirm_wheels_on_floor:=true");
    }
    if (transport_type_ != "serial" && transport_type_ != "fake") {
      throw std::runtime_error("platform transport must be serial or fake");
    }
    if (!(amplitude_ > 0.0) || !(longitudinal_length_ > 0.0) ||
        std::abs(speed_ - 0.05) > 1e-12 || path_samples_ < 101U ||
        !(publish_rate_ > 0.0) || !(state_timeout_ > 0.0) ||
        !(subscriber_wait_ > 0.0) || start_delay_ < 0.0 ||
        !(maximum_motion_time_ > 0.0) || !(maximum_step_ > 0.0) ||
        !(maximum_wheel_command_ > speed_) ||
        !(wheel_separation_ > 0.0) || reference_samples_ < 2U ||
        required_subscribers_ < 1 || required_subscribers_ > 8 ||
        command_sequence_ == 0U ||
        command_sequence_ > 0xFFFFFF00U) {
      throw std::runtime_error("invalid Robot1 S-pretest configuration");
    }
  }

  void initialiseGeometry() {
    path_.reset(new SCurvePath(
        {amplitude_, longitudinal_length_, path_samples_}));
    SupportGeometryConfig geometry_config;
    geometry_config.offsets = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    geometry_config.validation_samples = 10001U;
    SupportGeometry geometry(*path_, geometry_config);
    PlanarTrackerConfig tracker_config;
    tracker_config.longitudinal_gain = longitudinal_gain_;
    tracker_config.lateral_gain = lateral_gain_;
    tracker_config.heading_gain = heading_gain_;
    tracker_config.wheel_separation = {
        wheel_separation_, wheel_separation_, wheel_separation_};
    tracker_config.base_to_support = {
        support_offset_, support_offset_, support_offset_};
    tracker_config.chassis_reference_samples = reference_samples_;
    tracker_.reset(new PlanarSupportTracker(
        std::move(geometry), tracker_config));
    if (maximum_motion_time_ < path_->length() / speed_ + 1.0) {
      throw std::runtime_error(
          "maximum_motion_time is too short for the configured S path");
    }
  }

  void rejectCompetingPublisher() const {
    std::vector<std::string> publishers;
    if (topicHasPublisher("/agv1/chassis_command", &publishers)) {
      std::string names;
      for (const auto& publisher : publishers) {
        if (!names.empty()) names += ", ";
        names += publisher;
      }
      throw std::runtime_error(
          "refusing Robot1 S pretest because /agv1/chassis_command already "
          "has publisher(s): " + names);
    }
  }

  void receiveOdometry(const nav_msgs::Odometry::ConstPtr& message) {
    try {
      odometry_pose_.position = {
          message->pose.pose.position.x, message->pose.pose.position.y};
      odometry_pose_.yaw = yawFromQuaternion(message->pose.pose.orientation);
      odometry_receive_time_ = ros::WallTime::now();
      has_odometry_ = std::isfinite(odometry_pose_.position.x()) &&
                      std::isfinite(odometry_pose_.position.y());
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "Rejected Robot1 odometry: %s", error.what());
      has_odometry_ = false;
    }
  }

  void receiveFeedback(
      const agv_msgs::ChassisFeedback::ConstPtr& message) {
    if (message->robot_id != 1U) {
      return;
    }
    feedback_ = *message;
    feedback_receive_time_ = ros::WallTime::now();
    has_feedback_ = true;
  }

  bool stateFresh() const {
    const ros::WallTime now = ros::WallTime::now();
    return has_odometry_ && has_feedback_ &&
           (now - odometry_receive_time_).toSec() <= state_timeout_ &&
           (now - feedback_receive_time_).toSec() <= state_timeout_;
  }

  bool waitForInputsAndSubscribers(ros::Rate& rate) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(subscriber_wait_);
    while (ros::ok() && !stop_requested.load() &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      if (stateFresh() &&
          command_publisher_.getNumSubscribers() >=
              static_cast<std::uint32_t>(required_subscribers_)) {
        ROS_INFO("Robot1 S pretest ready: %u command subscribers connected",
                 command_publisher_.getNumSubscribers());
        return true;
      }
      rate.sleep();
    }
    ROS_ERROR("Robot1 S pretest did not start: state fresh=%s, command "
              "subscribers=%u/%d",
              stateFresh() ? "true" : "false",
              command_publisher_.getNumSubscribers(), required_subscribers_);
    return false;
  }

  void initialiseAnchor() {
    const PlanarPose dummy{{0.0, 0.0}, 0.0};
    const auto initial_reference =
        tracker_->track({0U, 0.0, 0.0, dummy, dummy});
    if (!initial_reference.valid) {
      throw std::runtime_error("failed to construct initial chassis reference");
    }
    anchor_yaw_ = normalizeAngle(
        odometry_pose_.yaw - initial_reference.chassis_pose_reference.yaw);
    anchor_translation_ =
        odometry_pose_.position -
        rotate(initial_reference.chassis_pose_reference.position, anchor_yaw_);
    const auto final_reference =
        tracker_->track({0U, path_->length(), 0.0, dummy, dummy});
    if (!final_reference.valid) {
      throw std::runtime_error("failed to construct final chassis reference");
    }
    const Eigen::Vector2d endpoint_in_starting_base =
        rotate(final_reference.chassis_pose_reference.position -
                   initial_reference.chassis_pose_reference.position,
               -initial_reference.chassis_pose_reference.yaw);
    ROS_INFO("Robot1 S pretest expected endpoint in the starting base frame: "
             "x=%+.3f m y=%+.3f m; final heading equals the starting heading",
             endpoint_in_starting_base.x(), endpoint_in_starting_base.y());
  }

  void synchroniseCommandSequence() {
    if (feedback_.command_seq_applied >= command_sequence_) {
      if (feedback_.command_seq_applied >= 0xFFFFFF00U) {
        throw std::runtime_error(
            "Robot1 command sequence is too close to uint32 exhaustion; "
            "restart the chassis node before this pretest");
      }
      command_sequence_ = feedback_.command_seq_applied + 1U;
      ROS_WARN("Robot1 S pretest advanced its initial command sequence to %u "
               "to remain newer than the running chassis controller",
               command_sequence_);
    }
  }

  PlanarPose odometryInPathFrame() const {
    PlanarPose value;
    value.position =
        rotate(odometry_pose_.position - anchor_translation_, -anchor_yaw_);
    value.yaw = normalizeAngle(odometry_pose_.yaw - anchor_yaw_);
    return value;
  }

  PlanarTrackingResult trackingAt(double progress) const {
    const auto robot = odometryInPathFrame();
    PlanarPose support;
    support.position =
        robot.position + rotate(support_offset_, robot.yaw);
    support.yaw = robot.yaw;
    return tracker_->track(
        {0U, progress, speed_, robot, support});
  }

  bool trackingErrorExceeded(const PlanarTrackingResult& value) const {
    return std::abs(value.longitudinal_error) >
               maximum_longitudinal_error_ ||
           std::abs(value.lateral_error) > maximum_lateral_error_ ||
           std::abs(value.heading_error) > maximum_heading_error_;
  }

  void publishTracking(const PlanarTrackingResult& value) {
    agv_msgs::ChassisCommand command;
    command.header.stamp = ros::Time::now();
    command.robot_id = 1U;
    command.command_seq = ++command_sequence_;
    command.control_mode = 1U;
    command.linear_velocity_reference = value.linear_velocity_raw;
    command.angular_velocity_reference = value.angular_velocity_raw;
    command.wheel_linear_velocity_left_raw =
        value.wheel_linear_velocity_left_raw;
    command.wheel_linear_velocity_right_raw =
        value.wheel_linear_velocity_right_raw;
    command.experiment_id = "robot1_single_s_pretest";
    command.method_id = "ODOM_BOUNDED_S";
    command_publisher_.publish(command);
  }

  void publishStop() {
    agv_msgs::ChassisCommand command;
    command.header.stamp = ros::Time::now();
    command.robot_id = 1U;
    command.command_seq = ++command_sequence_;
    command.control_mode = 1U;
    command.experiment_id = "robot1_single_s_pretest";
    command.method_id = "ODOM_BOUNDED_S_STOP";
    command_publisher_.publish(command);
  }

  void publishRepeatedStop() {
    for (int repeat = 0; repeat < 5; ++repeat) {
      if (!ros::master::check()) break;
      publishStop();
      ros::spinOnce();
      ros::WallDuration(0.05).sleep();
    }
  }

  bool waitForStopped(ros::Rate& rate) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(stop_confirmation_timeout_);
    int consecutive_stopped = 0;
    while (ros::ok() && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      publishStop();
      if (stateFresh() &&
          std::abs(feedback_.wheel_linear_velocity_left_actual) <=
              stopped_wheel_tolerance_ &&
          std::abs(feedback_.wheel_linear_velocity_right_actual) <=
              stopped_wheel_tolerance_) {
        ++consecutive_stopped;
        if (consecutive_stopped >= 5) return true;
      } else {
        consecutive_stopped = 0;
      }
      rate.sleep();
    }
    return false;
  }

  PlanarPose pathPoseToOdom(const PlanarPose& pose) const {
    return {anchor_translation_ + rotate(pose.position, anchor_yaw_),
            normalizeAngle(pose.yaw + anchor_yaw_)};
  }

  geometry_msgs::PoseStamped poseMessage(const PlanarPose& pose,
                                         const ros::Time& stamp) const {
    geometry_msgs::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "agv1/odom";
    message.pose.position.x = pose.position.x();
    message.pose.position.y = pose.position.y();
    message.pose.orientation = quaternionFromYaw(pose.yaw);
    return message;
  }

  void publishReference(const PlanarPose& local_reference) {
    reference_publisher_.publish(
        poseMessage(pathPoseToOdom(local_reference), ros::Time::now()));
  }

  void publishReferencePath() {
    nav_msgs::Path message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "agv1/odom";
    constexpr std::size_t kVisualizationSamples = 501U;
    message.poses.reserve(kVisualizationSamples);
    const PlanarPose dummy{{0.0, 0.0}, 0.0};
    for (std::size_t index = 0; index < kVisualizationSamples; ++index) {
      const double progress =
          path_->length() * static_cast<double>(index) /
          static_cast<double>(kVisualizationSamples - 1U);
      const auto reference =
          tracker_->track({0U, progress, 0.0, dummy, dummy});
      message.poses.push_back(poseMessage(
          pathPoseToOdom(reference.chassis_pose_reference),
          message.header.stamp));
    }
    path_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber feedback_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher reference_publisher_;
  std::unique_ptr<SCurvePath> path_;
  std::unique_ptr<PlanarSupportTracker> tracker_;

  PlanarPose odometry_pose_;
  agv_msgs::ChassisFeedback feedback_;
  ros::WallTime odometry_receive_time_;
  ros::WallTime feedback_receive_time_;
  bool has_odometry_{false};
  bool has_feedback_{false};
  bool hardware_authorized_{false};
  bool command_authorized_{false};
  bool confirm_floor_clear_{false};
  bool confirm_wheels_on_floor_{false};
  std::string transport_type_;
  double amplitude_{0.05};
  double longitudinal_length_{1.0};
  double speed_{0.05};
  std::size_t path_samples_{20001U};
  double publish_rate_{100.0};
  double state_timeout_{0.15};
  double subscriber_wait_{10.0};
  double start_delay_{3.0};
  double stop_confirmation_timeout_{3.0};
  double stopped_wheel_tolerance_{0.01};
  double maximum_step_{0.1};
  double maximum_motion_time_{30.0};
  double maximum_longitudinal_error_{0.20};
  double maximum_lateral_error_{0.12};
  double maximum_heading_error_{0.50};
  double maximum_wheel_command_{0.08};
  double wheel_separation_{0.114};
  Eigen::Vector2d support_offset_{-0.01783, 0.0};
  double longitudinal_gain_{1.0};
  double lateral_gain_{2.0};
  double heading_gain_{2.0};
  std::size_t reference_samples_{10001U};
  int required_subscribers_{2};
  std::uint32_t command_sequence_{31000U};
  double anchor_yaw_{0.0};
  Eigen::Vector2d anchor_translation_{Eigen::Vector2d::Zero()};
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot1_single_s_pretest",
            ros::init_options::NoSigintHandler);
  std::signal(SIGINT, multi_agv_control::requestStop);
  std::signal(SIGTERM, multi_agv_control::requestStop);
  std::signal(SIGHUP, multi_agv_control::requestStop);
  int result = 1;
  try {
    multi_agv_control::SingleCarSPretestNode node;
    result = node.run();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to run Robot1 single-car S pretest: %s", error.what());
  }
  ros::shutdown();
  return result;
}
