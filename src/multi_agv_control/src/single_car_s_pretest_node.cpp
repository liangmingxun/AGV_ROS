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
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
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

    if (usesWorldPose()) {
      pose_subscriber_ = node_.subscribe(
          camera_pose_topic_, 10,
          &SingleCarSPretestNode::receiveCameraPose, this);
      calibration_epoch_subscriber_ = node_.subscribe(
          "/vision/aruco/calibration_epoch", 2,
          &SingleCarSPretestNode::receiveCalibrationEpoch, this);
    } else {
      pose_subscriber_ = node_.subscribe(
          topic_prefix_ + "/odom", 10,
          &SingleCarSPretestNode::receiveOdometry, this);
    }
    feedback_subscriber_ = node_.subscribe(
        topic_prefix_ + "/chassis_feedback", 10,
        &SingleCarSPretestNode::receiveFeedback, this);
    command_publisher_ = node_.advertise<agv_msgs::ChassisCommand>(
        topic_prefix_ + "/chassis_command", 1, false);
    path_publisher_ = node_.advertise<nav_msgs::Path>(
        topic_prefix_ + "/s_pretest/reference_path", 1, true);
    reference_publisher_ = node_.advertise<geometry_msgs::PoseStamped>(
        topic_prefix_ + "/s_pretest/chassis_reference", 5, false);
    result_publisher_ = node_.advertise<std_msgs::String>(
        topic_prefix_ + "/s_pretest/result", 1, true);
  }

  int run() {
    publishResult("RUNNING");
    ros::Rate rate(publish_rate_);
    if (!waitForInputsAndSubscribers(rate)) {
      return abortRun(stop_requested.load() ? 130 : 2,
                      stop_requested.load() ? "ABORTED_SIGNAL"
                                            : "ABORTED_NOT_READY");
    }
    synchroniseCommandSequence();
    initialiseAnchor();
    publishReferencePath();

    ROS_WARN("%s bounded S pretest starts in %.1f seconds: "
             "A=%.3f m, longitudinal=%.3f m, arc_speed=%.3f m/s",
             robot_label_.c_str(), start_delay_, path_->config().amplitude,
             path_->config().longitudinal_length, speed_);
    const ros::WallTime delay_start = ros::WallTime::now();
    while (ros::ok() && !stop_requested.load() &&
           (ros::WallTime::now() - delay_start).toSec() < start_delay_) {
      ros::spinOnce();
      publishStop();
      rate.sleep();
    }
    if (!ros::ok() || stop_requested.load()) {
      return abortRun(130, "ABORTED_SIGNAL");
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
      if (actual_wheel_fault_) {
        ROS_ERROR("%s S pretest aborted by actual-wheel-speed watchdog",
                  robot_label_.c_str());
        return abortRun(8, "ABORTED_OVERSPEED");
      }
      if (battery_fault_) {
        ROS_ERROR("%s S pretest aborted by battery-voltage gate",
                  robot_label_.c_str());
        return abortRun(10, "ABORTED_LOW_BATTERY");
      }
      if (!stateFresh()) {
        ROS_ERROR("%s S pretest aborted: localization or chassis feedback stale",
                  robot_label_.c_str());
        return abortRun(3, "ABORTED_STALE");
      }
      if (dt > 0.0 && dt <= maximum_step_) {
        progress = std::min(path_->length(), progress + speed_ * dt);
      }
      const auto tracking = trackingAt(progress);
      if (!tracking.valid || trackingErrorExceeded(tracking)) {
        ROS_ERROR("%s S pretest aborted: invalid or excessive tracking "
                  "error (longitudinal=%+.3f, lateral=%+.3f, heading=%+.3f)",
                  robot_label_.c_str(), tracking.longitudinal_error,
                  tracking.lateral_error,
                  tracking.heading_error);
        return abortRun(4, "ABORTED_TRACKING_ERROR");
      }
      if (std::abs(tracking.wheel_linear_velocity_left_raw) >
              maximum_wheel_command_ ||
          std::abs(tracking.wheel_linear_velocity_right_raw) >
              maximum_wheel_command_) {
        ROS_ERROR("%s S pretest aborted: requested wheel speed exceeds "
                  "the dedicated %.3f m/s pretest bound (left=%+.3f right=%+.3f)",
                  robot_label_.c_str(), maximum_wheel_command_,
                  tracking.wheel_linear_velocity_left_raw,
                  tracking.wheel_linear_velocity_right_raw);
        return abortRun(5, "ABORTED_COMMAND_LIMIT");
      }
      publishTracking(tracking);
      publishReference(tracking.chassis_pose_reference);
      if ((now - motion_start).toSec() > maximum_motion_time_) {
        ROS_ERROR("%s S pretest aborted: motion timeout", robot_label_.c_str());
        return abortRun(6, "ABORTED_MOTION_TIMEOUT");
      }
      rate.sleep();
    }

    if (!ros::ok() || stop_requested.load()) {
      return abortRun(130, "ABORTED_SIGNAL");
    }
    // The specified experiment trajectory ends here. Do not add an
    // unmodelled low-speed terminal manoeuvre just to satisfy an endpoint
    // accuracy threshold: command zero immediately and retain endpoint error
    // only as an offline metric.
    publishRepeatedStop();
    if (!waitForStopped(rate)) {
      ROS_ERROR("%s S pretest stop was commanded, but stopped feedback "
                "was not confirmed before timeout", robot_label_.c_str());
      return abortRun(7, "ABORTED_STOP_NOT_CONFIRMED");
    }
    if (!recordPassiveStoppedWindow(rate)) {
      ROS_ERROR("%s S pretest passive stopped window was invalid",
                robot_label_.c_str());
      return abortRun(11, "ABORTED_PASSIVE_WINDOW");
    }
    publishResult("STOP_CONFIRMED");
    ros::WallDuration(0.10).sleep();
    ROS_INFO("%s bounded S pretest completed; zero wheel speed confirmed "
             "and %.1f s passive endpoint window recorded",
             robot_label_.c_str(), post_stop_record_seconds_);
    return 0;
  }

 private:
  bool usesWorldPose() const {
    return localization_source_ == "camera" ||
           localization_source_ == "fused";
  }

  std::string activeMethodId() const {
    if (localization_source_ == "fused") return "FUSED_BOUNDED_S";
    if (localization_source_ == "camera") return "CAMERA_BOUNDED_S";
    return "ODOM_BOUNDED_S";
  }

  void loadConfiguration() {
    const std::string root = "single_car_s_pretest/";
    robot_index_ = private_.param("robot_index", 0);
    robot_name_ = "agv" + std::to_string(robot_index_);
    robot_label_ = "Robot" + std::to_string(robot_index_);
    topic_prefix_ = "/" + robot_name_;
    hardware_authorized_ =
        private_.param(root + "hardware_execution_authorized", false);
    command_authorized_ =
        private_.param("command_publication_authorized", false);
    confirm_floor_clear_ = private_.param("confirm_test_area_clear", false);
    confirm_wheels_on_floor_ =
        private_.param("confirm_wheels_on_floor", false);
    transport_type_ =
        private_.param<std::string>("platform_transport_type", "serial");
    localization_source_ =
        private_.param<std::string>("localization_source", "odom");
    camera_pose_topic_ = private_.param<std::string>(
        "camera_pose_topic",
        "/pose_provider/" + robot_name_ + "/base_pose_filtered");

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
    post_stop_record_seconds_ = finiteParam(
        private_, root + "post_stop_record_seconds", 2.0);
    maximum_longitudinal_error_ = finiteParam(
        private_, root + "abort/maximum_longitudinal_error", 0.20);
    maximum_lateral_error_ = finiteParam(
        private_, root + "abort/maximum_lateral_error", 0.12);
    maximum_heading_error_ = finiteParam(
        private_, root + "abort/maximum_heading_error", 0.50);
    maximum_wheel_command_ = finiteParam(
        private_, root + "abort/maximum_wheel_linear_velocity", 0.08);
    minimum_battery_voltage_ = finiteParam(
        private_, root + "abort/minimum_battery_voltage", 10.0);
    const std::string common_watchdog =
        root + "actual_wheel_watchdog/";
    const std::string robot_watchdog =
        root + "robot_overrides/" + robot_name_ +
        "/actual_wheel_watchdog/";
    const auto watchdogParam = [this, &common_watchdog, &robot_watchdog](
                                   const std::string& name,
                                   double fallback) {
      const double common =
          finiteParam(private_, common_watchdog + name, fallback);
      return finiteParam(private_, robot_watchdog + name, common);
    };
    actual_wheel_warning_speed_ = watchdogParam("warning_speed", 0.08);
    actual_wheel_hard_stop_speed_ =
        watchdogParam("hard_stop_speed", 0.09);
    const int common_hard_stop_consecutive_samples = private_.param(
        common_watchdog + "hard_stop_consecutive_samples", 2);
    actual_wheel_hard_stop_consecutive_samples_ = private_.param(
        robot_watchdog + "hard_stop_consecutive_samples",
        common_hard_stop_consecutive_samples);
    actual_wheel_emergency_stop_speed_ =
        watchdogParam("emergency_stop_speed", 0.12);
    actual_wheel_sustained_speed_ =
        watchdogParam("sustained_speed", 0.08);
    actual_wheel_sustained_duration_ =
        watchdogParam("sustained_duration", 0.05);
    const std::string common_geometry = root + "geometry/";
    const std::string robot_geometry =
        root + "robot_overrides/" + robot_name_ + "/geometry/";
    const auto geometryParam = [this, &common_geometry, &robot_geometry](
                                   const std::string& name,
                                   double fallback) {
      const double common =
          finiteParam(private_, common_geometry + name, fallback);
      return finiteParam(private_, robot_geometry + name, common);
    };
    wheel_separation_ = geometryParam("wheel_separation", 0.114);
    support_offset_ = {
        geometryParam("base_to_support_x", -0.01783),
        geometryParam("base_to_support_y", 0.0)};
    const std::string common_tracker = root + "tracker/";
    const std::string robot_tracker =
        root + "robot_overrides/" + robot_name_ + "/tracker/";
    const auto trackerParam = [this, &common_tracker, &robot_tracker](
                                  const std::string& name,
                                  double fallback) {
      const double common =
          finiteParam(private_, common_tracker + name, fallback);
      return finiteParam(private_, robot_tracker + name, common);
    };
    tracker_profile_ = private_.param<std::string>(
        root + "robot_overrides/" + robot_name_ + "/profile", "shared");
    longitudinal_gain_ = trackerParam("longitudinal_gain", 1.0);
    lateral_gain_ = trackerParam("lateral_gain", 2.0);
    heading_gain_ = trackerParam("heading_gain", 2.0);
    angular_feedforward_scale_ =
        trackerParam("angular_feedforward_scale", 1.0);
    angular_feedforward_scale_positive_ = trackerParam(
        "angular_feedforward_scale_positive", angular_feedforward_scale_);
    angular_feedforward_scale_negative_ = trackerParam(
        "angular_feedforward_scale_negative", angular_feedforward_scale_);
    curvature_preview_seconds_ =
        trackerParam("curvature_preview_seconds", 0.0);
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
    if (robot_index_ < 1 || robot_index_ > 3) {
      throw std::runtime_error("robot_index must be 1, 2 or 3");
    }
    if (!command_authorized_) {
      throw std::runtime_error(
          "command publication is disabled; pass enable_commands:=true only "
          "for a supervised single-car floor pretest");
    }
    if (!hardware_authorized_) {
      throw std::runtime_error(
          "dedicated single-car S-pretest configuration is not authorized");
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
    if (localization_source_ != "odom" && localization_source_ != "camera" &&
        localization_source_ != "fused") {
      throw std::runtime_error(
          "localization_source must be odom, camera or fused");
    }
    if (usesWorldPose() && camera_pose_topic_.empty()) {
      throw std::runtime_error("camera_pose_topic must not be empty");
    }
    if (!(amplitude_ > 0.0) || !(longitudinal_length_ > 0.0) ||
        std::abs(speed_ - 0.05) > 1e-12 || path_samples_ < 101U ||
        !(publish_rate_ > 0.0) || !(state_timeout_ > 0.0) ||
        !(subscriber_wait_ > 0.0) || start_delay_ < 0.0 ||
        !(maximum_motion_time_ > 0.0) || !(maximum_step_ > 0.0) ||
        !(post_stop_record_seconds_ >= 1.0) ||
        !(maximum_wheel_command_ > speed_) ||
        !(minimum_battery_voltage_ >= 10.0) ||
        !(actual_wheel_warning_speed_ > 0.0) ||
        !(actual_wheel_hard_stop_speed_ > actual_wheel_warning_speed_) ||
        actual_wheel_hard_stop_consecutive_samples_ < 2 ||
        actual_wheel_hard_stop_consecutive_samples_ > 10 ||
        !(actual_wheel_emergency_stop_speed_ >
          actual_wheel_hard_stop_speed_) ||
        !(actual_wheel_sustained_speed_ > 0.0) ||
        actual_wheel_sustained_speed_ > actual_wheel_hard_stop_speed_ ||
        !(actual_wheel_sustained_duration_ > 0.0) ||
        !(angular_feedforward_scale_ >= 1.0 &&
          angular_feedforward_scale_ <= 1.25) ||
        !(angular_feedforward_scale_positive_ >= 0.75 &&
          angular_feedforward_scale_positive_ <= 1.25) ||
        !(angular_feedforward_scale_negative_ >= 0.75 &&
          angular_feedforward_scale_negative_ <= 1.25) ||
        !(curvature_preview_seconds_ >= 0.0 &&
          curvature_preview_seconds_ <= 0.10) ||
        !(wheel_separation_ > 0.0) || reference_samples_ < 2U ||
        required_subscribers_ < 1 || required_subscribers_ > 8 ||
        command_sequence_ == 0U ||
        command_sequence_ > 0xFFFFFF00U) {
      throw std::runtime_error("invalid single-car S-pretest configuration");
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
    ROS_INFO("%s tracker profile=%s longitudinal=%.3f lateral=%.3f "
             "heading=%.3f feedforward_scale=%.3f positive=%.3f "
             "negative=%.3f preview=%.3f s wheel_separation=%.6f m",
             robot_label_.c_str(), tracker_profile_.c_str(),
             longitudinal_gain_, lateral_gain_, heading_gain_,
             angular_feedforward_scale_,
             angular_feedforward_scale_positive_,
             angular_feedforward_scale_negative_,
             curvature_preview_seconds_, wheel_separation_);
    ROS_INFO("%s actual-wheel watchdog warning=%.3f hard=%.3f "
             "confirmation=%d samples emergency=%.3f sustained=%.3f/%.3f s",
             robot_label_.c_str(), actual_wheel_warning_speed_,
             actual_wheel_hard_stop_speed_,
             actual_wheel_hard_stop_consecutive_samples_,
             actual_wheel_emergency_stop_speed_,
             actual_wheel_sustained_speed_, actual_wheel_sustained_duration_);
    if (maximum_motion_time_ < path_->length() / speed_ + 1.0) {
      throw std::runtime_error(
          "maximum_motion_time is too short for the configured S path");
    }
  }

  void rejectCompetingPublisher() const {
    std::vector<std::string> publishers;
    const std::string command_topic = topic_prefix_ + "/chassis_command";
    if (topicHasPublisher(command_topic, &publishers)) {
      std::string names;
      for (const auto& publisher : publishers) {
        if (!names.empty()) names += ", ";
        names += publisher;
      }
      throw std::runtime_error(
          "refusing " + robot_label_ + " S pretest because " +
          command_topic + " already has publisher(s): " + names);
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
      ROS_ERROR_THROTTLE(1.0, "Rejected %s odometry: %s",
                         robot_label_.c_str(), error.what());
      has_odometry_ = false;
    }
  }

  void receiveCameraPose(
      const geometry_msgs::PoseStamped::ConstPtr& message) {
    try {
      if (message->header.frame_id.rfind("world@", 0U) != 0U) {
        throw std::runtime_error(
            "camera pose frame must carry the world@epoch token");
      }
      if (camera_frame_id_.empty()) {
        camera_frame_id_ = message->header.frame_id;
      } else if (message->header.frame_id != camera_frame_id_) {
        camera_epoch_fault_ = true;
        throw std::runtime_error(
            "camera world frame changed during the bounded run");
      }
      odometry_pose_.position = {
          message->pose.position.x, message->pose.position.y};
      odometry_pose_.yaw = yawFromQuaternion(message->pose.orientation);
      odometry_receive_time_ = ros::WallTime::now();
      has_odometry_ = std::isfinite(odometry_pose_.position.x()) &&
                      std::isfinite(odometry_pose_.position.y()) &&
                      !camera_epoch_fault_;
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "Rejected %s camera pose: %s",
                         robot_label_.c_str(), error.what());
      has_odometry_ = false;
    }
  }

  void receiveCalibrationEpoch(
      const std_msgs::UInt64::ConstPtr& message) {
    if (message->data == 0U) {
      camera_epoch_fault_ = true;
      return;
    }
    if (!has_calibration_epoch_) {
      calibration_epoch_ = message->data;
      has_calibration_epoch_ = true;
    } else if (message->data != calibration_epoch_) {
      camera_epoch_fault_ = true;
      ROS_ERROR("%s camera S pretest detected a calibration epoch change",
                robot_label_.c_str());
    }
  }

  void receiveFeedback(
      const agv_msgs::ChassisFeedback::ConstPtr& message) {
    if (message->robot_id != static_cast<std::uint8_t>(robot_index_)) {
      return;
    }
    feedback_ = *message;
    feedback_receive_time_ = ros::WallTime::now();
    has_feedback_ = true;
    if (!std::isfinite(message->battery_voltage) ||
        message->battery_voltage < minimum_battery_voltage_) {
      if (!battery_fault_) {
        ROS_ERROR("%s battery voltage %.3f V is below %.3f V",
                  robot_label_.c_str(), message->battery_voltage,
                  minimum_battery_voltage_);
      }
      battery_fault_ = true;
    }
    monitorActualWheelSpeed(*message, feedback_receive_time_);
  }

  void monitorActualWheelSpeed(
      const agv_msgs::ChassisFeedback& message, const ros::WallTime& now) {
    const double maximum_actual = std::max(
        std::abs(message.wheel_linear_velocity_left_actual),
        std::abs(message.wheel_linear_velocity_right_actual));
    if (!std::isfinite(maximum_actual)) {
      actual_wheel_fault_ = true;
      ROS_ERROR("%s actual wheel feedback is not finite",
                robot_label_.c_str());
      return;
    }
    if (maximum_actual > actual_wheel_warning_speed_) {
      ROS_WARN_THROTTLE(
          0.5, "%s actual wheel speed warning: %.4f m/s",
          robot_label_.c_str(), maximum_actual);
    }
    if (maximum_actual > actual_wheel_emergency_stop_speed_) {
      actual_wheel_fault_ = true;
      ROS_ERROR("%s actual wheel speed %.4f exceeds immediate emergency "
                "stop %.4f m/s",
                robot_label_.c_str(), maximum_actual,
                actual_wheel_emergency_stop_speed_);
      return;
    }
    if (maximum_actual > actual_wheel_hard_stop_speed_) {
      ++actual_wheel_hard_stop_consecutive_count_;
      if (actual_wheel_hard_stop_consecutive_count_ >=
          actual_wheel_hard_stop_consecutive_samples_) {
        actual_wheel_fault_ = true;
        ROS_ERROR("%s actual wheel speed %.4f exceeded hard stop %.4f m/s "
                  "for %d consecutive samples",
                  robot_label_.c_str(), maximum_actual,
                  actual_wheel_hard_stop_speed_,
                  actual_wheel_hard_stop_consecutive_count_);
        return;
      }
      ROS_WARN_THROTTLE(
          0.5, "%s isolated hard-threshold wheel-speed sample %.4f m/s "
          "(%d/%d); awaiting confirmation",
          robot_label_.c_str(), maximum_actual,
          actual_wheel_hard_stop_consecutive_count_,
          actual_wheel_hard_stop_consecutive_samples_);
    } else {
      actual_wheel_hard_stop_consecutive_count_ = 0;
    }
    // Remote TCPROS delivery can batch several feedback messages. Measuring
    // persistence with callback wall time then turns two samples only a few
    // milliseconds apart at the chassis into an apparent long overspeed.
    // Use the source timestamp and require a genuinely consecutive sequence.
    const double serial_sample_time = message.serial_receive_stamp.toSec();
    const double header_sample_time = message.header.stamp.toSec();
    double sample_time = now.toSec();
    if (!message.serial_receive_stamp.isZero() &&
        std::isfinite(serial_sample_time)) {
      sample_time = serial_sample_time;
    } else if (!message.header.stamp.isZero() &&
               std::isfinite(header_sample_time)) {
      sample_time = header_sample_time;
    }
    const double maximum_consecutive_gap =
        std::max(0.03, 3.0 / publish_rate_);
    if (maximum_actual > actual_wheel_sustained_speed_) {
      const bool sequence_continues =
          actual_wheel_overspeed_sample_count_ > 0 &&
          sample_time > actual_wheel_overspeed_last_sample_time_ &&
          sample_time - actual_wheel_overspeed_last_sample_time_ <=
              maximum_consecutive_gap;
      if (!sequence_continues) {
        actual_wheel_overspeed_start_time_ = sample_time;
        actual_wheel_overspeed_sample_count_ = 1;
      } else {
        ++actual_wheel_overspeed_sample_count_;
      }
      actual_wheel_overspeed_last_sample_time_ = sample_time;
      const double sustained_time =
          sample_time - actual_wheel_overspeed_start_time_;
      if (actual_wheel_overspeed_sample_count_ >= 2 &&
          sustained_time >= actual_wheel_sustained_duration_) {
        actual_wheel_fault_ = true;
        ROS_ERROR("%s actual wheel speed remained above %.4f m/s for "
                  "%.3f s (%d consecutive source samples)",
                  robot_label_.c_str(), actual_wheel_sustained_speed_,
                  sustained_time, actual_wheel_overspeed_sample_count_);
      }
    } else {
      actual_wheel_overspeed_start_time_ = 0.0;
      actual_wheel_overspeed_last_sample_time_ = 0.0;
      actual_wheel_overspeed_sample_count_ = 0;
    }
  }

  bool stateFresh() const {
    const ros::WallTime now = ros::WallTime::now();
    const bool camera_ready =
        !usesWorldPose() ||
        (has_calibration_epoch_ && !camera_epoch_fault_ &&
         !camera_frame_id_.empty());
    return has_odometry_ && has_feedback_ && camera_ready &&
           (now - odometry_receive_time_).toSec() <= state_timeout_ &&
           (now - feedback_receive_time_).toSec() <= state_timeout_;
  }

  bool feedbackStopped() const {
    return has_feedback_ &&
           std::abs(feedback_.wheel_linear_velocity_left_actual) <=
               stopped_wheel_tolerance_ &&
           std::abs(feedback_.wheel_linear_velocity_right_actual) <=
               stopped_wheel_tolerance_;
  }

  bool waitForInputsAndSubscribers(ros::Rate& rate) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(subscriber_wait_);
    while (ros::ok() && !stop_requested.load() &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      if (stateFresh() && feedbackStopped() && !actual_wheel_fault_ &&
          !battery_fault_ &&
          command_publisher_.getNumSubscribers() >=
              static_cast<std::uint32_t>(required_subscribers_)) {
        ROS_INFO("%s S pretest ready: %u command subscribers connected",
                 robot_label_.c_str(), command_publisher_.getNumSubscribers());
        return true;
      }
      rate.sleep();
    }
    ROS_ERROR("%s S pretest did not start: state fresh=%s stopped=%s "
              "safety_fault=%s command subscribers=%u/%d",
              robot_label_.c_str(), stateFresh() ? "true" : "false",
              feedbackStopped() ? "true" : "false",
              (actual_wheel_fault_ || battery_fault_) ? "true" : "false",
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
    ROS_INFO("%s S pretest expected endpoint in the starting base frame: "
             "x=%+.3f m y=%+.3f m; final heading equals the starting heading",
             robot_label_.c_str(), endpoint_in_starting_base.x(),
             endpoint_in_starting_base.y());
  }

  void synchroniseCommandSequence() {
    if (feedback_.command_seq_applied >= command_sequence_) {
      if (feedback_.command_seq_applied >= 0xFFFFFF00U) {
        throw std::runtime_error(
            robot_label_ +
            " command sequence is too close to uint32 exhaustion; "
            "restart the chassis node before this pretest");
      }
      command_sequence_ = feedback_.command_seq_applied + 1U;
      ROS_WARN("%s S pretest advanced its initial command sequence to %u "
               "to remain newer than the running chassis controller",
               robot_label_.c_str(), command_sequence_);
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
    return trackingAt(progress, speed_);
  }

  PlanarTrackingResult trackingAt(
      double progress, double channel_velocity) const {
    const auto robot = odometryInPathFrame();
    PlanarPose support;
    support.position =
        robot.position + rotate(support_offset_, robot.yaw);
    support.yaw = robot.yaw;
    auto result = tracker_->track(
        {0U, progress, channel_velocity, robot, support});
    if (!result.valid || channel_velocity == 0.0) return result;

    const double preview_progress = std::min(
        path_->length(), progress +
            std::abs(channel_velocity) * curvature_preview_seconds_);
    const auto preview = tracker_->track(
        {0U, preview_progress, channel_velocity, robot, support});
    if (!preview.valid) return result;

    // Optional bounded compensation affects only analytic curvature
    // feedforward. The validated default (scale=1, preview=0) is exactly the
    // original tracker; feedback gains and the frozen path remain unchanged.
    double directional_scale = angular_feedforward_scale_;
    if (preview.angular_velocity_feedforward > 0.0) {
      directional_scale = angular_feedforward_scale_positive_;
    } else if (preview.angular_velocity_feedforward < 0.0) {
      directional_scale = angular_feedforward_scale_negative_;
    }
    const double compensated_feedforward =
        directional_scale * preview.angular_velocity_feedforward;
    result.angular_velocity_raw +=
        compensated_feedforward - result.angular_velocity_feedforward;
    result.angular_velocity_feedforward = compensated_feedforward;
    const double half_track = 0.5 * wheel_separation_;
    result.wheel_linear_velocity_left_raw =
        result.linear_velocity_raw - half_track * result.angular_velocity_raw;
    result.wheel_linear_velocity_right_raw =
        result.linear_velocity_raw + half_track * result.angular_velocity_raw;
    result.valid = std::isfinite(result.wheel_linear_velocity_left_raw) &&
                   std::isfinite(result.wheel_linear_velocity_right_raw);
    return result;
  }

  bool trackingErrorExceeded(const PlanarTrackingResult& value) const {
    return std::abs(value.longitudinal_error) >
               maximum_longitudinal_error_ ||
           std::abs(value.lateral_error) > maximum_lateral_error_ ||
           std::abs(value.heading_error) > maximum_heading_error_;
  }

  void publishTracking(const PlanarTrackingResult& value) {
    publishTrackingMessage(value, activeMethodId());
  }

  void publishTrackingMessage(
      const PlanarTrackingResult& value, const std::string& method_id) {
    agv_msgs::ChassisCommand command;
    command.header.stamp = ros::Time::now();
    command.robot_id = static_cast<std::uint8_t>(robot_index_);
    command.command_seq = ++command_sequence_;
    command.control_mode = 1U;
    command.linear_velocity_reference = value.linear_velocity_raw;
    command.angular_velocity_reference = value.angular_velocity_raw;
    command.wheel_linear_velocity_left_raw =
        value.wheel_linear_velocity_left_raw;
    command.wheel_linear_velocity_right_raw =
        value.wheel_linear_velocity_right_raw;
    command.experiment_id = robot_name_ + "_single_s_pretest";
    command.method_id = method_id;
    command_publisher_.publish(command);
  }

  void publishStop() {
    agv_msgs::ChassisCommand command;
    command.header.stamp = ros::Time::now();
    command.robot_id = static_cast<std::uint8_t>(robot_index_);
    command.command_seq = ++command_sequence_;
    command.control_mode = 1U;
    command.experiment_id = robot_name_ + "_single_s_pretest";
    command.method_id = activeMethodId() + "_STOP";
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

  void publishResult(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    result_publisher_.publish(message);
  }

  int abortRun(int code, const std::string& result) {
    publishResult(result);
    publishRepeatedStop();
    return code;
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

  bool recordPassiveStoppedWindow(ros::Rate& rate) {
    const ros::WallTime deadline = ros::WallTime::now() +
        ros::WallDuration(post_stop_record_seconds_);
    while (ros::ok() && !stop_requested.load() &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      publishStop();
      if (!stateFresh() || actual_wheel_fault_ || battery_fault_ ||
          std::abs(feedback_.wheel_linear_velocity_left_actual) >
              stopped_wheel_tolerance_ ||
          std::abs(feedback_.wheel_linear_velocity_right_actual) >
              stopped_wheel_tolerance_) {
        return false;
      }
      rate.sleep();
    }
    return ros::ok() && !stop_requested.load();
  }

  PlanarPose pathPoseToOdom(const PlanarPose& pose) const {
    return {anchor_translation_ + rotate(pose.position, anchor_yaw_),
            normalizeAngle(pose.yaw + anchor_yaw_)};
  }

  geometry_msgs::PoseStamped poseMessage(const PlanarPose& pose,
                                         const ros::Time& stamp) const {
    geometry_msgs::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = referenceFrame();
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
    message.header.frame_id = referenceFrame();
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
  std::string referenceFrame() const {
    return usesWorldPose() ? camera_frame_id_ : robot_name_ + "/odom";
  }

  ros::Subscriber pose_subscriber_;
  ros::Subscriber calibration_epoch_subscriber_;
  ros::Subscriber feedback_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher reference_publisher_;
  ros::Publisher result_publisher_;
  std::unique_ptr<SCurvePath> path_;
  std::unique_ptr<PlanarSupportTracker> tracker_;

  PlanarPose odometry_pose_;
  agv_msgs::ChassisFeedback feedback_;
  ros::WallTime odometry_receive_time_;
  ros::WallTime feedback_receive_time_;
  bool has_odometry_{false};
  bool has_feedback_{false};
  bool has_calibration_epoch_{false};
  bool camera_epoch_fault_{false};
  bool actual_wheel_fault_{false};
  bool battery_fault_{false};
  bool hardware_authorized_{false};
  bool command_authorized_{false};
  bool confirm_floor_clear_{false};
  bool confirm_wheels_on_floor_{false};
  int robot_index_{0};
  std::string robot_name_;
  std::string robot_label_;
  std::string topic_prefix_;
  std::string transport_type_;
  std::string localization_source_{"odom"};
  std::string camera_pose_topic_;
  std::string camera_frame_id_;
  std::uint64_t calibration_epoch_{0U};
  double actual_wheel_overspeed_start_time_{0.0};
  double actual_wheel_overspeed_last_sample_time_{0.0};
  int actual_wheel_overspeed_sample_count_{0};
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
  double post_stop_record_seconds_{2.0};
  double maximum_longitudinal_error_{0.20};
  double maximum_lateral_error_{0.12};
  double maximum_heading_error_{0.50};
  double maximum_wheel_command_{0.08};
  double minimum_battery_voltage_{10.0};
  double actual_wheel_warning_speed_{0.08};
  double actual_wheel_hard_stop_speed_{0.09};
  int actual_wheel_hard_stop_consecutive_samples_{2};
  int actual_wheel_hard_stop_consecutive_count_{0};
  double actual_wheel_emergency_stop_speed_{0.12};
  double actual_wheel_sustained_speed_{0.08};
  double actual_wheel_sustained_duration_{0.05};
  double wheel_separation_{0.114};
  Eigen::Vector2d support_offset_{-0.01783, 0.0};
  double longitudinal_gain_{1.0};
  double lateral_gain_{2.0};
  double heading_gain_{2.0};
  double angular_feedforward_scale_{1.0};
  double angular_feedforward_scale_positive_{1.0};
  double angular_feedforward_scale_negative_{1.0};
  double curvature_preview_seconds_{0.0};
  std::string tracker_profile_{"shared"};
  std::size_t reference_samples_{10001U};
  int required_subscribers_{2};
  std::uint32_t command_sequence_{31000U};
  double anchor_yaw_{0.0};
  Eigen::Vector2d anchor_translation_{Eigen::Vector2d::Zero()};
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "single_car_s_pretest",
            ros::init_options::NoSigintHandler);
  std::signal(SIGINT, multi_agv_control::requestStop);
  std::signal(SIGTERM, multi_agv_control::requestStop);
  std::signal(SIGHUP, multi_agv_control::requestStop);
  int result = 1;
  try {
    multi_agv_control::SingleCarSPretestNode node;
    result = node.run();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to run single-car S pretest: %s", error.what());
  }
  ros::shutdown();
  return result;
}
