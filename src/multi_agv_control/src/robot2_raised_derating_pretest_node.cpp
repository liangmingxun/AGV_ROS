#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <agv_msgs/CapabilityReport.h>
#include <agv_msgs/ChassisFeedback.h>
#include <agv_msgs/DeratingCommand.h>
#include <agv_msgs/ExperimentState.h>
#include <nav_msgs/Odometry.h>
#include <ros/master.h>
#include <ros/ros.h>

#include "multi_agv_control/experiment_supervisor.hpp"

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

double yawFromQuaternion(const geometry_msgs::Quaternion& value) {
  const double norm = value.x * value.x + value.y * value.y +
                      value.z * value.z + value.w * value.w;
  if (!std::isfinite(norm) || norm <= 1e-12) {
    throw std::runtime_error("Robot2 odometry quaternion is invalid");
  }
  return std::atan2(
      2.0 * (value.w * value.z + value.x * value.y),
      1.0 - 2.0 * (value.y * value.y + value.z * value.z));
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

class Robot2RaisedDeratingPretestNode {
 public:
  Robot2RaisedDeratingPretestNode()
      : private_("~"), supervisor_(loadSupervisorConfig()) {
    loadConfiguration();
    validateAuthorization();
    rejectCompetingPublisher();

    odom_subscriber_ = node_.subscribe(
        "/agv2/odom", 10,
        &Robot2RaisedDeratingPretestNode::receiveOdometry, this);
    feedback_subscriber_ = node_.subscribe(
        "/agv2/chassis_feedback", 10,
        &Robot2RaisedDeratingPretestNode::receiveFeedback, this);
    capability_subscriber_ = node_.subscribe(
        "/agv2/capability_report", 10,
        &Robot2RaisedDeratingPretestNode::receiveCapability, this);
    derating_publisher_ = node_.advertise<agv_msgs::DeratingCommand>(
        "/agv2/derating_command", 5, false);
    experiment_publisher_ = node_.advertise<agv_msgs::ExperimentState>(
        "/agv2/raised_derating_pretest/experiment_state", 10, true);
  }

  int run() {
    ros::Rate rate(publish_rate_);
    if (!waitForInputsAndSubscribers(rate)) {
      publishRepeatedRestore();
      return stop_requested.load() ? 130 : 2;
    }

    anchor_x_ = odometry_.pose.pose.position.x;
    anchor_y_ = odometry_.pose.pose.position.y;
    anchor_yaw_ = yawFromQuaternion(odometry_.pose.pose.orientation);
    if (!supervisor_.arm() ||
        !supervisor_.start(ros::Time::now().toSec())) {
      publishRepeatedRestore();
      throw std::runtime_error("failed to arm Robot2 raised derating pretest");
    }

    ROS_INFO("Robot2 raised derating pretest ready; start the bounded "
             "0.05 m/s raised-wheel straight command");
    const ros::WallTime start_deadline =
        ros::WallTime::now() + ros::WallDuration(motion_start_timeout_);
    ros::WallTime motion_start;
    bool motion_started = false;
    int consecutive_stopped = 0;

    while (ros::ok() && !stop_requested.load()) {
      ros::spinOnce();
      const ros::WallTime wall_now = ros::WallTime::now();
      if (!stateFresh()) {
        ROS_ERROR("Robot2 raised derating pretest aborted because odometry, "
                  "feedback or capability became stale");
        publishRepeatedRestore();
        return 3;
      }

      const double progress = actualProgress();
      if (progress < minimum_progress_ || progress > maximum_progress_) {
        ROS_ERROR("Robot2 raised derating pretest progress out of bounds: "
                  "%+.3f m", progress);
        publishRepeatedRestore();
        return 4;
      }

      if (!motion_started && wheelsMoving()) {
        motion_started = true;
        motion_start = wall_now;
        ROS_INFO("Robot2 raised-wheel motion detected; odometry progress is "
                 "the only derating trigger");
      }
      if (!motion_started && wall_now > start_deadline) {
        ROS_ERROR("Robot2 raised derating pretest timed out waiting for motion");
        publishRepeatedRestore();
        return 5;
      }
      if (motion_started &&
          (wall_now - motion_start).toSec() > maximum_motion_time_) {
        ROS_ERROR("Robot2 raised derating pretest exceeded its bounded motion "
                  "time");
        publishRepeatedRestore();
        return 6;
      }

      const auto phase_before = supervisor_.snapshot().phase;
      supervisor_.updateActualProgress(
          progress, true, ros::Time::now().toSec());
      supervisor_.tick(ros::Time::now().toSec());
      const auto phase_after = supervisor_.snapshot().phase;
      if (phase_after != phase_before) {
        logPhaseTransition(phase_after, progress);
      }
      publishTarget();
      publishExperimentState();

      if (phase_after == ExperimentPhase::kFinished) {
        const bool stopped = !wheelsMoving();
        consecutive_stopped = stopped ? consecutive_stopped + 1 : 0;
        if (consecutive_stopped >= 5 && saw_derated_capability_ &&
            saw_restored_capability_) {
          publishRepeatedRestore();
          ROS_INFO("Robot2 raised derating pretest completed: actual-progress "
                   "activation, ramp restoration and zero wheel speed "
                   "confirmed");
          return 0;
        }
      }
      rate.sleep();
    }

    publishRepeatedRestore();
    ROS_WARN("Robot2 raised derating pretest interrupted; nominal derating "
             "ratios were restored");
    return 130;
  }

 private:
  SupervisorConfig loadSupervisorConfig() {
    const std::string root = "robot2_raised_derating_pretest/";
    SupervisorConfig config;
    config.activation_progress = finiteParam(
        private_, root + "activation_progress", 0.30);
    config.restoration_progress = finiteParam(
        private_, root + "restoration_progress", 0.60);
    config.target_speed_ratio_left = finiteParam(
        private_, root + "target_speed_ratio_left", 0.40);
    config.target_speed_ratio_right = finiteParam(
        private_, root + "target_speed_ratio_right", 0.40);
    config.target_acceleration_ratio_left = finiteParam(
        private_, root + "target_acceleration_ratio_left", 0.50);
    config.target_acceleration_ratio_right = finiteParam(
        private_, root + "target_acceleration_ratio_right", 0.50);
    config.target_deceleration_ratio_left = finiteParam(
        private_, root + "target_deceleration_ratio_left", 0.60);
    config.target_deceleration_ratio_right = finiteParam(
        private_, root + "target_deceleration_ratio_right", 0.60);
    config.ramp_down_time =
        finiteParam(private_, root + "ramp_down_time", 0.40);
    config.ramp_up_time =
        finiteParam(private_, root + "ramp_up_time", 0.40);
    const int mode = private_.param(root + "derating_mode", 1);
    if (mode < 0 || mode > 255) {
      throw std::runtime_error("derating mode outside uint8 range");
    }
    config.derating_mode = static_cast<std::uint8_t>(mode);
    return config;
  }

  void loadConfiguration() {
    const std::string root = "robot2_raised_derating_pretest/";
    publish_rate_ = finiteParam(private_, root + "publish_rate", 20.0);
    maximum_state_age_ =
        finiteParam(private_, root + "maximum_state_age", 0.15);
    subscriber_wait_ =
        finiteParam(private_, root + "subscriber_wait_seconds", 10.0);
    motion_start_timeout_ = finiteParam(
        private_, root + "motion_start_timeout_seconds", 30.0);
    maximum_motion_time_ = finiteParam(
        private_, root + "maximum_motion_time_seconds", 18.0);
    stopped_wheel_tolerance_ = finiteParam(
        private_, root + "stopped_wheel_tolerance", 0.01);
    motion_detection_speed_ = finiteParam(
        private_, root + "motion_detection_speed", 0.01);
    minimum_progress_ = finiteParam(
        private_, root + "minimum_forward_progress", -0.02);
    maximum_progress_ = finiteParam(
        private_, root + "maximum_forward_progress", 0.85);
    required_subscribers_ =
        private_.param(root + "required_derating_subscribers", 2);
    const int initial_sequence =
        private_.param(root + "initial_command_sequence", 42000);
    if (initial_sequence < 1 || initial_sequence > 0x7FFFFFF0) {
      throw std::runtime_error("invalid initial derating command sequence");
    }
    sequence_base_ = static_cast<std::uint32_t>(initial_sequence);
    private_.param<std::string>(
        root + "experiment_id", experiment_id_,
        "robot2_raised_derating_pretest");
    private_.param<std::string>(
        root + "method_id", method_id_, "ACTUAL_ODOM_PROGRESS");
    private_.param<std::string>(
        root + "block_id", block_id_, "raised_wheels");
    private_.param<std::string>(
        root + "run_id", run_id_, "robot2_pretest_001");

    if (!(publish_rate_ > 0.0) || !(maximum_state_age_ > 0.0) ||
        !(subscriber_wait_ > 0.0) || !(motion_start_timeout_ > 0.0) ||
        !(maximum_motion_time_ > 0.0) ||
        !(stopped_wheel_tolerance_ >= 0.0) ||
        motion_detection_speed_ < stopped_wheel_tolerance_ ||
        !(maximum_progress_ > minimum_progress_) ||
        required_subscribers_ < 1 || required_subscribers_ > 8) {
      throw std::runtime_error(
          "invalid Robot2 raised derating pretest configuration");
    }
  }

  void validateAuthorization() {
    const std::string root = "robot2_raised_derating_pretest/";
    const bool hardware_authorized =
        private_.param(root + "hardware_execution_authorized", false);
    const bool publication_authorized =
        private_.param("derating_publication_authorized", false);
    const bool wheels_raised =
        private_.param("confirm_wheels_raised", false);
    const std::string transport =
        private_.param<std::string>("platform_transport_type", "fake");
    if (!publication_authorized) {
      throw std::runtime_error(
          "derating publication disabled; pass enable_derating:=true only "
          "for this bounded pretest");
    }
    if (transport != "fake" &&
        !(hardware_authorized && wheels_raised)) {
      throw std::runtime_error(
          "serial Robot2 derating pretest requires hardware authorization "
          "and confirm_wheels_raised:=true");
    }
  }

  void rejectCompetingPublisher() const {
    std::vector<std::string> publishers;
    if (!topicHasPublisher("/agv2/derating_command", &publishers)) {
      return;
    }
    std::string names;
    for (const auto& publisher : publishers) {
      if (!names.empty()) names += ", ";
      names += publisher;
    }
    throw std::runtime_error(
        "refusing Robot2 raised derating pretest because "
        "/agv2/derating_command already has publisher(s): " + names);
  }

  void receiveOdometry(const nav_msgs::Odometry::ConstPtr& message) {
    try {
      yawFromQuaternion(message->pose.pose.orientation);
      if (!std::isfinite(message->pose.pose.position.x) ||
          !std::isfinite(message->pose.pose.position.y)) {
        throw std::runtime_error("Robot2 odometry position is not finite");
      }
      odometry_ = *message;
      odometry_receive_time_ = ros::WallTime::now();
      has_odometry_ = true;
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "Rejected Robot2 odometry: %s", error.what());
      has_odometry_ = false;
    }
  }

  void receiveFeedback(
      const agv_msgs::ChassisFeedback::ConstPtr& message) {
    if (message->robot_id != 2U) return;
    feedback_ = *message;
    feedback_receive_time_ = ros::WallTime::now();
    has_feedback_ = true;
  }

  void receiveCapability(
      const agv_msgs::CapabilityReport::ConstPtr& message) {
    if (message->robot_id != 2U) return;
    capability_ = *message;
    capability_receive_time_ = ros::WallTime::now();
    has_capability_ = true;
    if (message->derating_active && message->derating_ratio <= 0.45) {
      saw_derated_capability_ = true;
    }
    if (saw_derated_capability_ && !message->derating_active &&
        message->derating_ratio >= 0.99) {
      saw_restored_capability_ = true;
    }
  }

  bool stateFresh() const {
    const ros::WallTime now = ros::WallTime::now();
    return has_odometry_ && has_feedback_ && has_capability_ &&
           (now - odometry_receive_time_).toSec() <= maximum_state_age_ &&
           (now - feedback_receive_time_).toSec() <= maximum_state_age_ &&
           (now - capability_receive_time_).toSec() <= maximum_state_age_;
  }

  bool waitForInputsAndSubscribers(ros::Rate& rate) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(subscriber_wait_);
    while (ros::ok() && !stop_requested.load() &&
           ros::WallTime::now() < deadline) {
      ros::spinOnce();
      if (stateFresh() &&
          derating_publisher_.getNumSubscribers() >=
              static_cast<std::uint32_t>(required_subscribers_)) {
        ROS_INFO("Robot2 raised derating pretest ready: %u derating "
                 "subscribers connected",
                 derating_publisher_.getNumSubscribers());
        return true;
      }
      rate.sleep();
    }
    ROS_ERROR("Robot2 raised derating pretest not ready: state fresh=%s, "
              "derating subscribers=%u/%d",
              stateFresh() ? "true" : "false",
              derating_publisher_.getNumSubscribers(), required_subscribers_);
    return false;
  }

  double actualProgress() const {
    const double dx = odometry_.pose.pose.position.x - anchor_x_;
    const double dy = odometry_.pose.pose.position.y - anchor_y_;
    return std::cos(anchor_yaw_) * dx + std::sin(anchor_yaw_) * dy;
  }

  bool wheelsMoving() const {
    return std::abs(feedback_.wheel_linear_velocity_left_actual) >
               motion_detection_speed_ ||
           std::abs(feedback_.wheel_linear_velocity_right_actual) >
               motion_detection_speed_;
  }

  void publishTarget() {
    const auto& target = supervisor_.snapshot().target;
    agv_msgs::DeratingCommand command;
    command.header.stamp = ros::Time::now();
    command.robot_id = 2U;
    command.command_seq = sequence_base_ + target.sequence - 1U;
    command.mode = target.mode;
    command.active = target.active;
    command.target_speed_ratio_left = target.speed_ratio_left;
    command.target_speed_ratio_right = target.speed_ratio_right;
    command.target_accel_ratio_left = target.acceleration_ratio_left;
    command.target_accel_ratio_right = target.acceleration_ratio_right;
    command.target_decel_ratio_left = target.deceleration_ratio_left;
    command.target_decel_ratio_right = target.deceleration_ratio_right;
    command.ramp_down_time = target.ramp_down_time;
    command.ramp_up_time = target.ramp_up_time;
    command.experiment_id = experiment_id_;
    last_command_sequence_ =
        std::max(last_command_sequence_, command.command_seq);
    derating_publisher_.publish(command);
  }

  void publishRestore() {
    agv_msgs::DeratingCommand command;
    command.header.stamp = ros::Time::now();
    command.robot_id = 2U;
    command.command_seq = emergency_restore_sequence_;
    command.mode = 1U;
    command.active = false;
    command.target_speed_ratio_left = 1.0;
    command.target_speed_ratio_right = 1.0;
    command.target_accel_ratio_left = 1.0;
    command.target_accel_ratio_right = 1.0;
    command.target_decel_ratio_left = 1.0;
    command.target_decel_ratio_right = 1.0;
    command.ramp_down_time = 0.40;
    command.ramp_up_time = 0.40;
    command.experiment_id = experiment_id_;
    derating_publisher_.publish(command);
  }

  void publishRepeatedRestore() {
    emergency_restore_sequence_ =
        std::max(last_command_sequence_ + 1U, sequence_base_ + 3U);
    for (int repeat = 0; repeat < 10; ++repeat) {
      if (!ros::master::check()) break;
      publishRestore();
      ros::spinOnce();
      ros::WallDuration(0.05).sleep();
    }
  }

  void publishExperimentState() {
    const auto& snapshot = supervisor_.snapshot();
    agv_msgs::ExperimentState message;
    message.header.stamp = ros::Time::now();
    message.experiment_id = experiment_id_;
    message.method_id = method_id_;
    message.block_id = block_id_;
    message.run_id = run_id_;
    message.phase = static_cast<std::uint8_t>(snapshot.phase);
    message.run_active = snapshot.run_active;
    message.evaluation_active = snapshot.evaluation_active;
    message.manual_abort = snapshot.manual_abort;
    message.abort_reason = snapshot.abort_reason;
    experiment_publisher_.publish(message);
  }

  void logPhaseTransition(ExperimentPhase phase, double progress) const {
    if (phase == ExperimentPhase::kDeratingDown) {
      ROS_INFO("Robot2 actual progress %.3f m crossed activation threshold; "
               "starting derating ramp", progress);
    } else if (phase == ExperimentPhase::kDerated) {
      ROS_INFO("Robot2 derating ramp reached its target");
    } else if (phase == ExperimentPhase::kRestoring) {
      ROS_INFO("Robot2 actual progress %.3f m crossed restoration threshold; "
               "restoring nominal capability", progress);
    } else if (phase == ExperimentPhase::kFinished) {
      ROS_INFO("Robot2 capability restoration ramp completed; waiting for "
               "zero wheel speed");
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  ExperimentSupervisor supervisor_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber feedback_subscriber_;
  ros::Subscriber capability_subscriber_;
  ros::Publisher derating_publisher_;
  ros::Publisher experiment_publisher_;
  nav_msgs::Odometry odometry_;
  agv_msgs::ChassisFeedback feedback_;
  agv_msgs::CapabilityReport capability_;
  ros::WallTime odometry_receive_time_;
  ros::WallTime feedback_receive_time_;
  ros::WallTime capability_receive_time_;
  bool has_odometry_{false};
  bool has_feedback_{false};
  bool has_capability_{false};
  bool saw_derated_capability_{false};
  bool saw_restored_capability_{false};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_yaw_{0.0};
  double publish_rate_{20.0};
  double maximum_state_age_{0.15};
  double subscriber_wait_{10.0};
  double motion_start_timeout_{30.0};
  double maximum_motion_time_{18.0};
  double stopped_wheel_tolerance_{0.01};
  double motion_detection_speed_{0.01};
  double minimum_progress_{-0.02};
  double maximum_progress_{0.85};
  int required_subscribers_{2};
  std::uint32_t sequence_base_{42000U};
  std::uint32_t last_command_sequence_{0U};
  std::uint32_t emergency_restore_sequence_{42003U};
  std::string experiment_id_;
  std::string method_id_;
  std::string block_id_;
  std::string run_id_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot2_raised_derating_pretest",
            ros::init_options::NoSigintHandler);
  std::signal(SIGINT, multi_agv_control::requestStop);
  std::signal(SIGTERM, multi_agv_control::requestStop);
  std::signal(SIGHUP, multi_agv_control::requestStop);
  try {
    multi_agv_control::Robot2RaisedDeratingPretestNode node;
    return node.run();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed Robot2 raised derating pretest: %s", error.what());
    return 1;
  }
}
