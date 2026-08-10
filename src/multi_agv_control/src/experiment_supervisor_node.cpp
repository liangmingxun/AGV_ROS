#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <agv_msgs/CooperativeState.h>
#include <agv_msgs/DeratingCommand.h>
#include <agv_msgs/ExperimentState.h>
#include <ros/ros.h>

#include "multi_agv_control/experiment_supervisor.hpp"

namespace multi_agv_control {

class ExperimentSupervisorNode {
 public:
  ExperimentSupervisorNode()
      : private_node_("~"), supervisor_(loadConfig()) {
    private_node_.param("exp2a_derating_pretest/publish_rate",
                        publish_rate_, 10.0);
    private_node_.param<std::string>(
        "exp2a_derating_pretest/progress_source", progress_source_,
        "robot2_actual");
    private_node_.param<std::string>(
        "exp2a_derating_pretest/experiment_id", experiment_id_,
        "exp2a_derating_pretest");
    private_node_.param<std::string>("exp2a_derating_pretest/method_id",
                                     method_id_, "CAUSAL_CHAIN");
    private_node_.param<std::string>("exp2a_derating_pretest/block_id",
                                     block_id_, "software");
    private_node_.param<std::string>("exp2a_derating_pretest/run_id",
                                     run_id_, "fake_001");
    private_node_.param("derating_publication_authorized",
                        publication_authorized_, false);
    private_node_.param<std::string>("platform_transport_type",
                                     platform_transport_type_, "fake");
    bool hardware_authorized = false;
    private_node_.param(
        "exp2a_derating_pretest/hardware_execution_authorized",
        hardware_authorized, false);
    if (!(publish_rate_ > 0.0) ||
        (progress_source_ != "robot2_actual" &&
         progress_source_ != "load_actual")) {
      throw std::runtime_error("invalid supervisor node configuration");
    }
    if (publication_authorized_ && platform_transport_type_ != "fake" &&
        !hardware_authorized) {
      throw std::runtime_error(
          "physical derating publication refused by authorization gate");
    }
    if (!supervisor_.arm() || !supervisor_.start(ros::Time::now().toSec())) {
      throw std::runtime_error("failed to arm automatic pretest supervisor");
    }

    state_subscriber_ = node_.subscribe(
        "/multi_agv/cooperative_state", 5,
        &ExperimentSupervisorNode::receiveState, this);
    derating_publisher_ = node_.advertise<agv_msgs::DeratingCommand>(
        "/agv2/derating_command", 5, false);
    experiment_publisher_ = node_.advertise<agv_msgs::ExperimentState>(
        "/multi_agv/experiment_state", 10, true);
    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate_),
                              &ExperimentSupervisorNode::step, this);
  }

 private:
  SupervisorConfig loadConfig() {
    SupervisorConfig config;
    private_node_.param("exp2a_derating_pretest/activation_progress",
                        config.activation_progress, config.activation_progress);
    private_node_.param("exp2a_derating_pretest/restoration_progress",
                        config.restoration_progress,
                        config.restoration_progress);
    private_node_.param(
        "exp2a_derating_pretest/evaluation_start_progress",
        config.evaluation_start_progress,
        config.evaluation_start_progress);
    private_node_.param(
        "exp2a_derating_pretest/evaluation_end_progress",
        config.evaluation_end_progress,
        config.evaluation_end_progress);
    private_node_.param("exp2a_derating_pretest/target_speed_ratio_left",
                        config.target_speed_ratio_left,
                        config.target_speed_ratio_left);
    private_node_.param("exp2a_derating_pretest/target_speed_ratio_right",
                        config.target_speed_ratio_right,
                        config.target_speed_ratio_right);
    private_node_.param(
        "exp2a_derating_pretest/target_acceleration_ratio_left",
        config.target_acceleration_ratio_left,
        config.target_acceleration_ratio_left);
    private_node_.param(
        "exp2a_derating_pretest/target_acceleration_ratio_right",
        config.target_acceleration_ratio_right,
        config.target_acceleration_ratio_right);
    private_node_.param(
        "exp2a_derating_pretest/target_deceleration_ratio_left",
        config.target_deceleration_ratio_left,
        config.target_deceleration_ratio_left);
    private_node_.param(
        "exp2a_derating_pretest/target_deceleration_ratio_right",
        config.target_deceleration_ratio_right,
        config.target_deceleration_ratio_right);
    private_node_.param("exp2a_derating_pretest/ramp_down_time",
                        config.ramp_down_time, config.ramp_down_time);
    private_node_.param("exp2a_derating_pretest/ramp_up_time",
                        config.ramp_up_time, config.ramp_up_time);
    int mode = static_cast<int>(config.derating_mode);
    private_node_.param("exp2a_derating_pretest/derating_mode", mode, mode);
    if (mode < 0 || mode > 255) {
      throw std::runtime_error("derating mode outside uint8 range");
    }
    config.derating_mode = static_cast<std::uint8_t>(mode);
    private_node_.param(
        "exp2a_derating_pretest/finish_after_restoration",
        config.finish_after_restoration,
        config.finish_after_restoration);
    return config;
  }

  void receiveState(const agv_msgs::CooperativeState::ConstPtr& message) {
    const double now = ros::Time::now().toSec();
    if (progress_source_ == "robot2_actual") {
      supervisor_.updateActualProgress(message->s_actual[1],
                                       message->path_state_valid[1], now);
    } else {
      supervisor_.updateActualProgress(message->load_s_actual,
                                       message->load_path_state_valid, now);
    }
  }

  void step(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    supervisor_.tick(now.toSec());
    const auto& snapshot = supervisor_.snapshot();
    const auto& target = snapshot.target;
    if (publication_authorized_) {
      agv_msgs::DeratingCommand command;
      command.header.stamp = now;
      command.robot_id = 2U;
      command.command_seq = target.sequence;
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
      derating_publisher_.publish(command);
    }

    agv_msgs::ExperimentState state;
    state.header.stamp = now;
    state.experiment_id = experiment_id_;
    state.method_id = method_id_;
    state.block_id = block_id_;
    state.run_id = run_id_;
    state.phase = static_cast<std::uint8_t>(snapshot.phase);
    state.run_active = snapshot.run_active;
    state.evaluation_active = snapshot.evaluation_active;
    state.manual_abort = snapshot.manual_abort;
    state.abort_reason = snapshot.abort_reason;
    experiment_publisher_.publish(state);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ExperimentSupervisor supervisor_;
  ros::Subscriber state_subscriber_;
  ros::Publisher derating_publisher_;
  ros::Publisher experiment_publisher_;
  ros::Timer timer_;
  double publish_rate_{10.0};
  bool publication_authorized_{false};
  std::string progress_source_;
  std::string experiment_id_;
  std::string method_id_;
  std::string block_id_;
  std::string run_id_;
  std::string platform_transport_type_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "experiment_supervisor");
  try {
    multi_agv_control::ExperimentSupervisorNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start experiment_supervisor: %s", error.what());
    return 1;
  }
  return 0;
}
