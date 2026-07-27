#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "FirstOrderLPF.h"
#include "Parameters.h"
#include "QEKF.h"
#include "agv_msgs/CapabilityReport.h"
#include "agv_msgs/ChassisCommand.h"
#include "agv_msgs/ChassisFeedback.h"
#include "agv_msgs/DeratingCommand.h"
#include "chassis_controller/chassis_core.hpp"
#include "chassis_driver/chassis_device.hpp"
#include "vofa.hpp"

namespace {

Eigen::Vector3f vectorParam(const ros::NodeHandle& nh, const std::string& name,
                            const Eigen::Vector3f& fallback) {
  std::vector<double> values;
  if (!nh.hasParam(name)) return fallback;
  if (!nh.getParam(name, values) || values.size() != 3) {
    throw std::runtime_error(name + " must contain exactly three numbers");
  }
  return {static_cast<float>(values[0]), static_cast<float>(values[1]),
          static_cast<float>(values[2])};
}

bool finiteVector(const Eigen::Vector3f& value) {
  return value.array().isFinite().all();
}

class ChassisControllerNode {
 public:
  ChassisControllerNode() : node_(), private_("~"), qekf_(parameters_) {
    parameters_.controller.SampleRate = 250.0F;
    loadConfiguration();
    core_.reset(new chassis_controller::ChassisCore(config_));
    initialiseTransport();
    initialiseRos();
    initialiseImuFilters();
  }

  ~ChassisControllerNode() {
    if (device_) device_->sendMotorSpeed({0.0F, 0.0F, 0.0F});
  }

  void run() {
    ros::Rate rate(250.0);
    auto previous = std::chrono::steady_clock::now();
    const ros::Duration state_publish_period(0.01);
    const ros::Duration capability_publish_period(
        1.0 / capability_publish_rate_);
    ros::Time next_state_publish = ros::Time::now();
    ros::Time next_capability_publish = next_state_publish;

    while (ros::ok()) {
      ros::spinOnce();
      const auto current = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(current - previous).count();
      previous = current;

      updateSensorState();
      const auto applied = core_->step(dt);
      if (core_->feedback().command_watchdog_active) {
        ROS_ERROR_THROTTLE(
            1.0,
            "Chassis command watchdog expired after %.3f s; commanding zero",
            config_.command_timeout_seconds);
      }
      if (core_->feedback().sensor_watchdog_active) {
        ROS_ERROR_THROTTLE(
            1.0,
            "STM32 feedback watchdog is latched after %.3f s without a new "
            "packet; commanding zero until fresh feedback and a zero command",
            config_.sensor_feedback_timeout_seconds);
      }
      accumulateCycleFlags();
      sendAppliedCommand(applied);

      const ros::Time now = ros::Time::now();
      if (now >= next_state_publish) {
        const bool publish_capability = now >= next_capability_publish;
        publishState(now, publish_capability);
        next_state_publish += state_publish_period;
        if (next_state_publish < now - state_publish_period) {
          next_state_publish = now + state_publish_period;
        }
        if (publish_capability) {
          next_capability_publish += capability_publish_period;
          if (next_capability_publish < now - capability_publish_period) {
            next_capability_publish = now + capability_publish_period;
          }
        }
      }
      publishVofaIfEnabled();
      rate.sleep();
    }
  }

 private:
  void loadConfiguration() {
    robot_id_ = private_.param<std::string>("robot_id", "");
    const int robot_index = private_.param<int>("robot_index", 0);
    if (robot_id_.empty() || robot_index < 1 || robot_index > 3) {
      throw std::runtime_error("robot_id and robot_index (1..3) are required");
    }
    if (robot_id_ != "agv" + std::to_string(robot_index)) {
      throw std::runtime_error("robot_id must match robot_index");
    }
    config_.robot_index = static_cast<std::uint8_t>(robot_index);
    config_.wheel_separation = private_.param("wheel_separation", 0.114);
    config_.control_loop_overrun_seconds =
        private_.param("control_loop_overrun_seconds", 0.008);
    config_.command_timeout_seconds =
        private_.param("command_timeout_seconds", 0.20);
    config_.sensor_feedback_timeout_seconds =
        private_.param("sensor_feedback_timeout_seconds", 0.15);
    config_.odometry_stationary_wheel_velocity_tolerance =
        private_.param(
            "odometry_stationary_wheel_velocity_tolerance", 0.005);
    config_.nominal_limits = {
        private_.param("nominal/max_wheel_linear_velocity_left", 0.9),
        private_.param("nominal/max_wheel_linear_velocity_right", 0.9),
        private_.param("nominal/max_wheel_linear_acceleration_left", 1.0),
        private_.param("nominal/max_wheel_linear_acceleration_right", 1.0),
        private_.param("nominal/max_wheel_linear_deceleration_left", 2.0),
        private_.param("nominal/max_wheel_linear_deceleration_right", 2.0)};
    const double serial_velocity_limit =
        ChassisDevice::kMaxWheelLinearVelocityMetersPerSecond;
    if (config_.nominal_limits.max_velocity_left > serial_velocity_limit ||
        config_.nominal_limits.max_velocity_right > serial_velocity_limit) {
      ROS_WARN("Configured wheel velocity exceeds the serial safety limit; "
               "capability is clamped to %.3f m/s", serial_velocity_limit);
      config_.nominal_limits.max_velocity_left =
          std::min(config_.nominal_limits.max_velocity_left, serial_velocity_limit);
      config_.nominal_limits.max_velocity_right =
          std::min(config_.nominal_limits.max_velocity_right, serial_velocity_limit);
    }

    serial_device_ = private_.param<std::string>("serial_device", "/dev/ttyACM0");
    serial_startup_timeout_seconds_ =
        private_.param("serial_startup_timeout_seconds", 10.0);
    transport_type_ = private_.param<std::string>("transport_type", "serial");
    odom_frame_ = private_.param<std::string>("odom_frame", robot_id_ + "/odom");
    base_frame_ = private_.param<std::string>("base_frame", robot_id_ + "/base_link");
    imu_frame_ = private_.param<std::string>("imu_frame", robot_id_ + "/imu_link");
    base_link_z_ = private_.param("base_link_z", 0.05969);
    capability_publish_rate_ =
        private_.param("capability_publish_rate", 20.0);
    enable_vofa_ = private_.param("enable_vofa", false);

    if (odom_frame_ != robot_id_ + "/odom" ||
        base_frame_ != robot_id_ + "/base_link" ||
        imu_frame_ != robot_id_ + "/imu_link") {
      throw std::runtime_error("odom/base/imu frames must use the frozen robot prefix");
    }
    if (!std::isfinite(base_link_z_) || base_link_z_ < 0.0 ||
        !std::isfinite(capability_publish_rate_) ||
        capability_publish_rate_ <= 0.0 || capability_publish_rate_ > 100.0 ||
        !std::isfinite(serial_startup_timeout_seconds_) ||
        serial_startup_timeout_seconds_ <= 0.0) {
      throw std::runtime_error(
          "base_link_z, capability publish rate or serial startup timeout is "
          "invalid");
    }

    acc_bias_ = vectorParam(private_, "imu/acc_bias", Eigen::Vector3f::Zero());
    acc_scale_ = vectorParam(private_, "imu/acc_scale", Eigen::Vector3f::Ones());
    gyro_bias_ = vectorParam(private_, "imu/gyro_bias", Eigen::Vector3f::Zero());
    gyro_lpf_tau_ = private_.param("imu/gyro_lpf_tau", 0.02);
    if (!finiteVector(acc_bias_) || !finiteVector(acc_scale_) ||
        !finiteVector(gyro_bias_) ||
        (acc_scale_.array().abs() <= 1.0e-6F).any() ||
        !std::isfinite(gyro_lpf_tau_) || gyro_lpf_tau_ <= 0.0) {
      throw std::runtime_error("IMU calibration parameters are invalid");
    }
  }

  void initialiseTransport() {
    if (transport_type_ == "serial") {
      device_.reset(new ChassisDevice(serial_device_, 1000000,
                                      serial_startup_timeout_seconds_));
    } else if (transport_type_ != "fake") {
      throw std::runtime_error("transport_type must be 'serial' or 'fake'");
    }

    if (enable_vofa_) {
      vofa_.reset(new VofaFrame());
      const auto remote_ip = private_.param<std::string>("vofa_remote_ip", "192.168.0.52");
      const int remote_port = private_.param("vofa_remote_port", 1347);
      const int local_port = private_.param("vofa_local_port", 1349);
      if (!vofa_->initialize("0.0.0.0", local_port, remote_ip, remote_port)) {
        ROS_WARN("VOFA initialization failed; monitoring is disabled");
        vofa_.reset();
      }
    }
  }

  void initialiseRos() {
    command_sub_ = node_.subscribe("chassis_command", 1,
        &ChassisControllerNode::commandCallback, this,
        ros::TransportHints().tcpNoDelay());
    derating_sub_ = node_.subscribe("derating_command", 5,
        &ChassisControllerNode::deratingCallback, this,
        ros::TransportHints().tcpNoDelay());
    feedback_pub_ = node_.advertise<agv_msgs::ChassisFeedback>(
        "chassis_feedback", 5, false);
    capability_pub_ = node_.advertise<agv_msgs::CapabilityReport>(
        "capability_report", 5, false);
    odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 10, false);
    imu_pub_ = node_.advertise<sensor_msgs::Imu>("imu", 10, false);
    reset_odometry_service_ = node_.advertiseService(
        "reset_odometry", &ChassisControllerNode::resetOdometry, this);
  }

  void initialiseImuFilters() {
    const float dt = 1.0F / parameters_.controller.SampleRate;
    for (int axis = 0; axis < 3; ++axis) {
      gyro_filters_.emplace_back(dt, static_cast<float>(gyro_lpf_tau_));
    }
  }

  void commandCallback(const agv_msgs::ChassisCommand::ConstPtr& message) {
    const chassis_controller::CommandInput command{
        message->robot_id, message->command_seq,
        {message->wheel_linear_velocity_left_raw,
         message->wheel_linear_velocity_right_raw}};
    if (!core_->acceptCommand(command)) {
      ROS_WARN_THROTTLE(1.0, "Rejected chassis command for robot or sequence mismatch");
    }
  }

  void deratingCallback(const agv_msgs::DeratingCommand::ConstPtr& message) {
    chassis_controller::DeratingInput command;
    command.robot_id = message->robot_id;
    command.sequence = message->command_seq;
    command.active = message->active;
    command.mode = message->mode;
    command.ratios = {message->target_speed_ratio_left,
                      message->target_speed_ratio_right,
                      message->target_accel_ratio_left,
                      message->target_accel_ratio_right,
                      message->target_decel_ratio_left,
                      message->target_decel_ratio_right};
    command.ramp_down_seconds = message->ramp_down_time;
    command.ramp_up_seconds = message->ramp_up_time;
    if (!core_->acceptDerating(command)) {
      ROS_WARN_THROTTLE(1.0, "Rejected derating command for robot or sequence mismatch");
    }
  }

  bool resetOdometry(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    const auto& state = core_->feedback();
    constexpr double stationary_tolerance = 0.01;
    const auto stopped = [stationary_tolerance](double value) {
      return std::isfinite(value) && std::abs(value) <= stationary_tolerance;
    };
    if (!stopped(state.raw.left) || !stopped(state.raw.right) ||
        !stopped(state.applied.left) || !stopped(state.applied.right) ||
        !stopped(state.actual.left) || !stopped(state.actual.right)) {
      response.success = false;
      response.message = "odometry reset requires zero raw, applied and actual wheel speed";
      return true;
    }
    core_->resetOdometry();
    response.success = true;
    response.message = "odometry reset";
    return true;
  }

  void updateSensorState() {
    chassis_controller::SensorInput input;
    if (device_) {
      const auto sensor = device_->getSensorData();
      const Eigen::Vector3f acc =
          (sensor.acc_ - acc_bias_).cwiseQuotient(acc_scale_);
      const Eigen::Vector3f gyro = sensor.gyro_ - gyro_bias_;
      for (int axis = 0; axis < 3; ++axis) {
        corrected_acc_[axis] = acc[axis];
        corrected_gyro_[axis] = gyro_filters_[axis].Filter(gyro[axis]);
      }
      if (!qekf_initialised_) {
        qekf_.Reset(acc);
        qekf_initialised_ = true;
      }
      qekf_.Step(acc, corrected_gyro_, parameters_.estimator.EstimateBias);
      qekf_.GetQuaternion(body_quaternion_);
      input.wheel_left_mm_per_second = sensor.wheelmotor_speed_[0];
      input.wheel_right_mm_per_second = sensor.wheelmotor_speed_[1];
      input.imu_yaw_rate = corrected_gyro_[2];
      input.battery_voltage = sensor.battery_voltage_;
      input.packet_sequence = sensor.packet_seq_;
      input.packet_fresh =
          !has_serial_packet_ ||
          sensor.packet_seq_ != last_serial_packet_sequence_;
      if (input.packet_fresh) {
        has_serial_packet_ = true;
        last_serial_packet_sequence_ = sensor.packet_seq_;
        last_serial_receive_stamp_ = ros::Time::now();
      }
    } else {
      input.wheel_left_mm_per_second = last_applied_.left * 1000.0;
      input.wheel_right_mm_per_second = last_applied_.right * 1000.0;
      input.imu_yaw_rate =
          (last_applied_.right - last_applied_.left) /
          config_.wheel_separation;
      corrected_gyro_[2] = static_cast<float>(input.imu_yaw_rate);
      input.packet_sequence = ++fake_packet_sequence_;
      input.packet_fresh = true;
      last_serial_receive_stamp_ = ros::Time::now();
      body_quaternion_ = Eigen::Quaternionf::Identity();
    }
    core_->updateSensors(input);
  }

  void sendAppliedCommand(const chassis_controller::WheelCommand& applied) {
    last_applied_ = applied;
    if (device_) {
      if (!device_->sendMotorSpeed(
              {static_cast<float>(applied.left * 1000.0),
               static_cast<float>(applied.right * 1000.0), 0.0F},
              core_->feedback().packet_sequence)) {
        throw std::runtime_error("failed to send chassis serial command");
      }
    }
  }

  sensor_msgs::Imu makeImu(const ros::Time& stamp) const {
    sensor_msgs::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = imu_frame_;
    message.orientation.x = body_quaternion_.x();
    message.orientation.y = body_quaternion_.y();
    message.orientation.z = body_quaternion_.z();
    message.orientation.w = body_quaternion_.w();
    message.angular_velocity.x = corrected_gyro_[0];
    message.angular_velocity.y = corrected_gyro_[1];
    message.angular_velocity.z = corrected_gyro_[2];
    message.linear_acceleration.x = corrected_acc_[0];
    message.linear_acceleration.y = corrected_acc_[1];
    message.linear_acceleration.z = corrected_acc_[2];
    return message;
  }

  nav_msgs::Odometry makeOdometry(const ros::Time& stamp) const {
    const auto& state = core_->feedback();
    nav_msgs::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = state.odometry.x;
    message.pose.pose.position.y = state.odometry.y;
    message.pose.pose.position.z = base_link_z_;
    message.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.odometry.yaw);
    message.twist.twist.linear.x = state.linear_velocity;
    message.twist.twist.angular.z = state.angular_velocity;
    return message;
  }

  void accumulateCycleFlags() {
    const auto& state = core_->feedback();
    overrun_since_publish_ = overrun_since_publish_ || state.control_loop_overrun;
    speed_limited_left_since_publish_ =
        speed_limited_left_since_publish_ || state.speed_limited_left;
    speed_limited_right_since_publish_ =
        speed_limited_right_since_publish_ || state.speed_limited_right;
    accel_limited_left_since_publish_ =
        accel_limited_left_since_publish_ || state.accel_limited_left;
    accel_limited_right_since_publish_ =
        accel_limited_right_since_publish_ || state.accel_limited_right;
    decel_limited_left_since_publish_ =
        decel_limited_left_since_publish_ || state.decel_limited_left;
    decel_limited_right_since_publish_ =
        decel_limited_right_since_publish_ || state.decel_limited_right;
    capability_speed_limited_left_since_publish_ =
        capability_speed_limited_left_since_publish_ ||
        state.speed_limited_left;
    capability_speed_limited_right_since_publish_ =
        capability_speed_limited_right_since_publish_ ||
        state.speed_limited_right;
    capability_accel_limited_left_since_publish_ =
        capability_accel_limited_left_since_publish_ ||
        state.accel_limited_left;
    capability_accel_limited_right_since_publish_ =
        capability_accel_limited_right_since_publish_ ||
        state.accel_limited_right;
    capability_decel_limited_left_since_publish_ =
        capability_decel_limited_left_since_publish_ ||
        state.decel_limited_left;
    capability_decel_limited_right_since_publish_ =
        capability_decel_limited_right_since_publish_ ||
        state.decel_limited_right;
  }

  void publishState(const ros::Time& stamp, bool publish_capability) {
    const auto& state = core_->feedback();
    const auto& capability = core_->capability();
    publish_capability =
        publish_capability || !has_published_capability_ ||
        capability.limits.max_velocity_left !=
            last_capability_max_velocity_left_ ||
        capability.limits.max_velocity_right !=
            last_capability_max_velocity_right_ ||
        capability.limits.max_acceleration_left !=
            last_capability_max_acceleration_left_ ||
        capability.limits.max_acceleration_right !=
            last_capability_max_acceleration_right_ ||
        capability.limits.max_deceleration_left !=
            last_capability_max_deceleration_left_ ||
        capability.limits.max_deceleration_right !=
            last_capability_max_deceleration_right_ ||
        capability.derating_ratio != last_capability_derating_ratio_ ||
        capability.derating_mode != last_capability_derating_mode_ ||
        capability.derating_active != last_capability_derating_active_;
    const auto imu = makeImu(stamp);
    const auto odom = makeOdometry(stamp);

    agv_msgs::ChassisFeedback feedback;
    feedback.header.stamp = stamp;
    feedback.header.frame_id = base_frame_;
    feedback.robot_id = config_.robot_index;
    feedback.feedback_seq = ++feedback_publish_sequence_;
    feedback.command_seq_applied = state.command_seq_applied;
    feedback.packet_seq = state.packet_sequence;
    feedback.serial_receive_stamp = last_serial_receive_stamp_;
    feedback.wheel_linear_velocity_left_raw = state.raw.left;
    feedback.wheel_linear_velocity_right_raw = state.raw.right;
    feedback.wheel_linear_velocity_left_applied = state.applied.left;
    feedback.wheel_linear_velocity_right_applied = state.applied.right;
    feedback.wheel_linear_velocity_left_actual = state.actual.left;
    feedback.wheel_linear_velocity_right_actual = state.actual.right;
    feedback.linear_velocity_actual = state.linear_velocity;
    feedback.angular_velocity_actual = state.angular_velocity;
    feedback.imu = imu;
    feedback.odom = odom;
    feedback.battery_voltage = state.battery_voltage;
    feedback.control_loop_overrun = overrun_since_publish_;
    feedback.speed_limit_active_left = speed_limited_left_since_publish_;
    feedback.speed_limit_active_right = speed_limited_right_since_publish_;
    feedback.accel_limit_active_left = accel_limited_left_since_publish_;
    feedback.accel_limit_active_right = accel_limited_right_since_publish_;
    feedback.decel_limit_active_left = decel_limited_left_since_publish_;
    feedback.decel_limit_active_right = decel_limited_right_since_publish_;
    feedback_pub_.publish(feedback);

    if (publish_capability) {
      agv_msgs::CapabilityReport report;
      report.header.stamp = stamp;
      report.header.frame_id = base_frame_;
      report.robot_id = config_.robot_index;
      report.capability_seq = ++capability_publish_sequence_;
      report.max_wheel_linear_velocity_left =
          capability.limits.max_velocity_left;
      report.max_wheel_linear_velocity_right =
          capability.limits.max_velocity_right;
      report.max_wheel_linear_acceleration_left =
          capability.limits.max_acceleration_left;
      report.max_wheel_linear_acceleration_right =
          capability.limits.max_acceleration_right;
      report.max_wheel_linear_deceleration_left =
          capability.limits.max_deceleration_left;
      report.max_wheel_linear_deceleration_right =
          capability.limits.max_deceleration_right;
      report.derating_ratio = capability.derating_ratio;
      report.derating_mode = capability.derating_mode;
      report.derating_active = capability.derating_active;
      report.speed_limit_active_left =
          capability_speed_limited_left_since_publish_;
      report.speed_limit_active_right =
          capability_speed_limited_right_since_publish_;
      report.accel_limit_active_left =
          capability_accel_limited_left_since_publish_;
      report.accel_limit_active_right =
          capability_accel_limited_right_since_publish_;
      report.decel_limit_active_left =
          capability_decel_limited_left_since_publish_;
      report.decel_limit_active_right =
          capability_decel_limited_right_since_publish_;
      report.battery_voltage = state.battery_voltage;
      capability_pub_.publish(report);
      has_published_capability_ = true;
      last_capability_max_velocity_left_ =
          capability.limits.max_velocity_left;
      last_capability_max_velocity_right_ =
          capability.limits.max_velocity_right;
      last_capability_max_acceleration_left_ =
          capability.limits.max_acceleration_left;
      last_capability_max_acceleration_right_ =
          capability.limits.max_acceleration_right;
      last_capability_max_deceleration_left_ =
          capability.limits.max_deceleration_left;
      last_capability_max_deceleration_right_ =
          capability.limits.max_deceleration_right;
      last_capability_derating_ratio_ = capability.derating_ratio;
      last_capability_derating_mode_ = capability.derating_mode;
      last_capability_derating_active_ = capability.derating_active;
      capability_speed_limited_left_since_publish_ = false;
      capability_speed_limited_right_since_publish_ = false;
      capability_accel_limited_left_since_publish_ = false;
      capability_accel_limited_right_since_publish_ = false;
      capability_decel_limited_left_since_publish_ = false;
      capability_decel_limited_right_since_publish_ = false;
    }
    odom_pub_.publish(odom);
    imu_pub_.publish(imu);

    geometry_msgs::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = state.odometry.x;
    transform.transform.translation.y = state.odometry.y;
    transform.transform.translation.z = base_link_z_;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);

    overrun_since_publish_ = false;
    speed_limited_left_since_publish_ = false;
    speed_limited_right_since_publish_ = false;
    accel_limited_left_since_publish_ = false;
    accel_limited_right_since_publish_ = false;
    decel_limited_left_since_publish_ = false;
    decel_limited_right_since_publish_ = false;
  }

  void publishVofaIfEnabled() {
    if (!vofa_) return;
    const auto& state = core_->feedback();
    vofa_->reset();
    vofa_->push(static_cast<float>(state.raw.left), static_cast<float>(state.raw.right));
    vofa_->push(static_cast<float>(state.applied.left),
                static_cast<float>(state.applied.right));
    vofa_->push(static_cast<float>(state.actual.left),
                static_cast<float>(state.actual.right));
    vofa_->push(static_cast<float>(state.battery_voltage));
    vofa_->send();
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  ros::Subscriber command_sub_;
  ros::Subscriber derating_sub_;
  ros::Publisher feedback_pub_;
  ros::Publisher capability_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher imu_pub_;
  ros::ServiceServer reset_odometry_service_;
  tf::TransformBroadcaster tf_broadcaster_;

  chassis_controller::ChassisConfig config_;
  std::unique_ptr<chassis_controller::ChassisCore> core_;
  std::unique_ptr<ChassisDevice> device_;
  std::unique_ptr<VofaFrame> vofa_;
  chassis_controller::WheelCommand last_applied_{};
  std::uint32_t fake_packet_sequence_{0};
  std::uint32_t last_serial_packet_sequence_{0};
  bool has_serial_packet_{false};
  std::uint32_t feedback_publish_sequence_{0};
  std::uint32_t capability_publish_sequence_{0};
  ros::Time last_serial_receive_stamp_{};
  bool has_published_capability_{false};
  double last_capability_max_velocity_left_{0.0};
  double last_capability_max_velocity_right_{0.0};
  double last_capability_max_acceleration_left_{0.0};
  double last_capability_max_acceleration_right_{0.0};
  double last_capability_max_deceleration_left_{0.0};
  double last_capability_max_deceleration_right_{0.0};
  double last_capability_derating_ratio_{0.0};
  std::uint8_t last_capability_derating_mode_{0};
  bool last_capability_derating_active_{false};

  Parameters parameters_;
  QEKF qekf_;
  bool qekf_initialised_{false};
  Eigen::Quaternionf body_quaternion_{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f corrected_acc_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f corrected_gyro_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f acc_bias_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f acc_scale_{Eigen::Vector3f::Ones()};
  Eigen::Vector3f gyro_bias_{Eigen::Vector3f::Zero()};
  std::vector<FirstOrderLPF> gyro_filters_;

  std::string robot_id_;
  std::string serial_device_;
  std::string transport_type_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  double base_link_z_{0.0};
  double capability_publish_rate_{20.0};
  double gyro_lpf_tau_{0.02};
  double serial_startup_timeout_seconds_{10.0};
  bool enable_vofa_{false};
  bool overrun_since_publish_{false};
  bool speed_limited_left_since_publish_{false};
  bool speed_limited_right_since_publish_{false};
  bool accel_limited_left_since_publish_{false};
  bool accel_limited_right_since_publish_{false};
  bool decel_limited_left_since_publish_{false};
  bool decel_limited_right_since_publish_{false};
  bool capability_speed_limited_left_since_publish_{false};
  bool capability_speed_limited_right_since_publish_{false};
  bool capability_accel_limited_left_since_publish_{false};
  bool capability_accel_limited_right_since_publish_{false};
  bool capability_decel_limited_left_since_publish_{false};
  bool capability_decel_limited_right_since_publish_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "chassis_controller");
  try {
    ChassisControllerNode node;
    node.run();
  } catch (const std::exception& error) {
    ROS_FATAL("chassis_controller failed: %s", error.what());
    return 1;
  }
  return 0;
}
