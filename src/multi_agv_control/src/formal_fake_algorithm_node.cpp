#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <agv_msgs/CapabilityReport.h>
#include <agv_msgs/ChassisCommand.h>
#include <agv_msgs/ControllerState.h>
#include <agv_msgs/CooperativeState.h>
#include <agv_msgs/PathReference.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Scalar.h>

#include "multi_agv_control/capability_mapper.hpp"
#include "multi_agv_control/formal_execution_gate.hpp"
#include "multi_agv_control/lower_channel_controller.hpp"
#include "multi_agv_control/planar_support_tracker.hpp"
#include "multi_agv_control/s_curve_path.hpp"
#include "multi_agv_control/support_geometry.hpp"
#include "multi_agv_control/upper_reference_generator.hpp"

namespace multi_agv_control {
namespace {

constexpr std::size_t kRobotCount = 3U;
constexpr std::size_t kDebugHeaderFields = 9U;
constexpr std::size_t kDebugFieldsPerRobot = 27U;

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

std::array<double, 3> array3(
    const ros::NodeHandle& node, const std::string& name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      value.size() != 3) {
    throw std::runtime_error(name + " must contain three numeric values");
  }
  std::array<double, 3> result;
  for (int index = 0; index < 3; ++index) {
    const auto& item = value[index];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      result[static_cast<std::size_t>(index)] =
          static_cast<double>(item);
    } else if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result[static_cast<std::size_t>(index)] =
          static_cast<int>(item);
    } else {
      throw std::runtime_error(name + " contains a non-numeric value");
    }
  }
  return result;
}

std::array<double, 2> array2(
    const ros::NodeHandle& node, const std::string& name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      value.size() != 2) {
    throw std::runtime_error(name + " must contain two numeric values");
  }
  std::array<double, 2> result;
  for (int index = 0; index < 2; ++index) {
    const auto& item = value[index];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      result[static_cast<std::size_t>(index)] =
          static_cast<double>(item);
    } else if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result[static_cast<std::size_t>(index)] =
          static_cast<int>(item);
    } else {
      throw std::runtime_error(name + " contains a non-numeric value");
    }
  }
  return result;
}

double normalisedMargin(double value, double scale) {
  if (!std::isfinite(value) || !std::isfinite(scale) || scale <= 0.0) {
    return 0.0;
  }
  return std::clamp(value / scale, 0.0, 1.0);
}

PlanarPose poseValue(const geometry_msgs::Pose2D& pose) {
  return {{pose.x, pose.y}, pose.theta};
}

geometry_msgs::Pose2D poseMessage(const PlanarPose& pose) {
  geometry_msgs::Pose2D message;
  message.x = pose.position.x();
  message.y = pose.position.y();
  message.theta = pose.yaw;
  return message;
}

}  // namespace

class FormalFakeAlgorithmNode {
 public:
  FormalFakeAlgorithmNode()
      : private_node_("~"), path_(loadPathConfig()),
        geometry_(path_, loadGeometryConfig()),
        tracker_(geometry_, loadTrackerConfig()),
        capability_mapper_(loadCapabilityReserve()) {
    loadRuntime();
    const auto upper_config = loadUpperConfig();
    const auto lower_config = loadLowerConfig();
    enforceFakeOnlyGates(upper_config, lower_config);
    upper_mode_ = upper_config.mode;
    lower_mode_ = lower_config.mode;
    upper_generator_ =
        std::make_unique<UpperReferenceGenerator>(upper_config);
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      lower_controllers_[index] =
          std::make_unique<LowerChannelController>(lower_config);
      lower_controllers_[index]->reset(
          lower_initial_parameter_, lower_initial_disturbance_);
      previous_boundary_[index] =
          upper_config.agents[index].initial_boundary;
    }

    state_subscriber_ = node_.subscribe(
        "/multi_agv/cooperative_state", 5,
        &FormalFakeAlgorithmNode::receiveState, this);
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      capability_subscribers_[index] = node_.subscribe<agv_msgs::CapabilityReport>(
          "/agv" + std::to_string(index + 1U) + "/capability_report", 5,
          [this, index](const agv_msgs::CapabilityReport::ConstPtr& message) {
            receiveCapability(index, message);
          });
    }
    reference_publisher_ = node_.advertise<agv_msgs::PathReference>(
        "/multi_agv/path_reference", 5, false);
    controller_publisher_ = node_.advertise<agv_msgs::ControllerState>(
        "/multi_agv/controller_state", 10, false);
    debug_publisher_ = node_.advertise<std_msgs::Float64MultiArray>(
        "/multi_agv/formal_algorithm_state", 10, false);
    timer_ = node_.createTimer(
        ros::Duration(1.0 / publish_rate_),
        &FormalFakeAlgorithmNode::step, this);
    ROS_WARN(
        "Formal algorithm ROS integration is enabled for fake transport only "
        "(upper=%s lower=%s); serial transport is structurally refused",
        upperModeName(upper_mode_), lowerModeName(lower_mode_));
  }

 private:
  SCurveConfig loadPathConfig() {
    SCurveConfig config;
    private_node_.param("path_s_curve/amplitude", config.amplitude, 0.05);
    private_node_.param(
        "path_s_curve/longitudinal_length",
        config.longitudinal_length, 1.0);
    int samples = 20001;
    private_node_.param("path_s_curve/lookup_samples", samples, 20001);
    if (samples < 2) throw std::runtime_error("invalid path lookup_samples");
    config.lookup_samples = static_cast<std::size_t>(samples);
    return config;
  }

  SupportGeometryConfig loadGeometryConfig() {
    SupportGeometryConfig config;
    private_node_.param(
        "support_geometry/minimum_nondegeneracy",
        config.minimum_nondegeneracy, 0.2);
    private_node_.param(
        "support_geometry/minimum_speed_scale",
        config.minimum_speed_scale, 0.2);
    int samples = 10001;
    private_node_.param(
        "support_geometry/validation_samples", samples, 10001);
    if (samples < 2) {
      throw std::runtime_error("invalid support validation_samples");
    }
    config.validation_samples = static_cast<std::size_t>(samples);
    XmlRpc::XmlRpcValue supports;
    if (!private_node_.getParam("support_geometry/supports", supports) ||
        supports.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        supports.size() != 3) {
      throw std::runtime_error("support_geometry/supports must contain three entries");
    }
    for (int index = 0; index < 3; ++index) {
      config.offsets.push_back({
          xmlNumber(supports[index], "q_tangent"),
          xmlNumber(supports[index], "q_normal")});
    }
    return config;
  }

  PlanarTrackerConfig loadTrackerConfig() {
    PlanarTrackerConfig config;
    const std::string root = "formal_fake_runtime/tracker/";
    private_node_.param(
        root + "longitudinal_gain",
        config.longitudinal_gain, config.longitudinal_gain);
    private_node_.param(
        root + "lateral_gain",
        config.lateral_gain, config.lateral_gain);
    private_node_.param(
        root + "heading_gain",
        config.heading_gain, config.heading_gain);
    int samples = static_cast<int>(config.chassis_reference_samples);
    private_node_.param(root + "chassis_reference_samples", samples, samples);
    if (samples < 2) {
      throw std::runtime_error("tracker reference samples must be at least two");
    }
    config.chassis_reference_samples = static_cast<std::size_t>(samples);
    config.wheel_separation = array3(
        private_node_, root + "wheel_separation");
    XmlRpc::XmlRpcValue offsets;
    if (!private_node_.getParam(root + "base_to_support", offsets) ||
        offsets.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        offsets.size() != 3) {
      throw std::runtime_error("tracker base_to_support must contain three entries");
    }
    for (int index = 0; index < 3; ++index) {
      config.base_to_support[static_cast<std::size_t>(index)] = {
          xmlNumber(offsets[index], "x"), xmlNumber(offsets[index], "y")};
    }
    tracker_config_ = config;
    return config;
  }

  CapabilityReserve loadCapabilityReserve() {
    CapabilityReserve reserve;
    const std::string root = "formal_fake_runtime/capability_reserve/";
    private_node_.param(
        root + "body_linear_velocity",
        reserve.body_linear_velocity, 0.0);
    private_node_.param(
        root + "body_angular_velocity",
        reserve.body_angular_velocity, 0.0);
    private_node_.param(
        root + "wheel_linear_velocity",
        reserve.wheel_linear_velocity, 0.0);
    private_node_.param(
        root + "path_acceleration",
        reserve.path_acceleration, 0.0);
    private_node_.param(
        root + "path_deceleration",
        reserve.path_deceleration, 0.0);
    return reserve;
  }

  void loadRuntime() {
    const std::string root = "formal_fake_runtime/";
    private_node_.param(root + "publish_rate", publish_rate_, 100.0);
    private_node_.param(
        root + "maximum_state_age", maximum_state_age_, 0.15);
    private_node_.param(
        root + "maximum_capability_age", maximum_capability_age_, 0.20);
    private_node_.param(
        root + "require_valid_load_state", require_valid_load_state_, true);
    private_node_.param(
        root + "initial_progress", reference_progress_, 0.0);
    private_node_.param(
        root + "target_progress", target_progress_, path_.length());
    private_node_.param<std::string>(
        root + "experiment_id", experiment_id_,
        "formal_algorithm_fake_integration");
    private_node_.param(
        root + "leader/initial_position", leader_position_, 0.0);
    private_node_.param(
        root + "leader/velocity", leader_velocity_, 0.08);
    private_node_.param(
        root + "leader/acceleration", leader_acceleration_, 0.0);
    distributed_state_.position = array3(
        private_node_, root + "distributed_initial/position");
    distributed_state_.velocity = array3(
        private_node_, root + "distributed_initial/velocity");
    distributed_state_.auxiliary = array3(
        private_node_, root + "distributed_initial/auxiliary");
    private_node_.param(
        root + "risk/capability_reserve", capability_reserve_, 0.015);
    private_node_.param(
        root + "risk/capability_safe_margin",
        capability_safe_margin_, 0.12);
    private_node_.param(
        root + "risk/input_safe_margin", input_safe_margin_, 0.40);
    private_node_.param(
        root + "risk/wheel_safe_margin", wheel_safe_margin_, 0.08);
    private_node_.param(
        root + "risk/position_limit", position_limit_, 0.12);
    private_node_.param(
        root + "risk/position_safe_margin",
        position_safe_margin_, 0.06);
    private_node_.param(
        root + "lower/velocity_lower_bound",
        velocity_lower_bound_, -0.15);
    private_node_.param(
        root + "lower/velocity_upper_bound",
        velocity_upper_bound_, 0.58);
    lower_initial_parameter_ = array2(
        private_node_, root + "lower/initial_parameter");
    private_node_.param(
        root + "lower/initial_disturbance_estimate",
        lower_initial_disturbance_, 0.08);
    if (!(publish_rate_ > 0.0) || !(maximum_state_age_ > 0.0) ||
        !(maximum_capability_age_ > 0.0) ||
        !(target_progress_ > reference_progress_) ||
        target_progress_ > path_.length() ||
        velocity_lower_bound_ >= velocity_upper_bound_) {
      throw std::runtime_error("invalid formal fake runtime configuration");
    }
    bool command_authorized = false;
    private_node_.param(
        root + "command_publication_authorized",
        command_authorized, false);
    command_publication_authorized_ = command_authorized;
  }

  UpperReferenceConfig loadUpperConfig() {
    UpperReferenceConfig config;
    std::string mode;
    private_node_.param<std::string>("formal_upper/mode", mode, "M1");
    config.mode = upperModeFromString(mode);
    int ticks = 4;
    private_node_.param(
        "formal_upper/controller_ticks_per_update", ticks, 4);
    if (ticks <= 0) throw std::runtime_error("invalid upper update divider");
    config.controller_ticks_per_update = static_cast<std::size_t>(ticks);
    upper_ticks_per_update_ = config.controller_ticks_per_update;
    private_node_.param(
        "formal_upper/fixed_speed", config.fixed_m4_speed, 0.305);
    private_node_.param(
        "formal_upper/golden_vectors_verified",
        config.golden_vectors_verified, false);
    const std::string root = "formal_upper/agents/";
    const auto physical_lower = array3(private_node_, root + "physical_lower");
    const auto physical_upper = array3(private_node_, root + "physical_upper");
    const auto margin_lower =
        array3(private_node_, root + "physical_margin_lower");
    const auto margin_upper =
        array3(private_node_, root + "physical_margin_upper");
    const auto zero_margin = array3(private_node_, root + "zero_margin");
    const auto minimum_width = array3(private_node_, root + "minimum_width");
    const auto inner_margin = array3(private_node_, root + "inner_margin");
    const auto initial_lower = array3(private_node_, root + "initial_lower");
    const auto initial_upper = array3(private_node_, root + "initial_upper");
    const auto nominal_lower = array3(private_node_, root + "nominal_lower");
    const auto nominal_upper = array3(private_node_, root + "nominal_upper");
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      auto& agent = config.agents[index];
      agent.admissible_set = {
          physical_lower[index], physical_upper[index],
          margin_lower[index], margin_upper[index],
          zero_margin[index], minimum_width[index]};
      agent.initial_boundary = {
          initial_lower[index], initial_upper[index]};
      agent.nominal_boundary = {
          nominal_lower[index], nominal_upper[index]};
      agent.inner_margin = inner_margin[index];
    }
    if (config.mode == UpperMode::kM1) {
      const auto restore_lower =
          array3(private_node_, root + "restore_gain_lower");
      const auto restore_upper =
          array3(private_node_, root + "restore_gain_upper");
      const auto risk_lower =
          array3(private_node_, root + "risk_gain_lower");
      const auto risk_upper =
          array3(private_node_, root + "risk_gain_upper");
      const auto risk_scale = array3(private_node_, root + "risk_scale");
      const auto risk_decay = array3(private_node_, root + "risk_decay");
      for (std::size_t index = 0; index < kRobotCount; ++index) {
        auto& agent = config.agents[index];
        agent.restore_gain_lower = restore_lower[index];
        agent.restore_gain_upper = restore_upper[index];
        agent.risk_gain_lower = risk_lower[index];
        agent.risk_gain_upper = risk_upper[index];
        agent.risk_scale = risk_scale[index];
        agent.risk_decay = risk_decay[index];
      }
    }
    return config;
  }

  LowerChannelConfig loadLowerConfig() {
    LowerChannelConfig config;
    std::string mode;
    private_node_.param<std::string>("formal_lower/mode", mode, "R1");
    config.mode = lowerModeFromString(mode);
    private_node_.param(
        "formal_lower/composite_gain",
        config.composite_gain, config.composite_gain);
    private_node_.param(
        "formal_lower/feedback_gain",
        config.feedback_gain, config.feedback_gain);
    config.adaptation_gain = array2(
        private_node_, "formal_lower/adaptation_gain");
    private_node_.param(
        "formal_lower/disturbance_adaptation_gain",
        config.disturbance_adaptation_gain,
        config.disturbance_adaptation_gain);
    private_node_.param(
        "formal_lower/parameter_leakage",
        config.parameter_leakage, config.parameter_leakage);
    private_node_.param(
        "formal_lower/disturbance_leakage",
        config.disturbance_leakage, config.disturbance_leakage);
    private_node_.param(
        "formal_lower/robust_boundary_layer",
        config.robust_boundary_layer, config.robust_boundary_layer);
    config.parameter_min = array2(
        private_node_, "formal_lower/parameter_min");
    config.parameter_max = array2(
        private_node_, "formal_lower/parameter_max");
    private_node_.param(
        "formal_lower/disturbance_estimate_max",
        config.disturbance_estimate_max,
        config.disturbance_estimate_max);
    private_node_.param(
        "formal_lower/constraint_margin",
        config.constraint_margin, config.constraint_margin);
    private_node_.param(
        "formal_lower/sustained_saturation_seconds",
        config.sustained_saturation_seconds,
        config.sustained_saturation_seconds);
    private_node_.param(
        "formal_lower/golden_vectors_verified",
        config.golden_vectors_verified, false);
    return config;
  }

  void enforceFakeOnlyGates(
      const UpperReferenceConfig& upper,
      const LowerChannelConfig& lower) {
    std::string transport;
    private_node_.param<std::string>(
        "platform_transport_type", transport, "");
    bool upper_algorithm = false;
    bool upper_hardware = false;
    bool lower_algorithm = false;
    bool lower_hardware = false;
    private_node_.param(
        "formal_upper/algorithm_execution_authorized",
        upper_algorithm, false);
    private_node_.param(
        "formal_upper/hardware_execution_authorized",
        upper_hardware, false);
    private_node_.param(
        "formal_lower/algorithm_execution_authorized",
        lower_algorithm, false);
    private_node_.param(
        "formal_lower/hardware_execution_authorized",
        lower_hardware, false);
    bool preregistered = false;
    private_node_.param(
        "formal_upper/fixed_speed_preregistered",
        preregistered, false);
    FormalExecutionGateInput gate;
    gate.transport_type = transport;
    gate.command_publication_authorized =
        command_publication_authorized_;
    gate.upper_algorithm_authorized = upper_algorithm;
    gate.lower_algorithm_authorized = lower_algorithm;
    gate.upper_golden_verified = upper.golden_vectors_verified;
    gate.lower_golden_verified = lower.golden_vectors_verified;
    gate.upper_hardware_authorized = upper_hardware;
    gate.lower_hardware_authorized = lower_hardware;
    gate.m4_selected = upper.mode == UpperMode::kM4;
    gate.m4_speed_preregistered = preregistered;
    const auto result = evaluateFormalFakeGate(gate);
    if (!result.allowed) throw std::runtime_error(result.reason);
  }

  void receiveState(const agv_msgs::CooperativeState::ConstPtr& message) {
    state_ = *message;
    state_receive_time_ = ros::Time::now();
    has_state_ = true;
  }

  void receiveCapability(
      std::size_t index,
      const agv_msgs::CapabilityReport::ConstPtr& message) {
    if (message->robot_id != index + 1U) return;
    capability_[index] = *message;
    capability_receive_time_[index] = ros::Time::now();
    has_capability_[index] = true;
  }

  bool refreshFakeChassisBinding() {
    FakeChassisBindingInput input;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const std::string parameter =
          "/agv" + std::to_string(index + 1U) +
          "/chassis_controller/transport_type";
      input.transport_parameter_present[index] =
          node_.getParam(parameter, input.transport_type[index]);
    }
    const auto result = evaluateFakeChassisBinding(input);
    if (result.status == FakeChassisBindingStatus::kAllowed) {
      if (!command_outputs_ready_) {
        for (std::size_t index = 0; index < kRobotCount; ++index) {
          command_publishers_[index] =
              node_.advertise<agv_msgs::ChassisCommand>(
                  "/agv" + std::to_string(index + 1U) +
                      "/chassis_command",
                  1, false);
        }
        command_outputs_ready_ = true;
        ROS_INFO(
            "Formal fake command publishers enabled after all three "
            "chassis transport parameters were verified as fake");
      }
      return true;
    }

    if (command_outputs_ready_) {
      for (auto& publisher : command_publishers_) publisher.shutdown();
      command_outputs_ready_ = false;
    }
    if (result.status == FakeChassisBindingStatus::kRejected) {
      ROS_FATAL(
          "Formal fake algorithm rejected chassis binding: %s",
          result.reason.c_str());
      ros::shutdown();
    } else {
      ROS_WARN_THROTTLE(
          1.0, "Formal fake algorithm has no command authority: %s",
          result.reason.c_str());
    }
    return false;
  }

  bool inputsUsable(const ros::Time& now) const {
    if (!has_state_ ||
        (now - state_receive_time_).toSec() > maximum_state_age_) {
      return false;
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!state_.robot_pose_valid[index] ||
          !state_.support_pose_valid[index] ||
          !state_.path_state_valid[index] ||
          !has_capability_[index] ||
          (now - capability_receive_time_[index]).toSec() >
              maximum_capability_age_) {
        return false;
      }
    }
    return !require_valid_load_state_ ||
        (state_.load_pose_valid && state_.load_path_state_valid);
  }

  WheelCapability wheelCapability(std::size_t index) const {
    const auto& message = capability_[index];
    WheelCapability value;
    value.maximum_velocity_left =
        message.max_wheel_linear_velocity_left;
    value.maximum_velocity_right =
        message.max_wheel_linear_velocity_right;
    value.maximum_acceleration_left =
        message.max_wheel_linear_acceleration_left;
    value.maximum_acceleration_right =
        message.max_wheel_linear_acceleration_right;
    value.maximum_deceleration_left =
        message.max_wheel_linear_deceleration_left;
    value.maximum_deceleration_right =
        message.max_wheel_linear_deceleration_right;
    value.source_stamp = message.header.stamp;
    value.source_sequence = message.capability_seq;
    return value;
  }

  CapabilityGeometry capabilityGeometry(std::size_t index) const {
    const double step = std::max(path_.length() / 10000.0, 1.0e-5);
    const double s0 = std::clamp(reference_progress_, 0.0, path_.length());
    const double sm = std::max(0.0, s0 - step);
    const double sp = std::min(path_.length(), s0 + step);
    const auto minus = geometry_.sample(index, sm);
    const auto current = geometry_.sample(index, s0);
    const auto plus = geometry_.sample(index, sp);
    const double ds = std::max(sp - sm, 1.0e-9);
    const auto chassis_position =
        [&](const SupportSample& support) -> Eigen::Vector2d {
      const Eigen::Vector2d rotated_offset =
          Eigen::Rotation2Dd(support.heading) *
          tracker_config_.base_to_support[index];
      return support.position - rotated_offset;
    };
    const Eigen::Vector2d p_minus = chassis_position(minus);
    const Eigen::Vector2d p_current = chassis_position(current);
    const Eigen::Vector2d p_plus = chassis_position(plus);
    CapabilityGeometry result;
    result.speed_scale = (p_plus - p_minus).norm() / ds;
    const Eigen::Vector2d tangent_minus = p_current - p_minus;
    const Eigen::Vector2d tangent_plus = p_plus - p_current;
    if (tangent_minus.squaredNorm() > 1.0e-18 &&
        tangent_plus.squaredNorm() > 1.0e-18) {
      const double heading_minus =
          std::atan2(tangent_minus.y(), tangent_minus.x());
      const double heading_plus =
          std::atan2(tangent_plus.y(), tangent_plus.x());
      result.heading_rate =
          tf2NormalizeAngle(heading_plus - heading_minus) /
          (0.5 * ds);
    } else {
      result.heading_rate =
          tf2NormalizeAngle(plus.heading - minus.heading) / ds;
    }
    result.wheel_separation = tracker_config_.wheel_separation[index];
    return result;
  }

  void publishZero(const ros::Time& stamp) {
    if (!command_outputs_ready_) return;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      agv_msgs::ChassisCommand command;
      command.header.stamp = stamp;
      command.robot_id = static_cast<std::uint8_t>(index + 1U);
      command.command_seq = ++command_sequence_[index];
      command.control_mode = 1U;
      command.experiment_id = experiment_id_;
      command.method_id =
          std::string(upperModeName(upper_mode_)) + "_" +
          lowerModeName(lower_mode_);
      command_publishers_[index].publish(command);
    }
  }

  void publishDebug(
      bool valid, const UpperReferenceOutput& upper,
      const DistributedReferenceOutput& distributed,
      const std::array<LowerChannelOutput, 3>& lower) {
    std_msgs::Float64MultiArray message;
    message.layout.dim.resize(1);
    message.layout.dim[0].label =
        "formal_algorithm_state_v1:header9+3x27";
    message.layout.dim[0].size =
        kDebugHeaderFields + kRobotCount * kDebugFieldsPerRobot;
    message.layout.dim[0].stride = message.layout.dim[0].size;
    message.data.reserve(message.layout.dim[0].size);
    message.data.push_back(valid ? 1.0 : 0.0);
    message.data.push_back(static_cast<double>(upper_mode_));
    message.data.push_back(static_cast<double>(lower_mode_));
    message.data.push_back(upper.common_lower);
    message.data.push_back(upper.common_upper);
    message.data.push_back(current_velocity_reference_);
    message.data.push_back(reference_progress_);
    message.data.push_back(current_acceleration_reference_);
    message.data.push_back(ros::Time::now().toSec());
    for (std::size_t i = 0; i < kRobotCount; ++i) {
      const auto& u = upper.agents[i];
      const auto& d = distributed;
      const auto& l = lower[i];
      const std::array<double, kDebugFieldsPerRobot> fields{{
          u.boundary.lower, u.boundary.upper, u.robust_margin,
          u.risk_factor, u.risk_signal,
          distributed_state_.position[i],
          distributed_state_.velocity[i],
          distributed_state_.auxiliary[i],
          d.position_disagreement[i], d.velocity_disagreement[i],
          d.auxiliary_disagreement[i], d.acceleration[i],
          state_.s_actual[i], state_.s_dot_actual[i],
          l.position_error, l.velocity_error, l.transformed_error,
          l.transformation_gain, l.inverse_gain_term, l.composite_error,
          l.input_raw, l.input_limited,
          l.parameter_estimate[0], l.parameter_estimate[1],
          l.disturbance_estimate,
          l.input_limit_active ? 1.0 : 0.0,
          l.sustained_physical_saturation ? 1.0 : 0.0}};
      message.data.insert(
          message.data.end(), fields.begin(), fields.end());
    }
    debug_publisher_.publish(message);
  }

  void publishPublicState(
      const ros::Time& stamp, bool valid,
      const std::array<LowerChannelOutput, 3>& lower,
      const std::array<PlanarTrackingResult, 3>& tracking) {
    agv_msgs::PathReference reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = "world";
    reference.path_id = "s_curve";
    reference.path_version = 1U;
    reference.load_path_progress_reference = reference_progress_;
    reference.load_path_velocity_reference =
        valid ? current_velocity_reference_ : 0.0;
    reference.load_path_acceleration_reference =
        valid ? current_acceleration_reference_ : 0.0;
    const auto load = path_.sample(reference_progress_);
    reference.load_pose_reference =
        poseMessage({load.position, load.heading});
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (tracking[index].valid) {
        reference.support_pose_reference[index] =
            poseMessage(tracking[index].support_pose_reference);
        reference.chassis_linear_velocity_feedforward[index] =
            tracking[index].linear_velocity_feedforward;
        reference.chassis_angular_velocity_feedforward[index] =
            tracking[index].angular_velocity_feedforward;
      }
    }
    reference_publisher_.publish(reference);

    agv_msgs::ControllerState controller;
    controller.header.stamp = stamp;
    controller.header.frame_id = "world";
    controller.experiment_id = experiment_id_;
    controller.method_id =
        std::string(upperModeName(upper_mode_)) + "_" +
        lowerModeName(lower_mode_);
    controller.common_velocity_lower_bound = current_upper_.common_lower;
    controller.common_velocity_upper_bound = current_upper_.common_upper;
    controller.common_load_velocity_reference =
        valid ? current_velocity_reference_ : 0.0;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      controller.path_progress_actual[index] = state_.s_actual[index];
      controller.path_velocity_actual[index] = state_.s_dot_actual[index];
      controller.path_progress_execute_reference[index] =
          reference_progress_;
      controller.path_velocity_execute_reference[index] =
          valid ? lower[index].channel_velocity_command : 0.0;
      controller.channel_input_raw[index] =
          valid ? lower[index].input_raw : 0.0;
      controller.channel_input_limited[index] =
          valid ? lower[index].input_limited : 0.0;
      controller.channel_input_limit_active[index] =
          valid && lower[index].input_limit_active;
    }
    controller_publisher_.publish(controller);
  }

  void step(const ros::TimerEvent& event) {
    const ros::Time now = ros::Time::now();
    double dt = (event.current_real - event.last_real).toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
      dt = 1.0 / publish_rate_;
    }
    std::array<LowerChannelOutput, 3> lower;
    std::array<PlanarTrackingResult, 3> tracking;
    DistributedReferenceOutput distributed = current_distributed_;
    if (!refreshFakeChassisBinding()) {
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    if (!inputsUsable(now)) {
      ROS_WARN_THROTTLE(
          1.0, "Formal fake algorithm fail-zero: input state is missing, "
          "invalid or stale");
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }

    std::array<WheelCapability, 3> wheels;
    std::array<CapabilityGeometry, 3> geometries;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      wheels[index] = wheelCapability(index);
      geometries[index] = capabilityGeometry(index);
    }
    const auto fleet = capability_mapper_.mapFleet(wheels, geometries);
    if (!fleet.valid) {
      ROS_WARN_THROTTLE(
          1.0,
          "Formal fake algorithm fail-zero: capability mapping failed "
          "(geometry=[%.6f,%.6f; %.6f,%.6f; %.6f,%.6f], "
          "wheel_velocity=[%.6f,%.6f; %.6f,%.6f; %.6f,%.6f])",
          geometries[0].speed_scale, geometries[0].heading_rate,
          geometries[1].speed_scale, geometries[1].heading_rate,
          geometries[2].speed_scale, geometries[2].heading_rate,
          wheels[0].maximum_velocity_left,
          wheels[0].maximum_velocity_right,
          wheels[1].maximum_velocity_left,
          wheels[1].maximum_velocity_right,
          wheels[2].maximum_velocity_left,
          wheels[2].maximum_velocity_right);
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }

    std::array<UpperAgentInput, 3> upper_input;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      auto& value = upper_input[index];
      value.candidate_velocity = distributed_state_.velocity[index];
      value.mapped_upper_capability =
          fleet.robots[index].actuator_upper_velocity;
      const double capability_margin = normalisedMargin(
          value.mapped_upper_capability - capability_reserve_ -
              std::abs(distributed_state_.velocity[index]),
          capability_safe_margin_);
      const double input_margin = normalisedMargin(
          std::min(
              fleet.robots[index].available_acceleration,
              fleet.robots[index].available_deceleration) -
              std::abs(previous_input_raw_[index]),
          input_safe_margin_);
      const double wheel_margin = normalisedMargin(
          std::min(
              capability_[index].max_wheel_linear_velocity_left -
                  std::abs(previous_wheel_raw_[index][0]),
              capability_[index].max_wheel_linear_velocity_right -
                  std::abs(previous_wheel_raw_[index][1])),
          wheel_safe_margin_);
      const double position_margin = normalisedMargin(
          position_limit_ - previous_position_error_[index],
          position_safe_margin_);
      value.normalised_causal_margins = {{
          capability_margin, input_margin, wheel_margin,
          position_margin, 1.0}};
    }
    current_upper_ = upper_generator_->step(upper_input, dt);
    if (!current_upper_.valid) {
      ROS_WARN_THROTTLE(
          1.0, "Formal fake algorithm fail-zero: upper reference is invalid");
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }

    if (current_upper_.updated) {
      DistributedReferenceInput reference_input;
      reference_input.leader_position = leader_position_;
      reference_input.leader_velocity = leader_velocity_;
      reference_input.leader_acceleration = leader_acceleration_;
      reference_input.dt_seconds =
          dt * static_cast<double>(upper_ticks_per_update_);
      for (std::size_t index = 0; index < kRobotCount; ++index) {
        reference_input.boundary[index] =
            current_upper_.agents[index].boundary;
        reference_input.boundary_derivative[index] = {
            (reference_input.boundary[index].lower -
             previous_boundary_[index].lower) /
                reference_input.dt_seconds,
            (reference_input.boundary[index].upper -
             previous_boundary_[index].upper) /
                reference_input.dt_seconds};
        previous_boundary_[index] = reference_input.boundary[index];
      }
      distributed = stepDistributedReference(
          distributed_config_, distributed_state_, reference_input);
      if (!distributed.valid) {
        ROS_WARN_THROTTLE(
            1.0, "Formal fake algorithm fail-zero: distributed reference "
            "is invalid");
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishDebug(false, current_upper_, distributed, lower);
        return;
      }
      current_distributed_ = distributed;
      distributed_state_ = distributed.next;
      current_velocity_reference_ =
          upper_mode_ == UpperMode::kM4
              ? current_upper_.common_velocity
              : distributed.common_velocity;
      current_acceleration_reference_ =
          upper_mode_ == UpperMode::kM4
              ? 0.0
              : (distributed.acceleration[0] +
                 distributed.acceleration[1] +
                 distributed.acceleration[2]) /
                    static_cast<double>(kRobotCount);
    }
    leader_position_ += dt * leader_velocity_;
    reference_progress_ = std::min(
        target_progress_,
        reference_progress_ + dt * current_velocity_reference_);
    if (reference_progress_ >= target_progress_) {
      current_velocity_reference_ = 0.0;
      current_acceleration_reference_ = 0.0;
    }

    std::array<PlanarPose, 3> robot_pose;
    std::array<PlanarPose, 3> support_pose;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      LowerChannelInput input;
      input.position_actual = state_.s_actual[index];
      input.velocity_actual = state_.s_dot_actual[index];
      input.position_reference = reference_progress_;
      input.velocity_reference = current_velocity_reference_;
      input.acceleration_reference = current_acceleration_reference_;
      input.velocity_lower_bound = velocity_lower_bound_;
      input.velocity_upper_bound = velocity_upper_bound_;
      input.available_acceleration =
          fleet.robots[index].available_acceleration;
      input.available_deceleration =
          fleet.robots[index].available_deceleration;
      input.regressor = {{
          std::sin(input.position_actual),
          input.velocity_actual * input.velocity_actual}};
      input.physical_wheel_saturation =
          capability_[index].speed_limit_active_left ||
          capability_[index].speed_limit_active_right ||
          capability_[index].accel_limit_active_left ||
          capability_[index].accel_limit_active_right ||
          capability_[index].decel_limit_active_left ||
          capability_[index].decel_limit_active_right;
      input.dt_seconds = dt;
      lower[index] = lower_controllers_[index]->step(input);
      if (!lower[index].valid) {
        ROS_WARN_THROTTLE(
            1.0, "Formal fake algorithm fail-zero: lower controller for "
            "agv%zu is invalid", index + 1U);
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishDebug(false, current_upper_, distributed, lower);
        return;
      }
      robot_pose[index] = poseValue(state_.robot_pose[index]);
      support_pose[index] = poseValue(state_.support_pose[index]);
      PlanarTrackingInput tracking_input;
      tracking_input.support_index = index;
      tracking_input.load_progress_reference = reference_progress_;
      tracking_input.channel_velocity_command =
          lower[index].channel_velocity_command;
      tracking_input.robot_pose_actual = robot_pose[index];
      tracking_input.support_pose_actual = support_pose[index];
      tracking[index] = tracker_.track(tracking_input);
      if (!tracking[index].valid) {
        ROS_WARN_THROTTLE(
            1.0, "Formal fake algorithm fail-zero: planar tracker for "
            "agv%zu is invalid", index + 1U);
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishDebug(false, current_upper_, distributed, lower);
        return;
      }
    }

    for (std::size_t index = 0; index < kRobotCount; ++index) {
      agv_msgs::ChassisCommand command;
      command.header.stamp = now;
      command.robot_id = static_cast<std::uint8_t>(index + 1U);
      command.command_seq = ++command_sequence_[index];
      command.control_mode = 1U;
      command.linear_velocity_reference =
          tracking[index].linear_velocity_raw;
      command.angular_velocity_reference =
          tracking[index].angular_velocity_raw;
      command.wheel_linear_velocity_left_raw =
          tracking[index].wheel_linear_velocity_left_raw;
      command.wheel_linear_velocity_right_raw =
          tracking[index].wheel_linear_velocity_right_raw;
      command.experiment_id = experiment_id_;
      command.method_id =
          std::string(upperModeName(upper_mode_)) + "_" +
          lowerModeName(lower_mode_);
      command_publishers_[index].publish(command);
      previous_input_raw_[index] = lower[index].input_raw;
      previous_wheel_raw_[index] = {{
          tracking[index].wheel_linear_velocity_left_raw,
          tracking[index].wheel_linear_velocity_right_raw}};
      previous_position_error_[index] = std::hypot(
          tracking[index].longitudinal_error,
          tracking[index].lateral_error);
    }
    publishPublicState(now, true, lower, tracking);
    publishDebug(true, current_upper_, distributed, lower);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  SCurvePath path_;
  SupportGeometry geometry_;
  PlanarTrackerConfig tracker_config_;
  PlanarSupportTracker tracker_;
  CapabilityMapper capability_mapper_;
  std::unique_ptr<UpperReferenceGenerator> upper_generator_;
  std::array<std::unique_ptr<LowerChannelController>, 3> lower_controllers_;
  DistributedReferenceConfig distributed_config_;
  DistributedReferenceState distributed_state_;
  DistributedReferenceOutput current_distributed_;
  UpperReferenceOutput current_upper_;
  UpperMode upper_mode_{UpperMode::kM1};
  LowerMode lower_mode_{LowerMode::kR1};

  ros::Subscriber state_subscriber_;
  std::array<ros::Subscriber, 3> capability_subscribers_;
  std::array<ros::Publisher, 3> command_publishers_;
  ros::Publisher reference_publisher_;
  ros::Publisher controller_publisher_;
  ros::Publisher debug_publisher_;
  ros::Timer timer_;
  agv_msgs::CooperativeState state_;
  std::array<agv_msgs::CapabilityReport, 3> capability_;
  ros::Time state_receive_time_;
  std::array<ros::Time, 3> capability_receive_time_;
  bool has_state_{false};
  std::array<bool, 3> has_capability_{{false, false, false}};
  bool require_valid_load_state_{true};
  bool command_publication_authorized_{false};
  bool command_outputs_ready_{false};
  std::size_t upper_ticks_per_update_{4U};

  std::array<std::uint32_t, 3> command_sequence_{{0U, 0U, 0U}};
  std::array<DynamicBoundaryState, 3> previous_boundary_;
  std::array<double, 3> previous_input_raw_{{0.0, 0.0, 0.0}};
  std::array<std::array<double, 2>, 3> previous_wheel_raw_{{
      {{0.0, 0.0}}, {{0.0, 0.0}}, {{0.0, 0.0}}}};
  std::array<double, 3> previous_position_error_{{0.0, 0.0, 0.0}};
  std::array<double, 2> lower_initial_parameter_{{0.0, 0.0}};
  double lower_initial_disturbance_{0.08};

  double publish_rate_{100.0};
  double maximum_state_age_{0.15};
  double maximum_capability_age_{0.20};
  double reference_progress_{0.0};
  double target_progress_{1.0};
  double leader_position_{0.0};
  double leader_velocity_{0.08};
  double leader_acceleration_{0.0};
  double current_velocity_reference_{0.08};
  double current_acceleration_reference_{0.0};
  double velocity_lower_bound_{-0.15};
  double velocity_upper_bound_{0.58};
  double capability_reserve_{0.015};
  double capability_safe_margin_{0.12};
  double input_safe_margin_{0.40};
  double wheel_safe_margin_{0.08};
  double position_limit_{0.12};
  double position_safe_margin_{0.06};
  std::string experiment_id_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "formal_fake_algorithm");
  try {
    multi_agv_control::FormalFakeAlgorithmNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start formal fake algorithm: %s", error.what());
    return 1;
  }
  return 0;
}
