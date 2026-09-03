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
#include <agv_msgs/ChassisFeedback.h>
#include <agv_msgs/ControllerState.h>
#include <agv_msgs/CooperativeState.h>
#include <agv_msgs/PathReference.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Scalar.h>

#include "multi_agv_control/capability_mapper.hpp"
#include "multi_agv_control/formal_execution_gate.hpp"
#include "multi_agv_control/lower_channel_controller.hpp"
#include "multi_agv_control/m2b_controller.hpp"
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

std::array<double, 3> optionalArray3(
    const ros::NodeHandle& node, const std::string& name,
    const std::array<double, 3>& fallback) {
  return node.hasParam(name) ? array3(node, name) : fallback;
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
        execution_adapter_(tracker_, loadFleetExecutionConfig()),
        capability_mapper_(loadCapabilityReserve()) {
    loadRuntime();
    const auto upper_config = loadUpperConfig();
    const auto lower_config = loadLowerConfig();
    enforceExecutionGates(upper_config, lower_config);
    upper_mode_ = upper_config.mode;
    lower_mode_ = lower_config.mode;
    m2b_selected_ =
        upper_mode_ == UpperMode::kM2b && lower_mode_ == LowerMode::kM2b;
    if ((upper_mode_ == UpperMode::kM2b) !=
        (lower_mode_ == LowerMode::kM2b)) {
      throw std::runtime_error(
          "M2b is a complete literature method: upper and lower must both "
          "select M2b");
    }
    upper_generator_ =
        std::make_unique<UpperReferenceGenerator>(upper_config);
    if (m2b_selected_) {
      m2b_controller_ = std::make_unique<M2bController>(loadM2bConfig());
      M2bGeneratorState initial;
      initial.position = distributed_state_.position;
      initial.velocity = distributed_state_.velocity;
      initial.auxiliary = distributed_state_.auxiliary;
      m2b_controller_->reset(initial);
    }
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
    recorder_armed_subscriber_ = node_.subscribe(
        "/experiment_recorder/armed", 1,
        &FormalFakeAlgorithmNode::receiveRecorderArmed, this);
    recorder_method_subscriber_ = node_.subscribe(
        "/experiment_recorder/method_id", 1,
        &FormalFakeAlgorithmNode::receiveRecorderMethod, this);
    watchdog_subscriber_ = node_.subscribe(
        "/multi_agv/software_watchdog_ok", 1,
        &FormalFakeAlgorithmNode::receiveWatchdog, this);
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      capability_subscribers_[index] = node_.subscribe<agv_msgs::CapabilityReport>(
          "/agv" + std::to_string(index + 1U) + "/capability_report", 5,
          [this, index](const agv_msgs::CapabilityReport::ConstPtr& message) {
            receiveCapability(index, message);
          });
      feedback_subscribers_[index] = node_.subscribe<agv_msgs::ChassisFeedback>(
          "/agv" + std::to_string(index + 1U) + "/chassis_feedback", 5,
          [this, index](const agv_msgs::ChassisFeedback::ConstPtr& message) {
            receiveFeedback(index, message);
          });
    }
    reference_publisher_ = node_.advertise<agv_msgs::PathReference>(
        "/multi_agv/path_reference", 5, false);
    controller_publisher_ = node_.advertise<agv_msgs::ControllerState>(
        "/multi_agv/controller_state", 10, false);
    debug_publisher_ = node_.advertise<std_msgs::Float64MultiArray>(
        "/multi_agv/formal_algorithm_state", 10, false);
    execution_limiter_publisher_ =
        node_.advertise<std_msgs::Float64MultiArray>(
            "/multi_agv/formal_execution_limiter_state", 10, false);
    m2b_debug_publisher_ = node_.advertise<std_msgs::Float64MultiArray>(
        "/multi_agv/m2b_algorithm_state", 10, false);
    timer_ = node_.createTimer(
        ros::Duration(1.0 / publish_rate_),
        &FormalFakeAlgorithmNode::step, this);
    ROS_WARN(
        "Formal algorithm integration ready: transport=%s upper=%s lower=%s "
        "recorder_required=%s",
        transport_type_.c_str(), upperModeName(upper_mode_),
        lowerModeName(lower_mode_), require_recorder_armed_ ? "true" : "false");
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

  FleetPlanarExecutionConfig loadFleetExecutionConfig() {
    const std::string root = "formal_fake_runtime/tracker/";
    FleetPlanarExecutionConfig config;
    config.wheel_separation = tracker_config_.wheel_separation;
    config.longitudinal_gain = optionalArray3(
        private_node_, root + "longitudinal_gain_per_robot",
        {{tracker_config_.longitudinal_gain, tracker_config_.longitudinal_gain,
          tracker_config_.longitudinal_gain}});
    config.lateral_gain = optionalArray3(
        private_node_, root + "lateral_gain_per_robot",
        {{tracker_config_.lateral_gain, tracker_config_.lateral_gain,
          tracker_config_.lateral_gain}});
    config.heading_gain = optionalArray3(
        private_node_, root + "heading_gain_per_robot",
        {{tracker_config_.heading_gain, tracker_config_.heading_gain,
          tracker_config_.heading_gain}});
    config.angular_feedforward_scale_positive = optionalArray3(
        private_node_, root + "angular_feedforward_scale_positive",
        {{1.0, 1.0, 1.0}});
    config.angular_feedforward_scale_negative = optionalArray3(
        private_node_, root + "angular_feedforward_scale_negative",
        {{1.0, 1.0, 1.0}});
    config.curvature_preview_seconds_positive = optionalArray3(
        private_node_, root + "curvature_preview_seconds_positive",
        {{0.0, 0.0, 0.0}});
    config.curvature_preview_seconds_negative = optionalArray3(
        private_node_, root + "curvature_preview_seconds_negative",
        {{0.0, 0.0, 0.0}});
    private_node_.param(
        root + "formation_longitudinal_gain",
        config.formation_longitudinal_gain, 0.0);
    private_node_.param(
        root + "formation_lateral_gain",
        config.formation_lateral_gain, 0.0);
    private_node_.param(
        root + "formation_heading_gain",
        config.formation_heading_gain, 0.0);
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
        root + "maximum_feedback_age", maximum_feedback_age_, 0.25);
    private_node_.param(
        root + "minimum_battery_voltage", minimum_battery_voltage_, 10.0);
    private_node_.param(
        root + "emergency_abort_limit",
        emergency_abort_limit_, 0.12);
    private_node_.param(
        root + "execution/startup_ramp_seconds",
        startup_ramp_seconds_, 1.0);
    private_node_.param(
        root + "execution/emergency_abort_persistence_seconds",
        emergency_abort_persistence_seconds_, 0.10);
    private_node_.param(
        root + "execution/initialization_hold_seconds",
        initialization_hold_seconds_, 0.30);
    private_node_.param(
        root + "execution/initialization_max_wheel_speed",
        initialization_max_wheel_speed_, 0.015);
    private_node_.param(
        root + "execution/fleet_scale_recovery_rate_per_second",
        fleet_scale_recovery_rate_per_second_, 0.50);
    private_node_.param(
        root + "require_valid_load_state", require_valid_load_state_, true);
    private_node_.param(
        root + "require_recorder_armed", require_recorder_armed_, false);
    private_node_.param(
        root + "serial_execution_authorized",
        serial_execution_authorized_, false);
    private_node_.param(
        root + "confirm_test_area_clear", confirm_test_area_clear_, false);
    private_node_.param(
        root + "confirm_wheels_on_floor",
        confirm_wheels_on_floor_, false);
    private_node_.param(
        root + "confirm_unloaded_40cm_fixture",
        confirm_unloaded_fixture_, false);
    private_node_.param(
        root + "maximum_recorder_armed_age",
        maximum_recorder_armed_age_, 0.50);
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
    current_velocity_reference_ = leader_velocity_;
    current_acceleration_reference_ = leader_acceleration_;
    unramped_velocity_reference_ = leader_velocity_;
    unramped_acceleration_reference_ = leader_acceleration_;
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
        root + "risk/require_software_watchdog",
        require_software_watchdog_, false);
    private_node_.param(
        root + "risk/maximum_watchdog_age",
        maximum_watchdog_age_, 0.25);
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
        !(maximum_feedback_age_ > 0.0) ||
        !(minimum_battery_voltage_ > 0.0) ||
        !(emergency_abort_limit_ > 0.0) ||
        startup_ramp_seconds_ < 0.0 ||
        !(emergency_abort_persistence_seconds_ > 0.0) ||
        !(initialization_hold_seconds_ > 0.0) ||
        !(initialization_max_wheel_speed_ > 0.0) ||
        !(fleet_scale_recovery_rate_per_second_ > 0.0) ||
        !(maximum_recorder_armed_age_ > 0.0) ||
        !(maximum_watchdog_age_ > 0.0) ||
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

  M2bConfig loadM2bConfig() {
    M2bConfig config;
    const std::string root = "formal_m2b/";
    private_node_.param(root + "alpha", config.alpha, config.alpha);
    private_node_.param(root + "beta", config.beta, config.beta);
    private_node_.param(root + "l0", config.l0, config.l0);
    private_node_.param(root + "l1", config.l1, config.l1);
    private_node_.param(root + "l2", config.l2, config.l2);
    private_node_.param(root + "l3", config.l3, config.l3);
    config.h = array3(private_node_, root + "h");
    config.k1 = array3(private_node_, root + "k1");
    config.k2 = array3(private_node_, root + "k2");
    config.fixed_acceleration_limit = array3(
        private_node_, root + "fixed_acceleration_limit");
    config.fixed_deceleration_limit = array3(
        private_node_, root + "fixed_deceleration_limit");
    private_node_.param(
        root + "generator_guard", config.generator_guard,
        config.generator_guard);
    private_node_.param(
        root + "mapping_guard", config.mapping_guard,
        config.mapping_guard);
    private_node_.param(
        root + "auxiliary_boundary_layer",
        config.auxiliary_boundary_layer,
        config.auxiliary_boundary_layer);
    int substeps = static_cast<int>(config.generator_substeps);
    private_node_.param(root + "generator_substeps", substeps, substeps);
    if (substeps <= 0) {
      throw std::runtime_error("formal_m2b/generator_substeps must be positive");
    }
    config.generator_substeps = static_cast<std::size_t>(substeps);
    private_node_.param(
        root + "golden_vectors_verified",
        config.golden_vectors_verified, false);
    if (!config.golden_vectors_verified) {
      throw std::runtime_error(
          "M2b execution refused until golden vectors are verified");
    }
    return config;
  }

  void enforceExecutionGates(
      const UpperReferenceConfig& upper,
      const LowerChannelConfig& lower) {
    private_node_.param<std::string>(
        "platform_transport_type", transport_type_, "");
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
    gate.transport_type = transport_type_;
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
    gate.serial_execution_authorized = serial_execution_authorized_;
    gate.m1_r1_selected =
        upper.mode == UpperMode::kM1 && lower.mode == LowerMode::kR1;
    gate.recorder_required = require_recorder_armed_;
    gate.test_area_confirmed = confirm_test_area_clear_;
    gate.wheels_on_floor_confirmed = confirm_wheels_on_floor_;
    gate.unloaded_fixture_confirmed = confirm_unloaded_fixture_;
    const auto result = transport_type_ == "fake"
        ? evaluateFormalFakeGate(gate)
        : evaluateFormalSerialM1R1Gate(gate);
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

  void receiveFeedback(
      std::size_t index,
      const agv_msgs::ChassisFeedback::ConstPtr& message) {
    if (message->robot_id != index + 1U) return;
    feedback_[index] = *message;
    feedback_receive_time_[index] = ros::Time::now();
    has_feedback_[index] = true;
    if (transport_type_ == "serial" &&
        !command_sequence_synchronised_[index]) {
      if (!seedCommandSequenceFromFeedback(
              message->command_seq_applied,
              &command_sequence_[index])) {
        safety_abort_latched_ = true;
        ROS_ERROR(
            "Formal serial safety abort latched: agv%zu command sequence "
            "cannot be handed off from command_seq_applied=%u",
            index + 1U, message->command_seq_applied);
        return;
      }
      command_sequence_synchronised_[index] = true;
      ROS_INFO(
          "Formal serial command sequence handoff: agv%zu "
          "command_seq_applied=%u, next_command_seq=%u",
          index + 1U, message->command_seq_applied,
          message->command_seq_applied + 1U);
    }
  }

  bool commandSequencesReady() const {
    return transport_type_ != "serial" ||
        std::all_of(
            command_sequence_synchronised_.begin(),
            command_sequence_synchronised_.end(),
            [](bool value) { return value; });
  }

  bool refreshChassisBinding() {
    FakeChassisBindingInput input;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const std::string parameter =
          "/agv" + std::to_string(index + 1U) +
          "/chassis_controller/transport_type";
      input.transport_parameter_present[index] =
          node_.getParam(parameter, input.transport_type[index]);
    }
    const auto result = evaluateChassisBinding(input, transport_type_);
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
            "Formal command publishers enabled after all three chassis "
            "transport parameters were verified as %s",
            transport_type_.c_str());
      }
      return true;
    }

    if (command_outputs_ready_) {
      for (auto& publisher : command_publishers_) publisher.shutdown();
      command_outputs_ready_ = false;
    }
    if (result.status == FakeChassisBindingStatus::kRejected) {
      ROS_FATAL(
          "Formal algorithm rejected chassis binding: %s",
          result.reason.c_str());
      ros::shutdown();
    } else {
      ROS_WARN_THROTTLE(
          1.0, "Formal algorithm has no command authority: %s",
          result.reason.c_str());
    }
    return false;
  }

  std::string inputFailureReason(const ros::Time& now) const {
    if (safety_abort_latched_) {
      return "serial safety abort remains latched";
    }
    if (!has_state_) return "CooperativeState is missing";
    if ((now - state_receive_time_).toSec() > maximum_state_age_) {
      return "CooperativeState is stale";
    }
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      if (!state_.robot_pose_valid[index] ||
          !state_.support_pose_valid[index] ||
          !state_.path_state_valid[index]) {
        return "agv" + std::to_string(index + 1U) +
            " CooperativeState fields are invalid";
      }
      if (!has_capability_[index]) {
        return "agv" + std::to_string(index + 1U) +
            " CapabilityReport is missing";
      }
      if ((now - capability_receive_time_[index]).toSec() >
          maximum_capability_age_) {
        return "agv" + std::to_string(index + 1U) +
            " CapabilityReport is stale";
      }
      if (transport_type_ == "serial") {
        if (!has_feedback_[index]) {
          return "agv" + std::to_string(index + 1U) +
              " ChassisFeedback is missing";
        }
        if ((now - feedback_receive_time_[index]).toSec() >
            maximum_feedback_age_) {
          return "agv" + std::to_string(index + 1U) +
              " ChassisFeedback is stale";
        }
        if (!std::isfinite(feedback_[index].battery_voltage)) {
          return "agv" + std::to_string(index + 1U) +
              " battery voltage is invalid";
        }
        if (feedback_[index].battery_voltage < minimum_battery_voltage_) {
          return "agv" + std::to_string(index + 1U) +
              " battery voltage is below the bound";
        }
      }
    }
    if (require_valid_load_state_ &&
        (!state_.load_pose_valid || !state_.load_path_state_valid)) {
      return "virtual load pose or path state is invalid";
    }
    return "";
  }

  bool trackingPassesSerialEmergencyGate(
      const std::array<PlanarTrackingResult, 3>& tracking, double dt) {
    if (transport_type_ != "serial") return true;
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const double left = tracking[index].wheel_linear_velocity_left_raw;
      const double right = tracking[index].wheel_linear_velocity_right_raw;
      const auto assessment = assessSerialWheelDemand(
          left, right,
          capability_[index].max_wheel_linear_velocity_left,
          capability_[index].max_wheel_linear_velocity_right,
          emergency_abort_limit_);
      if (assessment.emergency_abort) {
        if (assessment.reason ==
            "raw wheel demand exceeds emergency abort limit") {
          emergency_violation_duration_[index] += dt;
          if (emergency_violation_duration_[index] <
              emergency_abort_persistence_seconds_) {
            ROS_WARN_THROTTLE(
                0.25,
                "agv%zu raw wheel emergency threshold transient: "
                "demand=[%.6f, %.6f] limit=%.6f m/s duration=%.3f/%.3f s; "
                "continuing with common fleet limiting",
                index + 1U, left, right, emergency_abort_limit_,
                emergency_violation_duration_[index],
                emergency_abort_persistence_seconds_);
            continue;
          }
        }
        safety_abort_latched_ = true;
        ROS_ERROR(
            "Formal serial safety abort latched: agv%zu raw wheel command "
            "[%.6f, %.6f], emergency_limit=%.6f m/s, duration=%.3f s, "
            "reason=%s",
            index + 1U, left, right, emergency_abort_limit_,
            emergency_violation_duration_[index],
            assessment.reason.c_str());
        return false;
      }
      emergency_violation_duration_[index] = 0.0;
      if (assessment.available_limit_exceeded) {
        ROS_WARN_THROTTLE(
            1.0, "agv%zu raw wheel demand [%.6f, %.6f] exceeds current "
            "available limits [%.6f, %.6f] m/s; the formal execution "
            "adapter will apply common fleet scaling",
            index + 1U, left, right,
            capability_[index].max_wheel_linear_velocity_left,
            capability_[index].max_wheel_linear_velocity_right);
      }
    }
    return true;
  }

  double applySerialFleetWheelLimit(
      std::array<PlanarTrackingResult, 3>* tracking, double dt) {
    if (transport_type_ != "serial") return 1.0;

    double required_scale = 1.0;
    double raw_peak = 0.0;
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const auto& value = (*tracking)[index];
      const double left =
          std::abs(value.wheel_linear_velocity_left_raw);
      const double right =
          std::abs(value.wheel_linear_velocity_right_raw);
      const double available_left =
          capability_[index].max_wheel_linear_velocity_left;
      const double available_right =
          capability_[index].max_wheel_linear_velocity_right;
      raw_peak = std::max(raw_peak, std::max(left, right));
      if (left > available_left) {
        required_scale = std::min(
            required_scale, 0.999 * available_left / left);
      }
      if (right > available_right) {
        required_scale = std::min(
            required_scale, 0.999 * available_right / right);
      }
    }

    // Tighten immediately so no transmitted wheel demand can exceed the
    // reported capability. Recover slowly so a one-frame camera/R1 peak does
    // not make the whole fleet jump back to full speed on the next tick.
    if (required_scale < fleet_wheel_scale_) {
      fleet_wheel_scale_ = required_scale;
    } else {
      fleet_wheel_scale_ = std::min(
          required_scale,
          fleet_wheel_scale_ + fleet_scale_recovery_rate_per_second_ * dt);
    }
    fleet_wheel_scale_ = std::clamp(fleet_wheel_scale_, 0.0, 1.0);
    if (fleet_wheel_scale_ >= 1.0) return 1.0;

    for (auto& value : *tracking) {
      value.linear_velocity_feedforward *= fleet_wheel_scale_;
      value.angular_velocity_feedforward *= fleet_wheel_scale_;
      value.linear_velocity_raw *= fleet_wheel_scale_;
      value.angular_velocity_raw *= fleet_wheel_scale_;
      value.wheel_linear_velocity_left_raw *= fleet_wheel_scale_;
      value.wheel_linear_velocity_right_raw *= fleet_wheel_scale_;
    }
    ROS_WARN_THROTTLE(
        1.0,
        "Formal common fleet wheel limiting active: raw_peak=%.6f m/s "
        "required_scale=%.6f applied_scale=%.6f; recovery is rate-limited",
        raw_peak, required_scale, fleet_wheel_scale_);
    return fleet_wheel_scale_;
  }

  void latchNonfiniteSerialWheelDemand(
      std::size_t index, const PlanarTrackingResult& tracking) {
    if (transport_type_ != "serial" ||
        (std::isfinite(tracking.wheel_linear_velocity_left_raw) &&
         std::isfinite(tracking.wheel_linear_velocity_right_raw))) {
      return;
    }
    safety_abort_latched_ = true;
    ROS_ERROR(
        "Formal serial safety abort latched: agv%zu raw wheel demand is "
        "NaN/Inf", index + 1U);
  }

  void updateExecutionReference(double dt) {
    if (transport_type_ != "serial" || startup_ramp_seconds_ <= 0.0) {
      startup_scale_ = 1.0;
      current_velocity_reference_ = unramped_velocity_reference_;
      current_acceleration_reference_ = unramped_acceleration_reference_;
      return;
    }
    startup_elapsed_seconds_ = std::min(
        startup_ramp_seconds_, startup_elapsed_seconds_ + dt);
    const double tau =
        startup_elapsed_seconds_ / startup_ramp_seconds_;
    // Quintic smoothstep: both scale rate and its derivative are zero at
    // rest and at the end of the ramp. R1 therefore sees no acceleration
    // discontinuity when normal 0.05 m/s execution begins.
    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    startup_scale_ = tau3 * (10.0 + tau * (-15.0 + 6.0 * tau));
    const double scale_rate =
        30.0 * tau2 * (1.0 - tau) * (1.0 - tau) /
        startup_ramp_seconds_;
    current_velocity_reference_ =
        unramped_velocity_reference_ * startup_scale_;
    current_acceleration_reference_ =
        unramped_acceleration_reference_ * startup_scale_ +
        unramped_velocity_reference_ * scale_rate;
  }

  bool updateExecutionInitialization(double dt) {
    if (transport_type_ != "serial") {
      execution_initialized_ = true;
      return true;
    }
    if (execution_initialized_) return true;

    bool stationary = true;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      stationary = stationary &&
          std::abs(feedback_[index].wheel_linear_velocity_left_actual) <=
              initialization_max_wheel_speed_ &&
          std::abs(feedback_[index].wheel_linear_velocity_right_actual) <=
              initialization_max_wheel_speed_;
    }
    if (!stationary) {
      initialization_elapsed_seconds_ = 0.0;
      initialization_sample_count_ = 0U;
      initialization_progress_sum_.fill(0.0);
      ROS_WARN_THROTTLE(
          1.0, "Formal execution initialization is waiting for all six "
          "wheels to remain stationary");
      return false;
    }

    initialization_elapsed_seconds_ += dt;
    ++initialization_sample_count_;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      initialization_progress_sum_[index] += state_.s_actual[index];
    }
    if (initialization_elapsed_seconds_ < initialization_hold_seconds_) {
      return false;
    }

    for (std::size_t index = 0; index < kRobotCount; ++index) {
      initial_progress_offset_[index] =
          initialization_progress_sum_[index] /
          static_cast<double>(initialization_sample_count_);
    }
    execution_initialized_ = true;
    startup_elapsed_seconds_ = 0.0;
    startup_scale_ = 0.0;
    fleet_wheel_scale_ = 1.0;
    execution_channel_velocity_command_.fill(0.0);
    ROS_INFO(
        "Formal execution initialized from %zu stationary samples: "
        "path offsets=[%.6f, %.6f, %.6f] m",
        initialization_sample_count_, initial_progress_offset_[0],
        initial_progress_offset_[1], initial_progress_offset_[2]);
    return true;
  }

  double algorithmPositionActual(std::size_t index) const {
    return state_.s_actual[index] -
        (execution_initialized_ ? initial_progress_offset_[index] : 0.0);
  }

  double algorithmVelocityActual(std::size_t index) const {
    // Suppress stationary camera-difference noise at launch, then introduce
    // measured velocity continuously with the same quintic execution ramp.
    return state_.s_dot_actual[index] *
        (transport_type_ == "serial" ? startup_scale_ : 1.0);
  }

  void applySerialAccelerationExecutionAdapter(
      std::size_t index, const PathCapability& capability, double dt,
      LowerChannelOutput* output) {
    if (transport_type_ != "serial") return;

    // R1 produces a bounded channel acceleration.  The former serial
    // adapter rebuilt a velocity command from the differentiated camera
    // velocity on every tick (v_measured + dt*u), so camera differentiation
    // noise was copied almost one-for-one into the chassis demand.  Keep the
    // measured velocity as R1 feedback, but integrate the R1 acceleration on
    // a persistent execution-command state before passing it to the planar
    // tracker.  This changes only the acceleration-to-velocity actuator
    // adapter; the M1/R1 equations and their recorded states are unchanged.
    const double lower_bound =
        std::max(velocity_lower_bound_, capability.lower_velocity);
    const double upper_bound =
        std::min(velocity_upper_bound_, capability.upper_velocity);
    if (!std::isfinite(lower_bound) || !std::isfinite(upper_bound) ||
        lower_bound > upper_bound || !std::isfinite(output->input_limited)) {
      output->valid = false;
      return;
    }
    const double unprojected =
        execution_channel_velocity_command_[index] +
        dt * output->input_limited;
    execution_channel_velocity_command_[index] =
        std::clamp(unprojected, lower_bound, upper_bound);
    output->channel_velocity_command =
        execution_channel_velocity_command_[index];
    output->velocity_projection_active =
        output->velocity_projection_active ||
        output->channel_velocity_command != unprojected;
    output->valid = output->valid &&
        std::isfinite(output->channel_velocity_command);
  }

  void blendStartupTrackingFeedback(
      std::size_t index, PlanarTrackingResult* value) const {
    if (transport_type_ != "serial" || startup_scale_ >= 1.0) return;
    value->linear_velocity_raw = value->linear_velocity_feedforward +
        startup_scale_ *
            (value->linear_velocity_raw - value->linear_velocity_feedforward);
    value->angular_velocity_raw = value->angular_velocity_feedforward +
        startup_scale_ *
            (value->angular_velocity_raw - value->angular_velocity_feedforward);
    const double half_track =
        0.5 * tracker_config_.wheel_separation[index];
    value->wheel_linear_velocity_left_raw =
        value->linear_velocity_raw - half_track * value->angular_velocity_raw;
    value->wheel_linear_velocity_right_raw =
        value->linear_velocity_raw + half_track * value->angular_velocity_raw;
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
    execution_channel_velocity_command_.fill(0.0);
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

  void receiveRecorderArmed(const std_msgs::Bool::ConstPtr& message) {
    recorder_armed_ = message->data;
    recorder_armed_receive_time_ = ros::Time::now();
    has_recorder_armed_ = true;
  }

  void receiveRecorderMethod(const std_msgs::String::ConstPtr& message) {
    recorder_method_id_ = message->data;
  }

  void receiveWatchdog(const std_msgs::Bool::ConstPtr& message) {
    watchdog_ok_ = message->data;
    watchdog_receive_time_ = ros::Time::now();
    has_watchdog_ = true;
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
          algorithmPositionActual(i), algorithmVelocityActual(i),
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
      controller.path_progress_actual[index] =
          algorithmPositionActual(index);
      // Preserve the measured velocity in the public record. The formal
      // debug stream separately records the ramp-conditioned R1 input.
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

  void publishM2bDebug(const M2bOutput& value) {
    std_msgs::Float64MultiArray message;
    constexpr std::size_t fields_per_robot = 20U;
    constexpr std::size_t header_fields = 6U;
    message.layout.dim.resize(1);
    message.layout.dim[0].label =
        "m2b_algorithm_state_v1:header6+3x20";
    message.layout.dim[0].size =
        header_fields + kRobotCount * fields_per_robot;
    message.layout.dim[0].stride = message.layout.dim[0].size;
    message.data.reserve(message.layout.dim[0].size);
    message.data.push_back(value.valid ? 1.0 : 0.0);
    message.data.push_back(-0.15);
    message.data.push_back(0.52);
    message.data.push_back(value.load_progress_reference);
    message.data.push_back(value.load_velocity_reference);
    message.data.push_back(ros::Time::now().toSec());
    for (std::size_t i = 0; i < kRobotCount; ++i) {
      const std::array<double, fields_per_robot> fields{{
          value.current.position[i], value.current.velocity[i],
          value.current.auxiliary[i], value.reference_acceleration[i],
          value.position_disagreement[i], value.velocity_disagreement[i],
          value.auxiliary_disagreement[i], algorithmPositionActual(i),
          algorithmVelocityActual(i), value.position_error[i],
          value.velocity_error[i], value.transformed_error[i],
          value.transformation_gain[i], value.inverse_gain_term[i],
          value.composite_error[i], value.input_raw[i],
          value.input_limited[i], value.reported_capability[i],
          value.delta_inverse[i], value.mapping_zeta[i]}};
      message.data.insert(message.data.end(), fields.begin(), fields.end());
    }
    m2b_debug_publisher_.publish(message);
  }

  void publishExecutionLimiter(
      const ros::Time& stamp, double scale,
      const std::array<std::array<double, 2>, 3>& demand,
      const std::array<PlanarTrackingResult, 3>& command) {
    std_msgs::Float64MultiArray message;
    constexpr std::size_t header_fields = 4U;
    constexpr std::size_t fields_per_robot = 7U;
    message.layout.dim.resize(1);
    message.layout.dim[0].label =
        "formal_execution_limiter_v1:header4+3x7";
    message.layout.dim[0].size =
        header_fields + kRobotCount * fields_per_robot;
    message.layout.dim[0].stride = message.layout.dim[0].size;
    message.data.reserve(message.layout.dim[0].size);
    double raw_peak = 0.0;
    for (const auto& wheel : demand) {
      raw_peak = std::max(
          raw_peak, std::max(std::abs(wheel[0]), std::abs(wheel[1])));
    }
    message.data.push_back(1.0);
    message.data.push_back(stamp.toSec());
    message.data.push_back(scale);
    message.data.push_back(raw_peak);
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      const std::array<double, fields_per_robot> fields{{
          demand[index][0], demand[index][1],
          command[index].wheel_linear_velocity_left_raw,
          command[index].wheel_linear_velocity_right_raw,
          capability_[index].max_wheel_linear_velocity_left,
          capability_[index].max_wheel_linear_velocity_right,
          initial_progress_offset_[index]}};
      message.data.insert(message.data.end(), fields.begin(), fields.end());
    }
    execution_limiter_publisher_.publish(message);
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
    if (!refreshChassisBinding()) {
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    if (!commandSequencesReady()) {
      if (safety_abort_latched_) {
        ROS_WARN_THROTTLE(
            1.0, "Formal algorithm held fail-zero: serial safety abort "
            "remains latched");
      } else {
        ROS_WARN_THROTTLE(
            1.0, "Formal algorithm held fail-zero: waiting for three "
            "serial chassis command-sequence handoffs");
      }
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    const std::string expected_method_id =
        std::string(upperModeName(upper_mode_)) + "_" +
        lowerModeName(lower_mode_);
    const bool recorder_gate_ok =
        has_recorder_armed_ && recorder_armed_ &&
        (now - recorder_armed_receive_time_).toSec() <=
            maximum_recorder_armed_age_ &&
        recorder_method_id_ == expected_method_id;
    if (require_recorder_armed_ && !recorder_gate_ok) {
      ROS_WARN_THROTTLE(
          1.0, "Formal algorithm held at zero until a fresh experiment "
          "recorder heartbeat confirms method_id=%s",
          expected_method_id.c_str());
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    const std::string input_failure = inputFailureReason(now);
    if (!input_failure.empty()) {
      ROS_WARN_THROTTLE(
          1.0, "Formal algorithm held fail-zero: %s",
          input_failure.c_str());
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    if (!updateExecutionInitialization(dt)) {
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    if (terminal_stop_latched_) {
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
          "Formal algorithm fail-zero: capability mapping failed "
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

    if (m2b_selected_) {
      M2bInput input;
      input.leader_position = leader_position_;
      input.leader_velocity = leader_velocity_;
      input.leader_acceleration = leader_acceleration_;
      input.dt_seconds = dt;
      for (std::size_t i = 0; i < kRobotCount; ++i) {
        input.position_actual[i] = algorithmPositionActual(i);
        input.velocity_actual[i] = algorithmVelocityActual(i);
        input.reported_capability[i] =
            fleet.robots[i].actuator_upper_velocity;
      }
      const M2bOutput m2b = m2b_controller_->step(input);
      if (!m2b.valid) {
        ROS_WARN_THROTTLE(
            1.0, "Formal fake M2b fail-zero: generator or nonlinear "
            "mapping reached its frozen guard");
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishM2bDebug(m2b);
        return;
      }
      current_upper_ = {};
      current_upper_.valid = true;
      current_upper_.updated = true;
      current_upper_.common_lower = -0.15;
      current_upper_.common_upper = 0.52;
      current_upper_.common_velocity = m2b.load_velocity_reference;
      reference_progress_ = std::clamp(
          m2b.load_progress_reference, 0.0, target_progress_);
      current_velocity_reference_ =
          reference_progress_ >= target_progress_
              ? 0.0 : m2b.load_velocity_reference;
      current_acceleration_reference_ =
          reference_progress_ >= target_progress_
              ? 0.0 : m2b.load_acceleration_reference;
      leader_position_ += dt * leader_velocity_;

      for (std::size_t i = 0; i < kRobotCount; ++i) {
        lower[i].position_error = m2b.position_error[i];
        lower[i].velocity_error = m2b.velocity_error[i];
        lower[i].transformed_error = m2b.transformed_error[i];
        lower[i].transformation_gain = m2b.transformation_gain[i];
        lower[i].inverse_gain_term = m2b.inverse_gain_term[i];
        lower[i].composite_error = m2b.composite_error[i];
        lower[i].input_raw = m2b.input_raw[i];
        lower[i].input_limited = m2b.input_limited[i];
        lower[i].input_plant = m2b.input_limited[i];
        lower[i].input_limit_active = m2b.input_limit_active[i];
        lower[i].channel_velocity_command = std::clamp(
            algorithmVelocityActual(i) + dt * m2b.input_limited[i],
            -0.15, 0.52);
        lower[i].valid = true;

        PlanarTrackingInput tracking_input;
        tracking_input.support_index = i;
        tracking_input.load_progress_reference = reference_progress_;
        tracking_input.channel_velocity_command =
            lower[i].channel_velocity_command;
        tracking_input.robot_pose_actual = poseValue(state_.robot_pose[i]);
        tracking_input.support_pose_actual =
            poseValue(state_.support_pose[i]);
        tracking[i] = tracker_.track(tracking_input);
        if (!tracking[i].valid) {
          latchNonfiniteSerialWheelDemand(i, tracking[i]);
          publishZero(now);
          publishPublicState(now, false, lower, tracking);
          publishM2bDebug(m2b);
          return;
        }
      }
      std::array<PlanarPose, 3> robot_pose;
      std::array<PlanarPose, 3> support_pose;
      std::array<double, 3> channel_velocity;
      for (std::size_t i = 0; i < kRobotCount; ++i) {
        robot_pose[i] = poseValue(state_.robot_pose[i]);
        support_pose[i] = poseValue(state_.support_pose[i]);
        channel_velocity[i] = lower[i].channel_velocity_command;
      }
      execution_adapter_.adapt(
          reference_progress_, channel_velocity, robot_pose, support_pose,
          &tracking);
      std::array<std::array<double, 2>, 3> wheel_demand_before_limit;
      for (std::size_t i = 0; i < kRobotCount; ++i) {
        wheel_demand_before_limit[i] = {{
            tracking[i].wheel_linear_velocity_left_raw,
            tracking[i].wheel_linear_velocity_right_raw}};
      }
      if (!trackingPassesSerialEmergencyGate(tracking, dt)) {
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishM2bDebug(m2b);
        return;
      }
      const double wheel_scale = applySerialFleetWheelLimit(&tracking, dt);
      publishExecutionLimiter(
          now, wheel_scale, wheel_demand_before_limit, tracking);
      for (std::size_t i = 0; i < kRobotCount; ++i) {
        agv_msgs::ChassisCommand command;
        command.header.stamp = now;
        command.robot_id = static_cast<std::uint8_t>(i + 1U);
        command.command_seq = ++command_sequence_[i];
        command.control_mode = 1U;
        command.linear_velocity_reference = tracking[i].linear_velocity_raw;
        command.angular_velocity_reference =
            tracking[i].angular_velocity_raw;
        command.wheel_linear_velocity_left_raw =
            tracking[i].wheel_linear_velocity_left_raw;
        command.wheel_linear_velocity_right_raw =
            tracking[i].wheel_linear_velocity_right_raw;
        command.experiment_id = experiment_id_;
        command.method_id = "M2b_M2b";
        command_publishers_[i].publish(command);
      }
      publishPublicState(now, true, lower, tracking);
      publishDebug(true, current_upper_, distributed, lower);
      publishM2bDebug(m2b);
      return;
    }

    std::array<UpperAgentInput, 3> upper_input;
    const double failure_margin =
        !require_software_watchdog_
            ? 1.0
            : (has_watchdog_ && watchdog_ok_ &&
               (now - watchdog_receive_time_).toSec() <=
                   maximum_watchdog_age_ ? 1.0 : 0.0);
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
          position_margin, failure_margin}};
    }
    current_upper_ = upper_generator_->step(upper_input, dt);
    if (!current_upper_.valid) {
      ROS_WARN_THROTTLE(
          1.0, "Formal algorithm fail-zero: upper reference is invalid");
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
            1.0, "Formal algorithm fail-zero: distributed reference "
            "is invalid");
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishDebug(false, current_upper_, distributed, lower);
        return;
      }
      current_distributed_ = distributed;
      distributed_state_ = distributed.next;
      unramped_velocity_reference_ =
          upper_mode_ == UpperMode::kM4
              ? current_upper_.common_velocity
              : distributed.common_velocity;
      unramped_acceleration_reference_ =
          upper_mode_ == UpperMode::kM4
              ? 0.0
              : (distributed.acceleration[0] +
                 distributed.acceleration[1] +
                 distributed.acceleration[2]) /
                    static_cast<double>(kRobotCount);
    }
    updateExecutionReference(dt);
    leader_position_ += dt * leader_velocity_;
    reference_progress_ = std::min(
        target_progress_,
        reference_progress_ + dt * current_velocity_reference_);
    if (reference_progress_ >= target_progress_) {
      unramped_velocity_reference_ = 0.0;
      unramped_acceleration_reference_ = 0.0;
      current_velocity_reference_ = 0.0;
      current_acceleration_reference_ = 0.0;
      terminal_stop_latched_ = true;
      ROS_INFO(
          "Formal execution reached %.3f m; synchronized terminal stop "
          "latched without endpoint pose hunting",
          target_progress_);
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }

    std::array<PlanarPose, 3> robot_pose;
    std::array<PlanarPose, 3> support_pose;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      LowerChannelInput input;
      input.position_actual = algorithmPositionActual(index);
      input.velocity_actual = algorithmVelocityActual(index);
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
      applySerialAccelerationExecutionAdapter(
          index, fleet.robots[index], dt, &lower[index]);
      if (!lower[index].valid) {
        ROS_WARN_THROTTLE(
            1.0, "Formal algorithm fail-zero: lower controller for "
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
        latchNonfiniteSerialWheelDemand(index, tracking[index]);
        ROS_WARN_THROTTLE(
            1.0, "Formal algorithm fail-zero: planar tracker for "
            "agv%zu is invalid", index + 1U);
        publishZero(now);
        publishPublicState(now, false, lower, tracking);
        publishDebug(false, current_upper_, distributed, lower);
        return;
      }
    }

    std::array<double, 3> channel_velocity;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      channel_velocity[index] = lower[index].channel_velocity_command;
    }
    execution_adapter_.adapt(
        reference_progress_, channel_velocity, robot_pose, support_pose,
        &tracking);
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      blendStartupTrackingFeedback(index, &tracking[index]);
    }

    std::array<std::array<double, 2>, 3> wheel_demand_before_limit;
    for (std::size_t index = 0; index < kRobotCount; ++index) {
      wheel_demand_before_limit[index] = {{
          tracking[index].wheel_linear_velocity_left_raw,
          tracking[index].wheel_linear_velocity_right_raw}};
    }
    if (!trackingPassesSerialEmergencyGate(tracking, dt)) {
      publishZero(now);
      publishPublicState(now, false, lower, tracking);
      publishDebug(false, current_upper_, distributed, lower);
      return;
    }
    const double wheel_scale = applySerialFleetWheelLimit(&tracking, dt);
    publishExecutionLimiter(
        now, wheel_scale, wheel_demand_before_limit, tracking);

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
      // M1's causal wheel-risk margin uses the demand before the execution
      // adapter, not the already-limited command sent to the chassis.
      previous_wheel_raw_[index] = wheel_demand_before_limit[index];
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
  FleetPlanarExecutionAdapter execution_adapter_;
  CapabilityMapper capability_mapper_;
  std::unique_ptr<UpperReferenceGenerator> upper_generator_;
  std::unique_ptr<M2bController> m2b_controller_;
  std::array<std::unique_ptr<LowerChannelController>, 3> lower_controllers_;
  DistributedReferenceConfig distributed_config_;
  DistributedReferenceState distributed_state_;
  DistributedReferenceOutput current_distributed_;
  UpperReferenceOutput current_upper_;
  UpperMode upper_mode_{UpperMode::kM1};
  LowerMode lower_mode_{LowerMode::kR1};

  ros::Subscriber state_subscriber_;
  ros::Subscriber recorder_armed_subscriber_;
  ros::Subscriber recorder_method_subscriber_;
  ros::Subscriber watchdog_subscriber_;
  std::array<ros::Subscriber, 3> capability_subscribers_;
  std::array<ros::Subscriber, 3> feedback_subscribers_;
  std::array<ros::Publisher, 3> command_publishers_;
  ros::Publisher reference_publisher_;
  ros::Publisher controller_publisher_;
  ros::Publisher debug_publisher_;
  ros::Publisher execution_limiter_publisher_;
  ros::Publisher m2b_debug_publisher_;
  ros::Timer timer_;
  agv_msgs::CooperativeState state_;
  std::array<agv_msgs::CapabilityReport, 3> capability_;
  std::array<agv_msgs::ChassisFeedback, 3> feedback_;
  ros::Time state_receive_time_;
  std::array<ros::Time, 3> capability_receive_time_;
  std::array<ros::Time, 3> feedback_receive_time_;
  bool has_state_{false};
  std::array<bool, 3> has_capability_{{false, false, false}};
  std::array<bool, 3> has_feedback_{{false, false, false}};
  std::array<bool, 3> command_sequence_synchronised_{{false, false, false}};
  bool require_valid_load_state_{true};
  bool require_recorder_armed_{false};
  bool has_recorder_armed_{false};
  bool recorder_armed_{false};
  bool require_software_watchdog_{false};
  bool has_watchdog_{false};
  bool watchdog_ok_{false};
  bool command_publication_authorized_{false};
  bool serial_execution_authorized_{false};
  bool confirm_test_area_clear_{false};
  bool confirm_wheels_on_floor_{false};
  bool confirm_unloaded_fixture_{false};
  bool command_outputs_ready_{false};
  bool m2b_selected_{false};
  bool safety_abort_latched_{false};
  bool execution_initialized_{false};
  bool terminal_stop_latched_{false};
  std::size_t upper_ticks_per_update_{4U};

  std::array<std::uint32_t, 3> command_sequence_{{0U, 0U, 0U}};
  std::array<DynamicBoundaryState, 3> previous_boundary_;
  std::array<double, 3> previous_input_raw_{{0.0, 0.0, 0.0}};
  std::array<std::array<double, 2>, 3> previous_wheel_raw_{{
      {{0.0, 0.0}}, {{0.0, 0.0}}, {{0.0, 0.0}}}};
  std::array<double, 3> emergency_violation_duration_{{0.0, 0.0, 0.0}};
  std::array<double, 3> initialization_progress_sum_{{0.0, 0.0, 0.0}};
  std::array<double, 3> initial_progress_offset_{{0.0, 0.0, 0.0}};
  std::array<double, 3> execution_channel_velocity_command_{{
      0.0, 0.0, 0.0}};
  std::size_t initialization_sample_count_{0U};
  std::array<double, 3> previous_position_error_{{0.0, 0.0, 0.0}};
  std::array<double, 2> lower_initial_parameter_{{0.0, 0.0}};
  double lower_initial_disturbance_{0.08};

  double publish_rate_{100.0};
  double maximum_state_age_{0.15};
  double maximum_capability_age_{0.20};
  double maximum_feedback_age_{0.25};
  double minimum_battery_voltage_{10.0};
  double emergency_abort_limit_{0.12};
  double startup_ramp_seconds_{1.0};
  double emergency_abort_persistence_seconds_{0.10};
  double initialization_hold_seconds_{0.30};
  double initialization_max_wheel_speed_{0.015};
  double fleet_scale_recovery_rate_per_second_{0.50};
  double initialization_elapsed_seconds_{0.0};
  double startup_scale_{0.0};
  double fleet_wheel_scale_{1.0};
  double startup_elapsed_seconds_{0.0};
  double reference_progress_{0.0};
  double target_progress_{1.0};
  double leader_position_{0.0};
  double leader_velocity_{0.08};
  double leader_acceleration_{0.0};
  double current_velocity_reference_{0.08};
  double current_acceleration_reference_{0.0};
  double unramped_velocity_reference_{0.08};
  double unramped_acceleration_reference_{0.0};
  double velocity_lower_bound_{-0.15};
  double velocity_upper_bound_{0.58};
  double capability_reserve_{0.015};
  double capability_safe_margin_{0.12};
  double input_safe_margin_{0.40};
  double wheel_safe_margin_{0.08};
  double position_limit_{0.12};
  double position_safe_margin_{0.06};
  double maximum_watchdog_age_{0.25};
  double maximum_recorder_armed_age_{0.50};
  ros::Time recorder_armed_receive_time_;
  std::string recorder_method_id_;
  ros::Time watchdog_receive_time_;
  std::string experiment_id_;
  std::string transport_type_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "formal_algorithm");
  try {
    multi_agv_control::FormalFakeAlgorithmNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start formal algorithm: %s", error.what());
    return 1;
  }
  return 0;
}
