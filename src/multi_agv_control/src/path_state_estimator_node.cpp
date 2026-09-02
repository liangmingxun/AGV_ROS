#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <XmlRpcValue.h>
#include <agv_msgs/CooperativeState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "multi_agv_control/path_projector.hpp"
#include "multi_agv_control/s_curve_path.hpp"
#include "multi_agv_control/state_estimator.hpp"
#include "multi_agv_control/support_geometry.hpp"

namespace multi_agv_control {
namespace {

constexpr std::size_t kRobotCount = 3U;

double quaternionYaw(const geometry_msgs::Quaternion& q) {
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

geometry_msgs::Pose2D toMessage(const PlanarPose& pose) {
  geometry_msgs::Pose2D message;
  message.x = pose.position.x();
  message.y = pose.position.y();
  message.theta = pose.yaw;
  return message;
}

PlanarPose inversePose(const PlanarPose& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {{-c * pose.position.x() - s * pose.position.y(),
           s * pose.position.x() - c * pose.position.y()},
          -pose.yaw};
}

double xmlNumber(const XmlRpc::XmlRpcValue& value, const char* key) {
  if (!value.hasMember(key)) {
    throw std::runtime_error(std::string("missing parameter field: ") + key);
  }
  const auto& member = value[key];
  if (member.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(member);
  }
  if (member.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(member);
  }
  throw std::runtime_error(std::string("parameter field is not numeric: ") + key);
}

std::string xmlString(const XmlRpc::XmlRpcValue& value, const char* key) {
  if (!value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error(std::string("missing string parameter field: ") + key);
  }
  return static_cast<std::string>(value[key]);
}

}  // namespace

class PathStateEstimatorNode {
 public:
  PathStateEstimatorNode() : private_node_("~"), path_(loadPathConfig()),
      geometry_(path_, loadGeometryConfig()) {
    private_node_.param("localization_mode", localization_mode_,
                        std::string("odometry_pretest"));
    if (localization_mode_ != "odometry_pretest" &&
        localization_mode_ != "camera" &&
        localization_mode_ != "fused") {
      throw std::runtime_error(
          "localization_mode must be odometry_pretest, camera or fused");
    }
    camera_mode_ = localization_mode_ != "odometry_pretest";
    robot_pose_source_ = localization_mode_ == "fused"
        ? agv_msgs::CooperativeState::SOURCE_FUSED
        : agv_msgs::CooperativeState::SOURCE_CAMERA;
    private_node_.param("require_calibration_epoch",
                        require_calibration_epoch_, true);
    private_node_.param("publish_rate", publish_rate_, 100.0);
    private_node_.param("maximum_state_age", maximum_state_age_, 0.15);
    private_node_.param("maximum_sync_slop", maximum_sync_slop_, 0.02);
    private_node_.param("maximum_synchronized_snapshot_hold",
                        maximum_synchronized_snapshot_hold_, 0.05);
    int synchronization_queue_size =
        static_cast<int>(synchronization_queue_size_);
    private_node_.param("synchronization_queue_size",
                        synchronization_queue_size,
                        synchronization_queue_size);
    private_node_.param("maximum_rigid_fit_residual",
                        maximum_rigid_fit_residual_, 0.05);
    private_node_.param("maximum_load_path_transient_hold",
                        maximum_load_path_transient_hold_, 0.05);
    private_node_.param("maximum_robot_path_transient_hold",
                        maximum_robot_path_transient_hold_, 0.05);
    private_node_.param("derive_virtual_load_from_robots",
                        derive_virtual_load_from_robots_, false);
    private_node_.param("auto_anchor_from_robot_poses",
                        auto_anchor_from_robot_poses_, false);
    private_node_.param("output_frame", output_frame_,
                        std::string("world"));
    if (!(publish_rate_ > 0.0) || !(maximum_state_age_ > 0.0) ||
        !(maximum_sync_slop_ >= 0.0) ||
        maximum_synchronized_snapshot_hold_ < 0.0 ||
        maximum_synchronized_snapshot_hold_ > maximum_state_age_ ||
        synchronization_queue_size < 3 ||
        !(maximum_rigid_fit_residual_ > 0.0) ||
        maximum_load_path_transient_hold_ < 0.0 ||
        maximum_load_path_transient_hold_ > 0.10 ||
        maximum_robot_path_transient_hold_ < 0.0 ||
        maximum_robot_path_transient_hold_ > 0.10) {
      throw std::runtime_error("invalid localization timing or residual configuration");
    }
    synchronization_queue_size_ =
        static_cast<std::size_t>(synchronization_queue_size);

    const auto projector_config = loadProjectorConfig();
    const auto estimator_config = loadEstimatorConfig();
    XmlRpc::XmlRpcValue robots;
    if (!private_node_.getParam("robots", robots) ||
        robots.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        robots.size() != static_cast<int>(kRobotCount)) {
      throw std::runtime_error("robots must contain exactly three entries");
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      loadRobot(index, robots[static_cast<int>(index)], projector_config,
                estimator_config);
    }

    PathProjector load_projector(
        path_.length(),
        [this](double s) {
          const auto sample = path_.sample(s);
          return ProjectionCurveSample{sample.position, sample.first_derivative,
                                       sample.second_derivative};
        },
        projector_config);
    load_estimator_ =
        std::make_unique<StateEstimator>(std::move(load_projector), estimator_config);
    if (camera_mode_ && !derive_virtual_load_from_robots_) {
      private_node_.param("load_pose_topic", load_pose_topic_, std::string());
      if (load_pose_topic_.empty()) {
        throw std::runtime_error(
            "camera localization requires load_pose_topic");
      }
      load_pose_subscriber_ = node_.subscribe<geometry_msgs::PoseStamped>(
          load_pose_topic_, 10,
          &PathStateEstimatorNode::receiveCameraLoad, this,
          ros::TransportHints().reliable().tcpNoDelay());
    }
    if ((derive_virtual_load_from_robots_ ||
         auto_anchor_from_robot_poses_) && !camera_mode_) {
      throw std::runtime_error(
          "virtual-load derivation and automatic anchoring require camera/fused mode");
    }
    if (auto_anchor_from_robot_poses_ &&
        !derive_virtual_load_from_robots_) {
      throw std::runtime_error(
          "automatic anchoring requires virtual load derived from the three robots");
    }

    state_publisher_ = node_.advertise<agv_msgs::CooperativeState>(
        "/multi_agv/cooperative_state", 5, false);
    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate_),
                              &PathStateEstimatorNode::publish, this);
  }

 private:
  struct OdometrySample {
    ros::Time stamp;
    PlanarPose robot_pose;
    PlanarPose support_pose;
    StateEstimate path_state;
  };

  struct RobotState {
    std::string robot_id;
    std::string odom_topic;
    std::string pose_topic;
    std::string odom_frame;
    std::string base_frame;
    PlanarPose world_to_odom;
    PlanarPose base_to_support;
    std::unique_ptr<StateEstimator> estimator;
    StateEstimate last_valid_path_state;
    ros::Time last_valid_path_stamp;
    ros::Subscriber subscriber;
    std::deque<OdometrySample> samples;
  };

  SCurveConfig loadPathConfig() {
    SCurveConfig config;
    private_node_.param("path_s_curve/model", config.model,
                        std::string("sine_single_period"));
    private_node_.param("path_s_curve/amplitude", config.amplitude, 0.05);
    private_node_.param("path_s_curve/longitudinal_length",
                        config.longitudinal_length, 1.0);
    private_node_.param("path_s_curve/circle_radius", config.circle_radius,
                        1.0);
    private_node_.param("path_s_curve/entry_straight_length",
                        config.entry_straight_length, 0.0);
    private_node_.param("path_s_curve/curvature_ramp_length",
                        config.curvature_ramp_length, 0.0);
    private_node_.param("path_s_curve/circle_direction",
                        config.circle_direction, 1.0);
    int samples = 20001;
    private_node_.param("path_s_curve/lookup_samples", samples, 20001);
    if (samples < 3) {
      throw std::runtime_error("path lookup_samples must be at least three");
    }
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
      throw std::runtime_error("support_geometry/supports must have three entries");
    }
    for (int index = 0; index < supports.size(); ++index) {
      if (xmlString(supports[index], "robot_id") !=
              "agv" + std::to_string(index + 1) ||
          static_cast<int>(xmlNumber(supports[index], "robot_index")) !=
              index + 1) {
        throw std::runtime_error(
            "support entries must be ordered agv1, agv2, agv3");
      }
      config.offsets.push_back({xmlNumber(supports[index], "q_tangent"),
                                xmlNumber(supports[index], "q_normal")});
    }
    return config;
  }

  PathProjectorConfig loadProjectorConfig() {
    PathProjectorConfig config;
    int coarse_samples = static_cast<int>(config.coarse_samples);
    int iterations = static_cast<int>(config.maximum_refinement_iterations);
    private_node_.param("projector/coarse_samples", coarse_samples, coarse_samples);
    private_node_.param("projector/maximum_refinement_iterations", iterations,
                        iterations);
    private_node_.param("projector/convergence_tolerance",
                        config.convergence_tolerance,
                        config.convergence_tolerance);
    private_node_.param("projector/maximum_projection_distance",
                        config.maximum_projection_distance,
                        config.maximum_projection_distance);
    if (coarse_samples < 0 || iterations < 0) {
      throw std::runtime_error("negative projector integer parameter");
    }
    config.coarse_samples = static_cast<std::size_t>(coarse_samples);
    config.maximum_refinement_iterations = static_cast<std::size_t>(iterations);
    return config;
  }

  StateEstimatorConfig loadEstimatorConfig() {
    StateEstimatorConfig config;
    private_node_.param("estimator/filter_alpha", config.filter_alpha,
                        config.filter_alpha);
    private_node_.param("estimator/projection_half_window",
                        config.projection_half_window,
                        config.projection_half_window);
    private_node_.param("estimator/initial_progress", config.initial_progress,
                        config.initial_progress);
    private_node_.param("estimator/minimum_measurement_interval",
                        config.minimum_measurement_interval,
                        config.minimum_measurement_interval);
    private_node_.param("estimator/maximum_measurement_interval",
                        config.maximum_measurement_interval,
                        config.maximum_measurement_interval);
    private_node_.param("estimator/maximum_absolute_speed",
                        config.maximum_absolute_speed,
                        config.maximum_absolute_speed);
    return config;
  }

  PlanarPose xmlPose(const XmlRpc::XmlRpcValue& value, const char* key) {
    if (!value.hasMember(key) ||
        value[key].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error(std::string("missing pose parameter: ") + key);
    }
    const auto& pose = value[key];
    return {{xmlNumber(pose, "x"), xmlNumber(pose, "y")},
            xmlNumber(pose, "yaw")};
  }

  void loadRobot(std::size_t index, const XmlRpc::XmlRpcValue& value,
                 const PathProjectorConfig& projector_config,
                 const StateEstimatorConfig& estimator_config) {
    auto& robot = robots_[index];
    robot.robot_id = xmlString(value, "robot_id");
    const int configured_index = static_cast<int>(xmlNumber(value, "robot_index"));
    if (configured_index != static_cast<int>(index + 1U) ||
        robot.robot_id != "agv" + std::to_string(index + 1U)) {
      throw std::runtime_error("robot entries must be ordered agv1, agv2, agv3");
    }
    robot.base_frame = xmlString(value, "base_frame");
    robot.base_to_support = xmlPose(value, "base_to_support");
    PathProjector projector(
        path_.length(),
        [this, index](double s) {
          const auto sample = geometry_.sample(index, s);
          return ProjectionCurveSample{sample.position, sample.first_derivative,
                                       sample.second_derivative};
        },
        projector_config);
    robot.estimator =
        std::make_unique<StateEstimator>(std::move(projector), estimator_config);
    if (camera_mode_) {
      robot.pose_topic = xmlString(value, "pose_topic");
      const boost::function<void(
          const geometry_msgs::PoseStamped::ConstPtr&)> callback =
          [this, index](const geometry_msgs::PoseStamped::ConstPtr& message) {
            receiveCameraRobot(index, message);
      };
      ros::TransportHints pose_transport_hints;
      if (localization_mode_ == "fused") {
        // camera_odom_fusion and this estimator run together on Robot1. This
        // local control-state hop must be lossless: UDPROS drops created
        // one-sample invalid CooperativeState bursts even while all remote
        // odometry streams remained at 100 Hz. Wi-Fi behavior is handled on
        // the upstream odometry subscriptions, not by discarding fused poses.
        pose_transport_hints.reliable().tcpNoDelay();
      } else {
        pose_transport_hints.reliable().tcpNoDelay();
      }
      robot.subscriber = node_.subscribe<geometry_msgs::PoseStamped>(
          robot.pose_topic,
          static_cast<std::uint32_t>(synchronization_queue_size_),
          callback, ros::VoidConstPtr(),
          pose_transport_hints);
    } else {
      robot.odom_topic = xmlString(value, "odom_topic");
      robot.odom_frame = xmlString(value, "odom_frame");
      robot.world_to_odom = xmlPose(value, "world_to_odom");
      const boost::function<void(const nav_msgs::Odometry::ConstPtr&)> callback =
          [this, index](const nav_msgs::Odometry::ConstPtr& message) {
            receiveOdometry(index, message);
          };
      robot.subscriber = node_.subscribe<nav_msgs::Odometry>(
          robot.odom_topic, 10, callback, ros::VoidConstPtr(),
          // Cooperative odometry is state, not disposable telemetry. Reliable
          // TCP plus the timestamp-matching history below avoids turning a
          // short Wi-Fi delivery pause into a fabricated three-robot pose.
          ros::TransportHints().reliable().tcpNoDelay());
    }
  }

  bool calibrationToken(const std::string& frame_id,
                        std::uint32_t* token) const {
    if (frame_id == "world" && !require_calibration_epoch_) {
      *token = 0U;
      return true;
    }
    const std::string prefix = "world@";
    if (frame_id.compare(0U, prefix.size(), prefix) != 0 ||
        frame_id.size() != prefix.size() + 8U) {
      return false;
    }
    try {
      std::size_t consumed = 0U;
      const unsigned long parsed =
          std::stoul(frame_id.substr(prefix.size()), &consumed, 16);
      if (consumed != 8U || parsed == 0UL ||
          parsed > std::numeric_limits<std::uint32_t>::max()) {
        return false;
      }
      *token = static_cast<std::uint32_t>(parsed);
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }

  bool validWorldPose(const geometry_msgs::PoseStamped& message,
                      std::uint32_t* epoch_token) const {
    const auto& p = message.pose.position;
    return !message.header.stamp.isZero() &&
        calibrationToken(message.header.frame_id, epoch_token) &&
        std::isfinite(p.x) &&
        std::isfinite(p.y) &&
        std::isfinite(quaternionYaw(message.pose.orientation));
  }

  PlanarPose planarPose(const geometry_msgs::PoseStamped& message) const {
    return {{message.pose.position.x, message.pose.position.y},
            quaternionYaw(message.pose.orientation)};
  }

  bool acceptCameraEpoch(std::uint32_t epoch_token) {
    if (require_calibration_epoch_ && epoch_token == 0U) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected camera pose with zero calibration epoch token");
      return false;
    }
    if (epoch_token == camera_epoch_token_) {
      return true;
    }
    if (retired_camera_epoch_tokens_.count(epoch_token) != 0U) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected pose from retired camera calibration epoch");
      return false;
    }
    if (camera_epoch_token_ != 0U) {
      retired_camera_epoch_tokens_.insert(camera_epoch_token_);
      ROS_ERROR(
          "Camera calibration epoch changed (%u -> %u): cleared all "
          "CooperativeState camera histories; restart any formal experiment",
          camera_epoch_token_, epoch_token);
    } else {
      ROS_INFO("State estimator accepted camera calibration epoch token=%u",
               epoch_token);
    }
    camera_epoch_token_ = epoch_token;
    path_anchor_initialized_ = false;
    path_frame_to_camera_world_ = PlanarPose{};

    // Preserve each local path seed only to allow a non-formal diagnostic run
    // to recover. All timestamps, velocity histories and cross-robot samples
    // are cleared, so measurements from two world frames are never combined.
    for (auto& robot : robots_) {
      const double progress_seed = robot.estimator->lastEstimate().progress;
      robot.samples.clear();
      robot.estimator->reset(progress_seed);
      robot.last_valid_path_state = StateEstimate{};
      robot.last_valid_path_stamp = ros::Time();
    }
    const double load_progress_seed = load_estimator_->lastEstimate().progress;
    load_camera_samples_.clear();
    load_estimator_->reset(load_progress_seed);
    last_load_path_state_ = StateEstimate{};
    last_load_measurement_stamp_ = ros::Time();
    last_valid_load_path_stamp_ = ros::Time();
    has_last_synchronized_samples_ = false;
    return true;
  }

  void receiveCameraRobot(
      std::size_t index,
      const geometry_msgs::PoseStamped::ConstPtr& message) {
    auto& robot = robots_[index];
    std::uint32_t epoch_token = 0U;
    if (!validWorldPose(*message, &epoch_token)) {
      ROS_WARN_THROTTLE(1.0, "Rejected %s camera pose with invalid stamp or frame",
                        robot.robot_id.c_str());
      return;
    }
    if (!acceptCameraEpoch(epoch_token)) {
      return;
    }
    if (!robot.samples.empty() &&
        message->header.stamp <= robot.samples.back().stamp) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected non-increasing %s camera timestamp",
          robot.robot_id.c_str());
      return;
    }
    OdometrySample sample;
    sample.stamp = message->header.stamp;
    sample.robot_pose = planarPose(*message);
    if (path_anchor_initialized_) {
      sample.robot_pose = composePose(
          path_frame_to_camera_world_, sample.robot_pose);
    }
    sample.support_pose =
        composePose(sample.robot_pose, robot.base_to_support);
    if (!auto_anchor_from_robot_poses_ || path_anchor_initialized_) {
      sample.path_state = updateRobotPathState(
          &robot, sample.support_pose.position, sample.stamp);
    }
    robot.samples.push_back(std::move(sample));
    while (robot.samples.size() > synchronization_queue_size_) {
      robot.samples.pop_front();
    }
  }

  void receiveCameraLoad(
      const geometry_msgs::PoseStamped::ConstPtr& message) {
    std::uint32_t epoch_token = 0U;
    if (!validWorldPose(*message, &epoch_token)) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected load camera pose with invalid stamp or frame");
      return;
    }
    if (!acceptCameraEpoch(epoch_token)) {
      return;
    }
    if (!load_camera_samples_.empty() &&
        message->header.stamp <= load_camera_samples_.back().stamp) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected non-increasing load camera timestamp");
      return;
    }
    OdometrySample sample;
    sample.stamp = message->header.stamp;
    sample.robot_pose = planarPose(*message);
    sample.support_pose = sample.robot_pose;
    sample.path_state = load_estimator_->update(
        sample.robot_pose.position, sample.stamp.toSec());
    load_camera_samples_.push_back(std::move(sample));
    while (load_camera_samples_.size() > synchronization_queue_size_) {
      load_camera_samples_.pop_front();
    }
  }

  void receiveOdometry(std::size_t index,
                       const nav_msgs::Odometry::ConstPtr& message) {
    auto& robot = robots_[index];
    if (message->header.stamp.isZero() ||
        message->header.frame_id != robot.odom_frame ||
        message->child_frame_id != robot.base_frame) {
      ROS_WARN_THROTTLE(1.0, "Rejected %s odometry with invalid stamp or frame",
                        robot.robot_id.c_str());
      return;
    }
    const auto& position = message->pose.pose.position;
    const auto& orientation = message->pose.pose.orientation;
    PlanarPose odom_to_base{{position.x, position.y},
                            quaternionYaw(orientation)};
    if (!std::isfinite(odom_to_base.position.x()) ||
        !std::isfinite(odom_to_base.position.y()) ||
        !std::isfinite(odom_to_base.yaw)) {
      return;
    }
    if (!robot.samples.empty() &&
        message->header.stamp <= robot.samples.back().stamp) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected non-increasing %s odometry timestamp",
          robot.robot_id.c_str());
      return;
    }
    OdometrySample sample;
    sample.stamp = message->header.stamp;
    sample.robot_pose = composePose(robot.world_to_odom, odom_to_base);
    sample.support_pose =
        composePose(sample.robot_pose, robot.base_to_support);
    sample.path_state = robot.estimator->update(
        sample.support_pose.position, sample.stamp.toSec());
    robot.samples.push_back(std::move(sample));
    while (robot.samples.size() > synchronization_queue_size_) {
      robot.samples.pop_front();
    }
  }

  bool fresh(const OdometrySample& sample, const ros::Time& now) const {
    const double age = (now - sample.stamp).toSec();
    return age >= -maximum_sync_slop_ && age <= maximum_state_age_;
  }

  const OdometrySample* latestSample(const RobotState& robot) const {
    return robot.samples.empty() ? nullptr : &robot.samples.back();
  }

  const OdometrySample* latestFreshSample(
      const RobotState& robot, const ros::Time& now) const {
    for (auto sample = robot.samples.rbegin();
         sample != robot.samples.rend(); ++sample) {
      if (fresh(*sample, now)) {
        return &*sample;
      }
    }
    return nullptr;
  }

  const OdometrySample* nearestFreshSample(
      const RobotState& robot, const ros::Time& target,
      const ros::Time& now) const {
    const OdometrySample* nearest = nullptr;
    double nearest_distance = 0.0;
    for (const auto& sample : robot.samples) {
      if (!fresh(sample, now)) {
        continue;
      }
      const double distance = std::abs((sample.stamp - target).toSec());
      if (nearest == nullptr || distance < nearest_distance) {
        nearest = &sample;
        nearest_distance = distance;
      }
    }
    return nearest;
  }

  bool selectSynchronizedSamples(
      const ros::Time& now,
      std::array<const OdometrySample*, kRobotCount>* selected) {
    const auto use_last_snapshot = [&]() {
      if (!has_last_synchronized_samples_) {
        return false;
      }
      ros::Time oldest = last_synchronized_samples_[0].stamp;
      for (std::size_t index = 0U; index < kRobotCount; ++index) {
        oldest = std::min(oldest, last_synchronized_samples_[index].stamp);
        if (!fresh(last_synchronized_samples_[index], now)) {
          return false;
        }
      }
      const double hold_age = (now - oldest).toSec();
      if (!std::isfinite(hold_age) || hold_age < 0.0 ||
          hold_age > maximum_synchronized_snapshot_hold_) {
        return false;
      }
      for (std::size_t index = 0U; index < kRobotCount; ++index) {
        (*selected)[index] = &last_synchronized_samples_[index];
      }
      ROS_WARN_THROTTLE(
          1.0,
          "Holding the last synchronized three-robot snapshot for %.6f s while fused streams re-align",
          hold_age);
      return true;
    };

    std::array<const OdometrySample*, kRobotCount> latest{};
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      latest[index] = latestFreshSample(robots_[index], now);
      if (latest[index] == nullptr) {
        return use_last_snapshot();
      }
    }
    const ros::Time target = (*std::min_element(
        latest.begin(), latest.end(),
        [](const OdometrySample* lhs, const OdometrySample* rhs) {
          return lhs->stamp < rhs->stamp;
        }))->stamp;
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      (*selected)[index] =
          nearestFreshSample(robots_[index], target, now);
      if ((*selected)[index] == nullptr) {
        return use_last_snapshot();
      }
    }
    const auto minmax = std::minmax_element(
        selected->begin(), selected->end(),
        [](const OdometrySample* lhs, const OdometrySample* rhs) {
          return lhs->stamp < rhs->stamp;
        });
    if (((*minmax.second)->stamp - (*minmax.first)->stamp).toSec() >
        maximum_sync_slop_) {
      return use_last_snapshot();
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      last_synchronized_samples_[index] = *(*selected)[index];
    }
    has_last_synchronized_samples_ = true;
    return true;
  }

  StateEstimate updateRobotPathState(
      RobotState* robot, const Eigen::Vector2d& position,
      const ros::Time& measurement_stamp) {
    const StateEstimate candidate = robot->estimator->update(
        position, measurement_stamp.toSec());
    if (candidate.valid) {
      robot->last_valid_path_state = candidate;
      robot->last_valid_path_stamp = measurement_stamp;
      return candidate;
    }

    const double hold_age = robot->last_valid_path_stamp.isZero()
        ? std::numeric_limits<double>::infinity()
        : (measurement_stamp - robot->last_valid_path_stamp).toSec();
    if (candidate.projection.valid &&
        robot->last_valid_path_state.valid &&
        hold_age >= 0.0 &&
        hold_age <= maximum_robot_path_transient_hold_) {
      ROS_WARN_THROTTLE(
          1.0,
          "Holding the last valid %s path state for %.6f s after a temporal estimator resynchronization",
          robot->robot_id.c_str(), hold_age);
      return robot->last_valid_path_state;
    }
    return candidate;
  }

  bool initializeCameraPathAnchor(
      const std::array<const OdometrySample*, kRobotCount>& selected) {
    std::array<Eigen::Vector2d, kRobotCount> supports;
    std::array<SupportOffset, kRobotCount> offsets;
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      supports[index] = selected[index]->support_pose.position;
      offsets[index] = geometry_.config().offsets[index];
    }
    const auto fit = fitRigidLoadPose(supports, offsets);
    if (!fit.valid || fit.rms_residual > maximum_rigid_fit_residual_) {
      const double distance_12 = (supports[0] - supports[1]).norm();
      const double distance_13 = (supports[0] - supports[2]).norm();
      const double distance_23 = (supports[1] - supports[2]).norm();
      ROS_WARN_THROTTLE(
          1.0,
          "Cannot anchor camera-fused fleet: initial rigid-fit residual %.6f m exceeds %.6f m; support distances d12=%.4f d13=%.4f d23=%.4f m (required approximately 0.4000 m each)",
          fit.rms_residual, maximum_rigid_fit_residual_,
          distance_12, distance_13, distance_23);
      return false;
    }
    const auto start = path_.sample(0.0);
    const PlanarPose path_start{{start.position.x(), start.position.y()},
                                start.heading};
    // path_frame_T_camera_world maps the measured initial virtual-load pose
    // onto the canonical s=0 path pose. Raw and fused camera topics remain in
    // the calibrated world frame; only CooperativeState uses this run-local
    // path frame.
    path_frame_to_camera_world_ =
        composePose(path_start, inversePose(fit.pose));
    path_anchor_initialized_ = true;

    for (auto& robot : robots_) {
      robot.estimator->reset(0.0);
      robot.last_valid_path_state = StateEstimate{};
      robot.last_valid_path_stamp = ros::Time();
      for (auto& sample : robot.samples) {
        sample.robot_pose = composePose(
            path_frame_to_camera_world_, sample.robot_pose);
        sample.support_pose =
            composePose(sample.robot_pose, robot.base_to_support);
        sample.path_state = updateRobotPathState(
            &robot, sample.support_pose.position, sample.stamp);
      }
    }
    load_estimator_->reset(0.0);
    last_load_path_state_ = StateEstimate{};
    last_load_measurement_stamp_ = ros::Time();
    last_valid_load_path_stamp_ = ros::Time();
    // The cached snapshot above is still expressed in camera-world. Do not
    // allow it to cross the newly established run-local path-frame boundary.
    has_last_synchronized_samples_ = false;
    ROS_INFO(
        "Anchored camera-fused three-robot state to path start: initial rigid-fit RMS=%.6f m",
        fit.rms_residual);
    return true;
  }

  void fillCameraState(const ros::Time& now,
                       agv_msgs::CooperativeState* state) {
    std::array<const OdometrySample*, kRobotCount> synchronized_samples{};
    bool synchronized =
        selectSynchronizedSamples(now, &synchronized_samples);
    if (synchronized && auto_anchor_from_robot_poses_ &&
        !path_anchor_initialized_) {
      synchronized = initializeCameraPathAnchor(synchronized_samples) &&
          selectSynchronizedSamples(now, &synchronized_samples);
    }
    if (!synchronized) {
      ROS_WARN_THROTTLE(
          1.0,
          "No synchronized camera-fused three-robot snapshot (slop=%.6f s)",
          maximum_sync_slop_);
    }
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const auto* sample = synchronized
          ? synchronized_samples[index] : latestSample(robots_[index]);
      const bool valid = synchronized && sample != nullptr &&
          fresh(*sample, now) &&
          (!auto_anchor_from_robot_poses_ || path_anchor_initialized_);
      state->robot_localization_source[index] =
          valid ? robot_pose_source_
                : static_cast<std::uint8_t>(
                      agv_msgs::CooperativeState::SOURCE_UNKNOWN);
      state->robot_pose_valid[index] = valid;
      state->support_pose_valid[index] = valid;
      state->path_state_valid[index] =
          valid && sample->path_state.valid;
      if (sample != nullptr) {
        state->robot_pose_stamp[index] = sample->stamp;
        state->robot_pose[index] = toMessage(sample->robot_pose);
        state->support_pose[index] = toMessage(sample->support_pose);
        if (sample->path_state.valid) {
          state->s_actual[index] = sample->path_state.progress;
          state->s_dot_actual[index] = sample->path_state.speed;
        }
      }
    }

    if (derive_virtual_load_from_robots_) {
      if (!synchronized ||
          (auto_anchor_from_robot_poses_ && !path_anchor_initialized_)) {
        return;
      }
      const auto minmax = std::minmax_element(
          synchronized_samples.begin(), synchronized_samples.end(),
          [](const OdometrySample* lhs, const OdometrySample* rhs) {
            return lhs->stamp < rhs->stamp;
          });
      std::array<Eigen::Vector2d, kRobotCount> supports;
      std::array<SupportOffset, kRobotCount> offsets;
      for (std::size_t index = 0U; index < kRobotCount; ++index) {
        supports[index] = synchronized_samples[index]->support_pose.position;
        offsets[index] = geometry_.config().offsets[index];
      }
      const auto fit = fitRigidLoadPose(supports, offsets);
      if (!fit.valid || fit.rms_residual > maximum_rigid_fit_residual_) {
        ROS_WARN_THROTTLE(
            1.0,
            "Camera-fused virtual-load rigid-fit residual %.6f m exceeds %.6f m",
            fit.rms_residual, maximum_rigid_fit_residual_);
        return;
      }
      state->load_localization_source = robot_pose_source_;
      state->load_pose_valid = true;
      state->load_pose_stamp = (*minmax.first)->stamp;
      state->load_pose = toMessage(fit.pose);
      updateDerivedLoadPath(fit.pose.position, synchronized_samples);
      state->load_path_state_valid = last_load_path_state_.valid;
      if (last_load_path_state_.valid) {
        state->load_s_actual = last_load_path_state_.progress;
        state->load_s_dot_actual = last_load_path_state_.speed;
      }
      return;
    }

    const OdometrySample* load_sample = nullptr;
    for (auto sample = load_camera_samples_.rbegin();
         sample != load_camera_samples_.rend(); ++sample) {
      if (fresh(*sample, now)) {
        load_sample = &*sample;
        break;
      }
    }
    if (load_sample != nullptr) {
      state->load_localization_source =
          agv_msgs::CooperativeState::SOURCE_CAMERA;
      state->load_pose_valid = true;
      state->load_pose_stamp = load_sample->stamp;
      state->load_pose = toMessage(load_sample->robot_pose);
      state->load_path_state_valid = load_sample->path_state.valid;
      if (load_sample->path_state.valid) {
        state->load_s_actual = load_sample->path_state.progress;
        state->load_s_dot_actual = load_sample->path_state.speed;
      }
    }
  }

  ros::Time representativeLoadStamp(
      const std::array<const OdometrySample*, kRobotCount>& samples) const {
    std::array<ros::Time, kRobotCount> stamps{{
        samples[0]->stamp, samples[1]->stamp, samples[2]->stamp}};
    std::sort(stamps.begin(), stamps.end());
    return stamps[1];
  }

  void updateDerivedLoadPath(
      const Eigen::Vector2d& position,
      const std::array<const OdometrySample*, kRobotCount>& samples) {
    // A rigid fit represents all three synchronized poses.  Using the oldest
    // sample as its derivative timestamp makes the timestamp source switch
    // between robots and can create artificial 2--4 ms intervals.  The
    // median is representative, remains within the synchronization slop and
    // is insensitive to either edge sample.
    const ros::Time measurement_stamp = representativeLoadStamp(samples);
    if (!last_load_measurement_stamp_.isZero() &&
        measurement_stamp <= last_load_measurement_stamp_) {
      return;
    }

    const StateEstimate candidate = load_estimator_->update(
        position, measurement_stamp.toSec());
    last_load_measurement_stamp_ = measurement_stamp;
    if (candidate.valid) {
      last_load_path_state_ = candidate;
      last_valid_load_path_stamp_ = measurement_stamp;
      return;
    }

    // StateEstimator deliberately returns one invalid result while
    // re-synchronizing after an implausible derivative.  If the spatial path
    // projection itself is still valid, retain the preceding path state for
    // a tightly bounded interval.  A true off-path projection is never held.
    const double hold_age = last_valid_load_path_stamp_.isZero()
        ? std::numeric_limits<double>::infinity()
        : (measurement_stamp - last_valid_load_path_stamp_).toSec();
    if (candidate.projection.valid && last_load_path_state_.valid &&
        hold_age >= 0.0 &&
        hold_age <= maximum_load_path_transient_hold_) {
      ROS_WARN_THROTTLE(
          1.0,
          "Holding the last valid virtual-load path state for %.6f s after a temporal estimator resynchronization",
          hold_age);
      return;
    }
    last_load_path_state_ = candidate;
  }

  void publish(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    agv_msgs::CooperativeState state;
    state.header.stamp = now;
    state.header.frame_id = output_frame_;
    if (camera_mode_) {
      fillCameraState(now, &state);
      state_publisher_.publish(state);
      return;
    }
    std::array<const OdometrySample*, kRobotCount> synchronized_samples{};
    const bool synchronized =
        selectSynchronizedSamples(now, &synchronized_samples);
    if (!synchronized) {
      std::array<double, kRobotCount> latest_ages{};
      for (std::size_t index = 0U; index < kRobotCount; ++index) {
        const auto* latest = latestSample(robots_[index]);
        latest_ages[index] = latest == nullptr
            ? std::numeric_limits<double>::infinity()
            : (now - latest->stamp).toSec();
      }
      ROS_WARN_THROTTLE(
          1.0,
          "No synchronized odometry set: latest ages "
          "agv1=%.6f agv2=%.6f agv3=%.6f s (slop=%.6f s)",
          latest_ages[0], latest_ages[1], latest_ages[2],
          maximum_sync_slop_);
    }
    std::array<bool, kRobotCount> is_fresh{};
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const auto& robot = robots_[index];
      const OdometrySample* sample = synchronized
          ? synchronized_samples[index] : latestSample(robot);
      // Individual latest samples remain available for diagnosis, but they
      // are not a valid cooperative snapshot unless all three timestamps can
      // be matched within maximum_sync_slop.
      is_fresh[index] =
          synchronized && sample != nullptr && fresh(*sample, now);
      state.robot_localization_source[index] =
          is_fresh[index] ? static_cast<std::uint8_t>(
                                agv_msgs::CooperativeState::SOURCE_ODOM)
                          : static_cast<std::uint8_t>(
                                agv_msgs::CooperativeState::SOURCE_UNKNOWN);
      state.robot_pose_valid[index] = is_fresh[index];
      state.support_pose_valid[index] = is_fresh[index];
      state.path_state_valid[index] =
          is_fresh[index] && sample->path_state.valid;
      if (sample != nullptr) {
        state.robot_pose_stamp[index] = sample->stamp;
        state.robot_pose[index] = toMessage(sample->robot_pose);
        state.support_pose[index] = toMessage(sample->support_pose);
        if (sample->path_state.valid) {
          state.s_actual[index] = sample->path_state.progress;
          state.s_dot_actual[index] = sample->path_state.speed;
        }
      }
    }

    if (synchronized &&
        std::all_of(is_fresh.begin(), is_fresh.end(), [](bool value) {
          return value;
        })) {
      const auto minmax = std::minmax_element(
          synchronized_samples.begin(), synchronized_samples.end(),
          [](const OdometrySample* lhs, const OdometrySample* rhs) {
            return lhs->stamp < rhs->stamp;
          });
      std::array<Eigen::Vector2d, kRobotCount> supports;
      std::array<SupportOffset, kRobotCount> offsets;
      for (std::size_t index = 0U; index < kRobotCount; ++index) {
        supports[index] = synchronized_samples[index]->support_pose.position;
        offsets[index] = geometry_.config().offsets[index];
      }
      const auto fit = fitRigidLoadPose(supports, offsets);
      if (fit.valid && fit.rms_residual <= maximum_rigid_fit_residual_) {
        state.load_localization_source =
            agv_msgs::CooperativeState::SOURCE_ODOM;
        state.load_pose_valid = true;
        state.load_pose_stamp = (*minmax.first)->stamp;
        state.load_pose = toMessage(fit.pose);
        updateDerivedLoadPath(fit.pose.position, synchronized_samples);
        state.load_path_state_valid = last_load_path_state_.valid;
        if (last_load_path_state_.valid) {
          state.load_s_actual = last_load_path_state_.progress;
          state.load_s_dot_actual = last_load_path_state_.speed;
        }
      }
    }
    state_publisher_.publish(state);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  SCurvePath path_;
  SupportGeometry geometry_;
  std::array<RobotState, kRobotCount> robots_;
  std::deque<OdometrySample> load_camera_samples_;
  std::unique_ptr<StateEstimator> load_estimator_;
  StateEstimate last_load_path_state_;
  ros::Time last_load_measurement_stamp_;
  ros::Time last_valid_load_path_stamp_;
  ros::Publisher state_publisher_;
  ros::Subscriber load_pose_subscriber_;
  ros::Timer timer_;
  double publish_rate_{100.0};
  double maximum_state_age_{0.15};
  double maximum_sync_slop_{0.02};
  double maximum_synchronized_snapshot_hold_{0.05};
  std::size_t synchronization_queue_size_{64U};
  double maximum_rigid_fit_residual_{0.05};
  double maximum_load_path_transient_hold_{0.05};
  double maximum_robot_path_transient_hold_{0.05};
  std::string localization_mode_{"odometry_pretest"};
  std::string load_pose_topic_;
  std::string output_frame_{"world"};
  bool camera_mode_{false};
  bool derive_virtual_load_from_robots_{false};
  bool auto_anchor_from_robot_poses_{false};
  bool path_anchor_initialized_{false};
  std::array<OdometrySample, kRobotCount> last_synchronized_samples_{};
  bool has_last_synchronized_samples_{false};
  PlanarPose path_frame_to_camera_world_;
  std::uint8_t robot_pose_source_{
      agv_msgs::CooperativeState::SOURCE_CAMERA};
  bool require_calibration_epoch_{true};
  std::uint32_t camera_epoch_token_{0U};
  std::unordered_set<std::uint32_t> retired_camera_epoch_tokens_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_state_estimator");
  try {
    multi_agv_control::PathStateEstimatorNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start path_state_estimator: %s", error.what());
    return 1;
  }
  return 0;
}
