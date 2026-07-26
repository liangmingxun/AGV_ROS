#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <XmlRpcValue.h>
#include <agv_msgs/CooperativeState.h>
#include <geometry_msgs/Pose2D.h>
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
    private_node_.param("publish_rate", publish_rate_, 100.0);
    private_node_.param("maximum_state_age", maximum_state_age_, 0.15);
    private_node_.param("maximum_sync_slop", maximum_sync_slop_, 0.02);
    private_node_.param("maximum_rigid_fit_residual",
                        maximum_rigid_fit_residual_, 0.05);
    if (!(publish_rate_ > 0.0) || !(maximum_state_age_ > 0.0) ||
        !(maximum_sync_slop_ >= 0.0) || !(maximum_rigid_fit_residual_ > 0.0)) {
      throw std::runtime_error("invalid localization timing or residual configuration");
    }

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

    state_publisher_ = node_.advertise<agv_msgs::CooperativeState>(
        "/multi_agv/cooperative_state", 5, false);
    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate_),
                              &PathStateEstimatorNode::publish, this);
  }

 private:
  struct RobotState {
    std::string robot_id;
    std::string odom_topic;
    std::string odom_frame;
    std::string base_frame;
    PlanarPose world_to_odom;
    PlanarPose base_to_support;
    std::unique_ptr<StateEstimator> estimator;
    ros::Subscriber subscriber;
    bool received{false};
    ros::Time stamp;
    PlanarPose robot_pose;
    PlanarPose support_pose;
    StateEstimate path_state;
  };

  SCurveConfig loadPathConfig() {
    SCurveConfig config;
    private_node_.param("path_s_curve/amplitude", config.amplitude, 0.05);
    private_node_.param("path_s_curve/longitudinal_length",
                        config.longitudinal_length, 1.0);
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
    robot.odom_topic = xmlString(value, "odom_topic");
    robot.odom_frame = xmlString(value, "odom_frame");
    robot.base_frame = xmlString(value, "base_frame");
    robot.world_to_odom = xmlPose(value, "world_to_odom");
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
    const boost::function<void(const nav_msgs::Odometry::ConstPtr&)> callback =
        [this, index](const nav_msgs::Odometry::ConstPtr& message) {
          receiveOdometry(index, message);
        };
    robot.subscriber = node_.subscribe<nav_msgs::Odometry>(
        robot.odom_topic, 10, callback, ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());
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
    robot.stamp = message->header.stamp;
    robot.robot_pose = composePose(robot.world_to_odom, odom_to_base);
    robot.support_pose = composePose(robot.robot_pose, robot.base_to_support);
    robot.path_state = robot.estimator->update(
        robot.support_pose.position, robot.stamp.toSec());
    robot.received = true;
  }

  bool fresh(const RobotState& robot, const ros::Time& now) const {
    if (!robot.received) {
      return false;
    }
    const double age = (now - robot.stamp).toSec();
    return age >= -maximum_sync_slop_ && age <= maximum_state_age_;
  }

  void publish(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    agv_msgs::CooperativeState state;
    state.header.stamp = now;
    state.header.frame_id = "world";
    std::array<bool, kRobotCount> is_fresh{};
    for (std::size_t index = 0U; index < kRobotCount; ++index) {
      const auto& robot = robots_[index];
      is_fresh[index] = fresh(robot, now);
      state.robot_localization_source[index] =
          is_fresh[index] ? agv_msgs::CooperativeState::SOURCE_ODOM
                          : agv_msgs::CooperativeState::SOURCE_UNKNOWN;
      state.robot_pose_valid[index] = is_fresh[index];
      state.support_pose_valid[index] = is_fresh[index];
      state.path_state_valid[index] =
          is_fresh[index] && robot.path_state.valid;
      if (robot.received) {
        state.robot_pose_stamp[index] = robot.stamp;
        state.robot_pose[index] = toMessage(robot.robot_pose);
        state.support_pose[index] = toMessage(robot.support_pose);
        if (robot.path_state.valid) {
          state.s_actual[index] = robot.path_state.progress;
          state.s_dot_actual[index] = robot.path_state.speed;
        }
      }
    }

    if (std::all_of(is_fresh.begin(), is_fresh.end(), [](bool value) {
          return value;
        })) {
      const auto minmax = std::minmax_element(
          robots_.begin(), robots_.end(),
          [](const RobotState& lhs, const RobotState& rhs) {
            return lhs.stamp < rhs.stamp;
          });
      if (((*minmax.second).stamp - (*minmax.first).stamp).toSec() <=
          maximum_sync_slop_) {
        std::array<Eigen::Vector2d, kRobotCount> supports;
        std::array<SupportOffset, kRobotCount> offsets;
        for (std::size_t index = 0U; index < kRobotCount; ++index) {
          supports[index] = robots_[index].support_pose.position;
          offsets[index] = geometry_.config().offsets[index];
        }
        const auto fit = fitRigidLoadPose(supports, offsets);
        if (fit.valid && fit.rms_residual <= maximum_rigid_fit_residual_) {
          state.load_localization_source = agv_msgs::CooperativeState::SOURCE_ODOM;
          state.load_pose_valid = true;
          state.load_pose_stamp = (*minmax.first).stamp;
          state.load_pose = toMessage(fit.pose);
          if (last_load_measurement_stamp_.isZero() ||
              state.load_pose_stamp > last_load_measurement_stamp_) {
            last_load_path_state_ = load_estimator_->update(
                fit.pose.position, state.load_pose_stamp.toSec());
            last_load_measurement_stamp_ = state.load_pose_stamp;
          }
          state.load_path_state_valid = last_load_path_state_.valid;
          if (last_load_path_state_.valid) {
            state.load_s_actual = last_load_path_state_.progress;
            state.load_s_dot_actual = last_load_path_state_.speed;
          }
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
  std::unique_ptr<StateEstimator> load_estimator_;
  StateEstimate last_load_path_state_;
  ros::Time last_load_measurement_stamp_;
  ros::Publisher state_publisher_;
  ros::Timer timer_;
  double publish_rate_{100.0};
  double maximum_state_age_{0.15};
  double maximum_sync_slop_{0.02};
  double maximum_rigid_fit_residual_{0.05};
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
