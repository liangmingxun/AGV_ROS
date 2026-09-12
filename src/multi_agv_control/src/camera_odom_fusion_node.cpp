#include <cmath>
#include <deque>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "multi_agv_control/temporal_pose.hpp"
#include <std_msgs/UInt64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace multi_agv_control {
namespace {

double yawFromQuaternion(const geometry_msgs::Quaternion& message) {
  tf2::Quaternion quaternion(
      message.x, message.y, message.z, message.w);
  if (quaternion.length2() <= 1e-12) {
    throw std::runtime_error("pose quaternion is invalid");
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

std::string xmlString(const XmlRpc::XmlRpcValue& value, const char* key) {
  if (!value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error(
        std::string("missing fusion stream string: ") + key);
  }
  return static_cast<std::string>(value[key]);
}

}  // namespace

class CameraOdomFusionNode {
 public:
  CameraOdomFusionNode() : private_("~") {
    private_.param("maximum_camera_age", maximum_camera_age_, 0.15);
    private_.param("maximum_odometry_interval", maximum_odometry_interval_,
                   0.05);
    private_.param("minimum_confidence", minimum_confidence_, 0.5);
    private_.param("position_measurement_weight", position_weight_, 0.8);
    private_.param("heading_measurement_weight", heading_weight_, 0.8);
    private_.param("maximum_fused_publication_age", maximum_publication_age_, 0.05);
    if (!(maximum_publication_age_ > 0.0 && maximum_publication_age_ <= 0.15) ||
        !(maximum_camera_age_ > 0.0) ||
        !(maximum_odometry_interval_ > 0.0) ||
        !(minimum_confidence_ >= 0.0) ||
        !(position_weight_ > 0.0 && position_weight_ <= 1.0) ||
        !(heading_weight_ > 0.0 && heading_weight_ <= 1.0)) {
      throw std::runtime_error("invalid camera/odometry fusion parameters");
    }

    XmlRpc::XmlRpcValue streams;
    if (!private_.getParam("streams", streams) ||
        streams.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        streams.size() != 3) {
      throw std::runtime_error(
          "fusion streams must contain exactly agv1, agv2 and agv3");
    }
    states_.reserve(3U);
    for (int index = 0; index < streams.size(); ++index) {
      addStream(streams[index], static_cast<std::size_t>(index));
    }
    epoch_subscriber_ = node_.subscribe<std_msgs::UInt64>(
        "/vision/aruco/calibration_epoch", 2,
        &CameraOdomFusionNode::receiveEpoch, this);
    ROS_INFO("Camera/IMU-wheel odometry fusion ready for three AGVs");
  }

 private:
  struct StreamState {
    std::string entity_id;
    std::string camera_pose_topic;
    std::string confidence_topic;
    std::string odometry_topic;
    std::string output_topic;
    ros::Subscriber camera_subscriber;
    ros::Subscriber confidence_subscriber;
    ros::Subscriber odometry_subscriber;
    ros::Publisher publisher;
    ros::Publisher motion_publisher;
    ros::Publisher timing_publisher;
    std::deque<std::pair<ros::Time, Pose2>> odometry_history;
    Pose2 pending_camera;
    ros::Time pending_camera_stamp;
    ros::Time last_camera_stamp;
    bool camera_pending{false};
    Pose2 fused;
    Pose2 previous_odometry;
    ros::Time previous_odometry_stamp;
    ros::WallTime confidence_receive_time;
    ros::WallTime camera_receive_time;
    double confidence{0.0};
    bool has_confidence{false};
    bool has_camera{false};
    bool has_previous_odometry{false};
    bool initialized{false};
    std::string frame_id;
  };

  void addStream(const XmlRpc::XmlRpcValue& config, std::size_t index) {
    auto state = std::make_unique<StreamState>();
    state->entity_id = xmlString(config, "entity_id");
    if (state->entity_id != "agv" + std::to_string(index + 1U)) {
      throw std::runtime_error("fusion streams must be ordered agv1..agv3");
    }
    state->camera_pose_topic = xmlString(config, "camera_pose_topic");
    state->confidence_topic = xmlString(config, "confidence_topic");
    state->odometry_topic = xmlString(config, "odometry_topic");
    state->output_topic = xmlString(config, "output_topic");
    state->publisher = node_.advertise<geometry_msgs::PoseStamped>(
        state->output_topic, 10, false);
    state->motion_publisher = node_.advertise<nav_msgs::Odometry>(
        "/pose_provider/" + state->entity_id + "/base_motion_fused", 10, false);
    state->timing_publisher = node_.advertise<std_msgs::Float64MultiArray>(
        "/pose_provider/" + state->entity_id + "/fusion_timing", 10, false);
    states_.push_back(std::move(state));
    auto& stored = *states_.back();
    stored.camera_subscriber = node_.subscribe<geometry_msgs::PoseStamped>(
        stored.camera_pose_topic, 10,
        [this, index](const geometry_msgs::PoseStamped::ConstPtr& message) {
          receiveCamera(index, message);
        }, ros::VoidConstPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
    stored.confidence_subscriber = node_.subscribe<std_msgs::Float64>(
        stored.confidence_topic, 10,
        [this, index](const std_msgs::Float64::ConstPtr& message) {
          receiveConfidence(index, message);
        });
    const boost::function<void(const ros::MessageEvent<nav_msgs::Odometry const>&)> callback =
        [this, index](const ros::MessageEvent<nav_msgs::Odometry const>& event) {
          receiveOdometry(index, event.getMessage(), event.getReceiptTime());
        };
    stored.odometry_subscriber = node_.subscribe<nav_msgs::Odometry>(
        stored.odometry_topic, 30, callback, ros::VoidConstPtr(),
        ros::TransportHints().reliable().tcpNoDelay());
  }

  void resetAll(const char* reason) {
    for (auto& state : states_) {
      state->initialized = false;
      state->has_camera = false;
      state->has_previous_odometry = false;
      state->odometry_history.clear();
      state->camera_pending = false;
      state->last_camera_stamp = ros::Time();
      state->frame_id.clear();
      state->previous_odometry_stamp = ros::Time();
    }
    ROS_WARN("Reset all camera/odometry fusion streams: %s", reason);
  }

  void receiveEpoch(const std_msgs::UInt64::ConstPtr& message) {
    if (message->data == 0U) {
      resetAll("zero calibration epoch");
      calibration_epoch_ = 0U;
      return;
    }
    if (calibration_epoch_ != 0U && message->data != calibration_epoch_) {
      resetAll("camera calibration epoch changed");
    }
    calibration_epoch_ = message->data;
  }

  void receiveConfidence(
      std::size_t index, const std_msgs::Float64::ConstPtr& message) {
    auto& state = *states_.at(index);
    state.confidence = message->data;
    state.confidence_receive_time = ros::WallTime::now();
    state.has_confidence = std::isfinite(message->data);
  }

  bool confidenceValid(const StreamState& state,
                       const ros::WallTime& now) const {
    return state.has_confidence &&
           (now - state.confidence_receive_time).toSec() <=
               maximum_camera_age_ &&
           state.confidence >= minimum_confidence_;
  }

  void receiveCamera(
      std::size_t index,
      const geometry_msgs::PoseStamped::ConstPtr& message) {
    auto& state = *states_.at(index);
    const ros::WallTime now = ros::WallTime::now();
    if (!timelyPose(message->header.stamp.toSec(), ros::Time::now().toSec(), maximum_camera_age_) ||
        message->header.stamp <= state.last_camera_stamp) return;
    if (calibration_epoch_ == 0U || !confidenceValid(state, now)) {
      ROS_WARN_THROTTLE(1.0,
                        "Rejected fused camera correction for %s: epoch or confidence invalid",
                        state.entity_id.c_str());
      return;
    }
    if (message->header.frame_id.rfind("world@", 0U) != 0U) {
      ROS_WARN_THROTTLE(1.0, "Rejected fused camera frame for %s",
                        state.entity_id.c_str());
      return;
    }
    Pose2 camera;
    try {
      camera = {message->pose.position.x, message->pose.position.y,
                yawFromQuaternion(message->pose.orientation)};
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "Rejected fused camera pose for %s: %s",
                        state.entity_id.c_str(), error.what());
      return;
    }
    if (!std::isfinite(camera.x) || !std::isfinite(camera.y) || !std::isfinite(camera.yaw)) return;
    if (!state.frame_id.empty() && state.frame_id != message->header.frame_id) {
      state.initialized = false;
      state.has_previous_odometry = false;
      state.odometry_history.clear();
      ROS_WARN("Reset %s fusion on world frame change", state.entity_id.c_str());
    }
    state.frame_id = message->header.frame_id;
    state.pending_camera = camera;
    state.pending_camera_stamp = message->header.stamp;
    state.last_camera_stamp = message->header.stamp;
    state.camera_pending = true;
    state.camera_receive_time = now;
    state.has_camera = true;
    applyPendingCamera(state);
    // Camera updates correct the absolute state, but publication is driven by
    // odometry so downstream control receives one regular chassis-rate stream
    // instead of a bursty camera-rate + odometry-rate mixture.
  }

  void receiveOdometry(
      std::size_t index, const nav_msgs::Odometry::ConstPtr& message,
      const ros::Time& receipt) {
    auto& state = *states_.at(index);
    const ros::Time callback_time = ros::Time::now();
    if (message->header.stamp.isZero() ||
        (message->header.stamp - callback_time).toSec() > 0.02 ||
        (state.has_previous_odometry && message->header.stamp <= state.previous_odometry_stamp)) return;
    Pose2 odometry;
    try {
      odometry = {message->pose.pose.position.x,
                  message->pose.pose.position.y,
                  yawFromQuaternion(message->pose.pose.orientation)};
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "Rejected odometry propagation for %s: %s",
                        state.entity_id.c_str(), error.what());
      return;
    }
    if (!std::isfinite(odometry.x) || !std::isfinite(odometry.y) || !std::isfinite(odometry.yaw)) return;
    if (state.has_previous_odometry) {
      const double dt =
          (message->header.stamp - state.previous_odometry_stamp).toSec();
      if (dt > 0.0 && dt <= maximum_odometry_interval_ &&
          state.initialized && state.has_camera) {
        state.fused = compose(
            state.fused, odometryDelta(state.previous_odometry, odometry));
      } else if (!(dt > 0.0 && dt <= maximum_odometry_interval_)) {
        state.initialized = false;
        ROS_WARN_THROTTLE(1.0, "Reset %s odometry delta after %.4f s interval",
                          state.entity_id.c_str(), dt);
      }
    }
    state.previous_odometry = odometry;
    state.previous_odometry_stamp = message->header.stamp;
    state.has_previous_odometry = true;
    state.odometry_history.emplace_back(message->header.stamp, odometry);
    while (state.odometry_history.size() > 100U) state.odometry_history.pop_front();
    applyPendingCamera(state);

    const ros::Time publication_time = ros::Time::now();
    const bool timely = timelyPose(message->header.stamp.toSec(), publication_time.toSec(), maximum_publication_age_);
    std_msgs::Float64MultiArray timing;
    timing.data = {static_cast<double>(message->header.seq), message->header.stamp.toSec(),
        receipt.toSec(), callback_time.toSec(), publication_time.toSec(),
        timely ? 1.0 : 0.0, (callback_time - receipt).toSec()};
    state.timing_publisher.publish(timing);
    if (!timely) {
      ROS_WARN_THROTTLE(1.0, "%s fusion suppressed stale output: source age %.3f s, callback queue wait %.3f s; odometry delta retained",
          state.entity_id.c_str(), (publication_time-message->header.stamp).toSec(), (callback_time-receipt).toSec());
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    if (!state.initialized || !state.has_camera ||
        (now - state.camera_receive_time).toSec() > maximum_camera_age_) {
      if (state.has_camera &&
          (now - state.camera_receive_time).toSec() > maximum_camera_age_) {
        state.initialized = false;
        state.has_camera = false;
        ROS_ERROR_THROTTLE(1.0,
                           "Stopped fused publication for %s: camera stale",
                           state.entity_id.c_str());
      }
      return;
    }
    publish(state, message->header.stamp);
    nav_msgs::Odometry motion;
    motion.header = message->header;
    motion.header.frame_id = state.frame_id;
    motion.child_frame_id = state.entity_id + "/base_link";
    motion.pose.pose.position.x = state.fused.x;
    motion.pose.pose.position.y = state.fused.y;
    motion.pose.pose.orientation = quaternionFromYaw(state.fused.yaw);
    // Body-frame measured twist remains independent of camera corrections.
    // Do not differentiate the corrected absolute pose or use commands here.
    motion.twist = message->twist;
    state.motion_publisher.publish(motion);
  }

  void applyPendingCamera(StreamState& state) {
    if (!state.camera_pending || state.odometry_history.empty()) return;
    const auto& history = state.odometry_history;
    if (state.pending_camera_stamp > history.back().first) return;
    if (state.pending_camera_stamp < history.front().first) {
      state.camera_pending = false;
      return;
    }
    Pose2 camera_odom = history.front().second;
    for (std::size_t i=1; i<history.size(); ++i) {
      if (history[i].first >= state.pending_camera_stamp) {
        const double dt=(history[i].first-history[i-1].first).toSec();
        if (dt > maximum_odometry_interval_) { state.camera_pending=false; return; }
        const double q=(state.pending_camera_stamp-history[i-1].first).toSec()/dt;
        camera_odom=interpolatePose(history[i-1].second, history[i].second, q);
        break;
      }
    }
    if (state.initialized) {
      state.fused=correctAtMeasurementTime(state.fused, history.back().second,
          camera_odom, state.pending_camera, position_weight_, heading_weight_);
    } else {
      state.fused=compose(state.pending_camera, odometryDelta(camera_odom, history.back().second));
      state.initialized=true;
    }
    state.camera_pending=false;
  }

  void publish(const StreamState& state, const ros::Time& stamp) const {
    geometry_msgs::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = state.frame_id;
    message.pose.position.x = state.fused.x;
    message.pose.position.y = state.fused.y;
    message.pose.orientation = quaternionFromYaw(state.fused.yaw);
    state.publisher.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  std::vector<std::unique_ptr<StreamState>> states_;
  ros::Subscriber epoch_subscriber_;
  double maximum_camera_age_{0.15};
  double maximum_publication_age_{0.05};
  double maximum_odometry_interval_{0.05};
  double minimum_confidence_{0.5};
  double position_weight_{0.8};
  double heading_weight_{0.8};
  std::uint64_t calibration_epoch_{0U};
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_odom_fusion");
  try {
    multi_agv_control::CameraOdomFusionNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start camera/odometry fusion: %s", error.what());
    return 1;
  }
  return 0;
}
