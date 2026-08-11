#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace multi_agv_control {
namespace {

constexpr std::size_t kStreamCount = 4U;

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

bool xmlBool(const XmlRpc::XmlRpcValue& value, const char* key,
             bool fallback) {
  if (!value.hasMember(key)) return fallback;
  if (value[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::runtime_error(std::string("parameter field is not boolean: ") +
                             key);
  }
  return static_cast<bool>(value[key]);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace

class CameraPoseAdapterNode {
 public:
  CameraPoseAdapterNode() : private_node_("~") {
    bool calibration_authorized = false;
    bool extrinsics_frozen = false;
    private_node_.param("camera_adapter/calibration_authorized",
                        calibration_authorized, false);
    private_node_.param("camera_adapter/extrinsics_frozen",
                        extrinsics_frozen, false);
    private_node_.param("camera_adapter/world_frame", world_frame_,
                        std::string("world"));
    private_node_.param("camera_adapter/maximum_input_age",
                        maximum_input_age_, 0.15);
    private_node_.param("camera_adapter/maximum_future_offset",
                        maximum_future_offset_, 0.02);
    private_node_.param("camera_adapter/minimum_confidence",
                        minimum_confidence_, 0.5);
    private_node_.param("camera_adapter/require_confidence",
                        require_confidence_, true);
    private_node_.param("camera_adapter/require_calibration_epoch",
                        require_calibration_epoch_, true);
    private_node_.param("camera_adapter/filter_alpha", filter_alpha_, 0.7);
    if (!calibration_authorized || !extrinsics_frozen) {
      throw std::runtime_error(
          "camera calibration/extrinsics are not frozen and authorized; "
          "refusing formal pose output");
    }
    if (world_frame_.empty() || !(maximum_input_age_ > 0.0) ||
        !(maximum_future_offset_ >= 0.0) ||
        !(minimum_confidence_ >= 0.0 && minimum_confidence_ <= 1.0) ||
        !(filter_alpha_ >= 0.0 && filter_alpha_ < 1.0)) {
      throw std::runtime_error("invalid camera adapter configuration");
    }

    XmlRpc::XmlRpcValue streams;
    if (!private_node_.getParam("camera_adapter/streams", streams) ||
        streams.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        streams.size() != static_cast<int>(kStreamCount)) {
      throw std::runtime_error(
          "camera_adapter/streams must contain agv1, agv2, agv3 and load");
    }
    for (std::size_t index = 0U; index < kStreamCount; ++index) {
      loadStream(index, streams[static_cast<int>(index)]);
    }
  }

 private:
  struct Stream {
    std::string entity_id;
    bool enabled{true};
    std::string input_topic;
    std::string confidence_topic;
    std::string raw_output_topic;
    std::string filtered_output_topic;
    double transform_x{0.0};
    double transform_y{0.0};
    double transform_yaw{0.0};
    double confidence{0.0};
    ros::Time confidence_stamp;
    ros::Time last_pose_stamp;
    bool filter_initialized{false};
    double filtered_x{0.0};
    double filtered_y{0.0};
    double filtered_yaw{0.0};
    ros::Subscriber pose_subscriber;
    ros::Subscriber confidence_subscriber;
    ros::Publisher raw_publisher;
    ros::Publisher filtered_publisher;
  };

  void loadStream(std::size_t index, const XmlRpc::XmlRpcValue& value) {
    auto& stream = streams_[index];
    stream.entity_id = xmlString(value, "entity_id");
    stream.enabled = xmlBool(value, "enabled", true);
    static const std::array<std::string, kStreamCount> expected{
        "agv1", "agv2", "agv3", "load"};
    if (stream.entity_id != expected[index]) {
      throw std::runtime_error(
          "camera streams must be ordered agv1, agv2, agv3, load");
    }
    stream.input_topic = xmlString(value, "input_topic");
    stream.confidence_topic = xmlString(value, "confidence_topic");
    stream.raw_output_topic = xmlString(value, "raw_output_topic");
    stream.filtered_output_topic =
        xmlString(value, "filtered_output_topic");
    stream.transform_x = xmlNumber(value, "tag_to_target_x");
    stream.transform_y = xmlNumber(value, "tag_to_target_y");
    stream.transform_yaw = xmlNumber(value, "tag_to_target_yaw");
    if (stream.input_topic.empty() || stream.raw_output_topic.empty() ||
        stream.filtered_output_topic.empty() ||
        (require_confidence_ && stream.confidence_topic.empty()) ||
        !std::isfinite(stream.transform_x) ||
        !std::isfinite(stream.transform_y) ||
        !std::isfinite(stream.transform_yaw)) {
      throw std::runtime_error("invalid camera stream configuration");
    }
    if (!stream.enabled) {
      ROS_WARN("Camera stream %s is configured but disabled because its "
               "rigid transform has not been frozen",
               stream.entity_id.c_str());
      return;
    }
    stream.raw_publisher = node_.advertise<geometry_msgs::PoseStamped>(
        stream.raw_output_topic, 10, false);
    stream.filtered_publisher = node_.advertise<geometry_msgs::PoseStamped>(
        stream.filtered_output_topic, 10, false);
    stream.pose_subscriber = node_.subscribe<geometry_msgs::PoseStamped>(
        stream.input_topic, 10,
        [this, index](const geometry_msgs::PoseStamped::ConstPtr& message) {
          receivePose(index, message);
        });
    if (!stream.confidence_topic.empty()) {
      stream.confidence_subscriber = node_.subscribe<std_msgs::Float64>(
          stream.confidence_topic, 10,
          [this, index](const std_msgs::Float64::ConstPtr& message) {
            receiveConfidence(index, message);
          });
    }
  }

  void receiveConfidence(std::size_t index,
                         const std_msgs::Float64::ConstPtr& message) {
    if (!std::isfinite(message->data)) {
      return;
    }
    streams_[index].confidence = message->data;
    streams_[index].confidence_stamp = ros::Time::now();
  }

  bool calibrationToken(const std::string& frame_id,
                        std::uint32_t* token) const {
    if (frame_id == world_frame_ && !require_calibration_epoch_) {
      *token = 0U;
      return true;
    }
    const std::string prefix = world_frame_ + "@";
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

  void receivePose(std::size_t index,
                   const geometry_msgs::PoseStamped::ConstPtr& message) {
    auto& stream = streams_[index];
    const ros::Time now = ros::Time::now();
    const double age = (now - message->header.stamp).toSec();
    const auto& p = message->pose.position;
    const double yaw = yawFromQuaternion(message->pose.orientation);
    std::uint32_t epoch_token = 0U;
    if (message->header.stamp.isZero() ||
        !calibrationToken(message->header.frame_id, &epoch_token) ||
        age < -maximum_future_offset_ || age > maximum_input_age_ ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) {
      ROS_WARN_THROTTLE(
          1.0, "Rejected camera pose for %s: frame, time or pose invalid",
          stream.entity_id.c_str());
      return;
    }
    if (epoch_token != calibration_epoch_token_) {
      if (retired_calibration_tokens_.count(epoch_token) != 0U) {
        ROS_WARN_THROTTLE(
            1.0,
            "Rejected camera pose for %s from a retired calibration epoch",
            stream.entity_id.c_str());
        return;
      }
      if (calibration_epoch_token_ != 0U) {
        retired_calibration_tokens_.insert(calibration_epoch_token_);
        ROS_ERROR(
            "Camera ground-reference epoch changed (%u -> %u): cleared all "
            "Task15 pose filters; restart any active formal experiment",
            calibration_epoch_token_, epoch_token);
      } else {
        ROS_INFO("Task15 accepted camera calibration epoch token=%u",
                 epoch_token);
      }
      calibration_epoch_token_ = epoch_token;
      for (auto& candidate : streams_) {
        candidate.last_pose_stamp = ros::Time();
        candidate.filter_initialized = false;
      }
    }
    const bool reset_filter_after_gap =
        !stream.last_pose_stamp.isZero() &&
        (message->header.stamp - stream.last_pose_stamp).toSec() >
            maximum_input_age_;
    if (!stream.last_pose_stamp.isZero() &&
        message->header.stamp <= stream.last_pose_stamp) {
      ROS_WARN_THROTTLE(1.0, "Rejected non-increasing camera stamp for %s",
                        stream.entity_id.c_str());
      return;
    }
    stream.last_pose_stamp = message->header.stamp;
    if (reset_filter_after_gap) {
      stream.filter_initialized = false;
      ROS_WARN("Reset camera pose filter for %s after an input gap",
               stream.entity_id.c_str());
    }

    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    geometry_msgs::PoseStamped raw;
    raw.header = message->header;
    raw.pose.position.x =
        p.x + cosine * stream.transform_x - sine * stream.transform_y;
    raw.pose.position.y =
        p.y + sine * stream.transform_x + cosine * stream.transform_y;
    raw.pose.position.z = 0.0;
    const double target_yaw = wrapAngle(yaw + stream.transform_yaw);
    raw.pose.orientation = yawQuaternion(target_yaw);
    stream.raw_publisher.publish(raw);

    const bool confidence_valid =
        !require_confidence_ ||
        (!stream.confidence_stamp.isZero() &&
         (now - stream.confidence_stamp).toSec() <= maximum_input_age_ &&
         stream.confidence >= minimum_confidence_);
    if (!confidence_valid) {
      ROS_WARN_THROTTLE(
          1.0, "Camera pose for %s retained as raw but rejected from causal state due to confidence",
          stream.entity_id.c_str());
      return;
    }

    if (!stream.filter_initialized) {
      stream.filtered_x = raw.pose.position.x;
      stream.filtered_y = raw.pose.position.y;
      stream.filtered_yaw = target_yaw;
      stream.filter_initialized = true;
    } else {
      const double measurement_weight = 1.0 - filter_alpha_;
      stream.filtered_x =
          filter_alpha_ * stream.filtered_x +
          measurement_weight * raw.pose.position.x;
      stream.filtered_y =
          filter_alpha_ * stream.filtered_y +
          measurement_weight * raw.pose.position.y;
      stream.filtered_yaw = wrapAngle(
          stream.filtered_yaw +
          measurement_weight * wrapAngle(target_yaw - stream.filtered_yaw));
    }
    geometry_msgs::PoseStamped filtered = raw;
    filtered.pose.position.x = stream.filtered_x;
    filtered.pose.position.y = stream.filtered_y;
    filtered.pose.orientation = yawQuaternion(stream.filtered_yaw);
    stream.filtered_publisher.publish(filtered);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::array<Stream, kStreamCount> streams_;
  std::string world_frame_{"world"};
  double maximum_input_age_{0.15};
  double maximum_future_offset_{0.02};
  double minimum_confidence_{0.5};
  double filter_alpha_{0.7};
  bool require_confidence_{true};
  bool require_calibration_epoch_{true};
  std::uint32_t calibration_epoch_token_{0U};
  std::unordered_set<std::uint32_t> retired_calibration_tokens_;
};

}  // namespace multi_agv_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_provider");
  try {
    multi_agv_control::CameraPoseAdapterNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start camera pose adapter: %s", error.what());
    return 1;
  }
  return 0;
}
