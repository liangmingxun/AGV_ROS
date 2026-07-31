#!/usr/bin/env bash
set -euo pipefail

NODE="${1:-/vision_udp_bridge}"
required_pose=(agv1 agv2 agv3 load)
for entity in "${required_pose[@]}"; do
  pose="/camera/world/${entity}_tag_pose"
  conf="/camera/world/${entity}_confidence"
  [[ "$(rostopic type "$pose")" == "geometry_msgs/PoseStamped" ]]
  [[ "$(rostopic type "$conf")" == "std_msgs/Float64" ]]
done

node_info="$(rosnode info "$NODE")"
for forbidden in /tf /tf_static /multi_agv/cooperative_state /cmd_vel /agv1/chassis_command /agv2/chassis_command /agv3/chassis_command; do
  if printf '%s\n' "$node_info" | grep -Fq "$forbidden"; then
    echo "ERROR: $NODE unexpectedly references forbidden topic $forbidden" >&2
    exit 2
  fi
done

echo "Task 15 ROS graph contract passed for $NODE"
