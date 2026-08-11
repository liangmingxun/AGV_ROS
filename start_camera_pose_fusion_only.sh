#!/usr/bin/env bash
set -euo pipefail

workspace="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101

exec roslaunch multi_agv_bringup camera_pose_fusion_only.launch \
  calibration_authorized:=true
