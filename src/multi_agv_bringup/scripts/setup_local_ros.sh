#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced so that it can configure the current shell." >&2
  echo "usage: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_agv_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_agv_workspace="$(cd -- "${_agv_script_dir}/../../.." && pwd)"
_agv_ros_setup="/opt/ros/noetic/setup.bash"
_agv_workspace_setup="${_agv_workspace}/devel/setup.bash"

if [[ ! -r "${_agv_ros_setup}" ]]; then
  echo "ROS Noetic setup not found: ${_agv_ros_setup}" >&2
  unset _agv_script_dir _agv_workspace _agv_ros_setup _agv_workspace_setup
  return 2
fi

if [[ ! -r "${_agv_workspace_setup}" ]]; then
  echo "Workspace setup not found: ${_agv_workspace_setup}" >&2
  echo "Build the workspace with catkin_make before sourcing this script." >&2
  unset _agv_script_dir _agv_workspace _agv_ros_setup _agv_workspace_setup
  return 2
fi

source "${_agv_ros_setup}"
source "${_agv_workspace_setup}"
cd -- "${_agv_workspace}" || return 2

unset ROS_IP
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_HOSTNAME="127.0.0.1"

echo "AGV local ROS environment ready"
echo "workspace=${_agv_workspace}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_HOSTNAME=${ROS_HOSTNAME}"

unset _agv_script_dir _agv_workspace _agv_ros_setup _agv_workspace_setup
