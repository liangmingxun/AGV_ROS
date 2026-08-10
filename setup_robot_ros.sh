#!/usr/bin/env bash
# Source this file so ROS_* exports remain in the current terminal:
#   source ./setup_robot_ros.sh 1

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: source this script instead of executing it." >&2
  echo "Usage: source ./setup_robot_ros.sh <robot-index: 1|2|3>" >&2
  exit 2
fi

if [[ $# -ne 1 || ! "$1" =~ ^[123]$ ]]; then
  echo "Usage: source ./setup_robot_ros.sh <robot-index: 1|2|3>" >&2
  return 2
fi

AGV_WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
AGV_ROBOT_INDEX="$1"
AGV_ROS_MASTER_IP="192.168.6.101"
case "${AGV_ROBOT_INDEX}" in
  1) AGV_LOCAL_IP="192.168.6.101" ;;
  2) AGV_LOCAL_IP="192.168.6.102" ;;
  3) AGV_LOCAL_IP="192.168.6.103" ;;
esac

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ERROR: /opt/ros/noetic/setup.bash is missing." >&2
  return 3
fi
if [[ ! -f "${AGV_WORKSPACE}/devel/setup.bash" ]]; then
  echo "ERROR: ${AGV_WORKSPACE}/devel/setup.bash is missing; run catkin_make first." >&2
  return 4
fi
if ! ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 |
     grep -Fqx "${AGV_LOCAL_IP}"; then
  echo "ERROR: this computer does not own ${AGV_LOCAL_IP}; check the robot index or network configuration." >&2
  return 5
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "${AGV_WORKSPACE}/devel/setup.bash"
export ROS_MASTER_URI="http://${AGV_ROS_MASTER_IP}:11311"
export ROS_IP="${AGV_LOCAL_IP}"
unset ROS_HOSTNAME

cd "${AGV_WORKSPACE}" || return 6
echo "Robot${AGV_ROBOT_INDEX} ROS environment ready"
echo "ROS_IP=${ROS_IP}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"

