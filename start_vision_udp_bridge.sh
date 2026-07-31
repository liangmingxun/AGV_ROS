#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
ROS_DISTRO_SETUP="/opt/ros/noetic/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/devel/setup.bash"
PACKAGE="multi_agv_vision_bridge"
LAUNCH_FILE="vision_udp_bridge.launch"
ROBOT1_IP="192.168.6.101"
MASTER_URI="http://${ROBOT1_IP}:11311"
ROSCORE_LOG="/tmp/multi_agv_vision_bridge_roscore.log"

if [[ ! -f "${ROS_DISTRO_SETUP}" ]]; then
  echo "ERROR: missing ROS Noetic setup: ${ROS_DISTRO_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: missing workspace setup: ${WORKSPACE_SETUP}" >&2
  echo "Build the workspace once with: cd ${WORKSPACE} && catkin_make" >&2
  exit 1
fi

if ! ip -4 address show | grep -Fq "${ROBOT1_IP}/"; then
  echo "ERROR: Robot1 does not currently own IPv4 address ${ROBOT1_IP}." >&2
  echo "Configure the Robot1 network interface before starting ROS." >&2
  ip -4 -brief address show >&2
  exit 1
fi

cd "${WORKSPACE}"

# shellcheck disable=SC1091
source "${ROS_DISTRO_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"

export ROS_MASTER_URI="${MASTER_URI}"
export ROS_IP="${ROBOT1_IP}"
unset ROS_HOSTNAME || true

rospack profile >/dev/null
PACKAGE_PATH="$(rospack find "${PACKAGE}")"
LAUNCH_PATH="${PACKAGE_PATH}/launch/${LAUNCH_FILE}"

if [[ ! -f "${LAUNCH_PATH}" ]]; then
  echo "ERROR: launch file not found: ${LAUNCH_PATH}" >&2
  exit 1
fi

STARTED_ROSCORE=0
ROSCORE_PID=""

cleanup() {
  if [[ "${STARTED_ROSCORE}" -eq 1 ]] && [[ -n "${ROSCORE_PID}" ]]; then
    kill "${ROSCORE_PID}" 2>/dev/null || true
    wait "${ROSCORE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ! rosparam list >/dev/null 2>&1; then
  echo "ROS master is not running; starting roscore at ${ROS_MASTER_URI}"
  roscore >"${ROSCORE_LOG}" 2>&1 &
  ROSCORE_PID=$!
  STARTED_ROSCORE=1

  for _ in {1..40}; do
    if rosparam list >/dev/null 2>&1; then
      break
    fi
    if ! kill -0 "${ROSCORE_PID}" 2>/dev/null; then
      echo "ERROR: roscore exited during startup. Log: ${ROSCORE_LOG}" >&2
      exit 1
    fi
    sleep 0.25
  done

  if ! rosparam list >/dev/null 2>&1; then
    echo "ERROR: ROS master did not become ready. Log: ${ROSCORE_LOG}" >&2
    exit 1
  fi
else
  echo "Using existing ROS master at ${ROS_MASTER_URI}"
fi

echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"
echo "Package: ${PACKAGE_PATH}"
echo "Launch:  ${LAUNCH_PATH}"
echo "UDP:     192.168.6.100:15000 -> 192.168.6.101:15001"
echo "Press Ctrl+C to stop the bridge."

roslaunch "${PACKAGE}" "${LAUNCH_FILE}"
