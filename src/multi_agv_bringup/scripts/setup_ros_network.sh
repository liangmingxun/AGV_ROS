#!/usr/bin/env bash
set -euo pipefail

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "source this script: source setup_ros_network.sh <local-ip> [master-ip]" >&2
  exit 2
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: source setup_ros_network.sh <local-ip> [master-ip]" >&2
  return 2
fi

local_ip="$1"
master_ip="${2:-192.168.0.50}"
export ROS_IP="$local_ip"
export ROS_MASTER_URI="http://${master_ip}:11311"

echo "ROS_IP=${ROS_IP}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
