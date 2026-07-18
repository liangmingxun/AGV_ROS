#!/usr/bin/env bash
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
if [[ ! "$local_ip" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ||
      ! "$master_ip" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
  echo "local-ip and master-ip must be IPv4 addresses" >&2
  return 2
fi

unset ROS_HOSTNAME
export ROS_IP="$local_ip"
export ROS_MASTER_URI="http://${master_ip}:11311"

echo "ROS_IP=${ROS_IP}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
if command -v chronyc >/dev/null 2>&1; then
  chronyc tracking | grep -E 'Reference ID|System time|Last offset|Leap status' || true
else
  echo "warning: chronyc is unavailable; verify three-host clock sync before recording" >&2
fi
