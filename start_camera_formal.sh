#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/setup_robot_ros.sh" 1

echo "Starting Task 15 adapter, camera/odometry fusion and path estimator."
echo "Press Ctrl+C to stop."
exec roslaunch multi_agv_bringup camera_formal.launch \
  calibration_authorized:=true

