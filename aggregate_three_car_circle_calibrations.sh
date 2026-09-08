#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
cd "${WORKSPACE}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source devel/setup.bash

reports=()
for robot in 1 2 3; do
  root="${WORKSPACE}/experiment_data/robot${robot}_camera_circle_r0p7_cw"
  report="$(find "${root}" -type f \
    -name "robot${robot}_circle_analysis.json" 2>/dev/null | sort | tail -n 1)"
  if [[ -z "${report}" ]]; then
    echo "ERROR: no completed Robot${robot} circle analysis under ${root}" >&2
    exit 2
  fi
  reports+=("${report}")
done

run_id="$(date +%Y%m%d_%H%M%S)"
output_root="${WORKSPACE}/experiment_data/three_car_circle_calibration_candidates/${run_id}"
mkdir -p "${output_root}"
output="${output_root}/three_car_circle_r0p7_cw_candidate.yaml"
rosrun multi_agv_bringup aggregate_single_car_circle_calibrations.py \
  --robot1 "${reports[0]}" \
  --robot2 "${reports[1]}" \
  --robot3 "${reports[2]}" \
  --output "${output}"
echo "Review candidate: ${output}"
