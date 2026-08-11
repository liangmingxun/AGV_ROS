#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
ROBOT1_IP="192.168.6.101"

usage() {
  echo "Usage: $0 --robot-index {1|2|3} --confirm-test-area-clear --confirm-wheels-on-floor" >&2
}

if [[ $# -ne 4 || "$1" != "--robot-index" ||
      "$3" != "--confirm-test-area-clear" ||
      "$4" != "--confirm-wheels-on-floor" ||
      ! "$2" =~ ^[123]$ ]]; then
  usage
  exit 2
fi
ROBOT_INDEX="$2"
ROBOT_NAME="agv${ROBOT_INDEX}"
ROBOT_LABEL="Robot${ROBOT_INDEX}"
ROBOT_NS="/${ROBOT_NAME}"
POSE_NS="/pose_provider/${ROBOT_NAME}"

cd "${WORKSPACE}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source devel/setup.bash
export ROS_MASTER_URI="http://${ROBOT1_IP}:11311"
export ROS_IP="${ROBOT1_IP}"
unset ROS_HOSTNAME || true

if ! timeout 5 rosparam get /rosversion >/dev/null 2>&1; then
  echo "ERROR: ROS master is not responding." >&2
  exit 2
fi
if ! timeout 5 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null |
     grep -Fq "data: True"; then
  echo "ERROR: ArUco UDP link is not alive; no motion sent." >&2
  exit 2
fi
for topic in "${ROBOT_NS}/chassis_feedback" \
             "${POSE_NS}/base_pose_raw" \
             "/camera/world/${ROBOT_NAME}_confidence"; do
  if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "ERROR: ${topic} has no fresh message; no motion sent." >&2
    exit 2
  fi
done
for index in 1 2 3; do
  command_topic="/agv${index}/chassis_command"
  if timeout 3 rostopic info "${command_topic}" 2>/dev/null |
     sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '; then
    echo "ERROR: ${command_topic} already has a publisher; single-car isolation failed." >&2
    exit 2
  fi
done

run_id="$(date +%Y%m%d_%H%M%S)"
run_directory="${WORKSPACE}/experiment_data/robot${ROBOT_INDEX}_camera_wheel_calibration/${run_id}"
bag_path="${run_directory}/robot${ROBOT_INDEX}_camera_wheel_calibration_${run_id}.bag"
mkdir -p "${run_directory}"
epoch_before="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
if [[ -z "${epoch_before}" ]]; then
  echo "ERROR: calibration epoch is unavailable; no motion sent." >&2
  exit 2
fi

{
  echo "run_id=${run_id}"
  echo "robot_index=${ROBOT_INDEX}"
  echo "status=RUNNING"
  echo "motion=3_cycles_forward_reverse_positive_arc_negative_arc"
  echo "straight_speed_mps=0.03"
  echo "straight_duration_sec=7.0"
  echo "arc_linear_speed_mps=0.025"
  echo "arc_angular_speed_radps=0.12"
  echo "arc_duration_sec=6.0"
  echo "estimator=per_cycle_median_with_explicit_outlier_rejection"
  echo "parameters_automatically_applied=false"
  echo "minimum_calibration_voltage=11.20"
  echo "calibration_epoch_before=${epoch_before}"
  echo "git_commit=$(git rev-parse HEAD 2>/dev/null || echo unknown)"
} >"${run_directory}/metadata.txt"

echo "${ROBOT_LABEL} automatic camera wheel/turn calibration"
echo "Required clear area: 0.8 m forward/back and 0.6 m on both sides."
echo "Bag: ${bag_path}"
echo "Candidate parameters will NOT be applied automatically."

rosbag record -O "${bag_path}" \
  /vision/aruco/alive \
  /vision/aruco/calibration_epoch \
  /vision/aruco/diagnostics \
  "/camera/world/${ROBOT_NAME}_tag_pose" \
  "/camera/world/${ROBOT_NAME}_confidence" \
  "${POSE_NS}/base_pose_raw" \
  "${POSE_NS}/base_pose_filtered" \
  "${POSE_NS}/base_pose_fused" \
  "${ROBOT_NS}/chassis_command" \
  "${ROBOT_NS}/chassis_feedback" \
  "${ROBOT_NS}/odom" \
  "${ROBOT_NS}/imu" \
  >"${run_directory}/rosbag_record.log" 2>&1 &
recorder_pid=$!

cleanup() {
  if kill -0 "${recorder_pid}" 2>/dev/null; then
    kill -INT "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
sleep 2

set +e
rosrun multi_agv_bringup camera_wheel_turn_calibration.py \
  --robot-id "${ROBOT_INDEX}" \
  --output-dir "${run_directory}" \
  --cycles 3 \
  --straight-speed 0.03 \
  --straight-duration 7.0 \
  --arc-linear-speed 0.025 \
  --arc-angular-speed 0.12 \
  --arc-duration 6.0 \
  --nominal-wheel-separation 0.114 \
  --minimum-confidence 0.40 \
  --minimum-calibration-voltage 11.20 \
  --required-command-subscribers 2 \
  --confirm-test-area-clear \
  --confirm-wheels-on-floor \
  2>&1 | tee "${run_directory}/run.log"
calibration_status=${PIPESTATUS[0]}
set -e

cleanup
trap - EXIT INT TERM

epoch_after="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
echo "calibration_epoch_after=${epoch_after}" >>"${run_directory}/metadata.txt"
if [[ "${epoch_after}" != "${epoch_before}" ]]; then
  sed -i 's/^status=RUNNING$/status=INVALID_CALIBRATION_EPOCH_CHANGED/' \
    "${run_directory}/metadata.txt"
  echo "ERROR: camera calibration epoch changed during the run." >&2
  exit 6
fi

if [[ -s "${bag_path}" ]]; then
  rosbag info "${bag_path}" >"${run_directory}/rosbag_info.txt"
else
  echo "ERROR: calibration rosbag is missing or empty." >&2
  exit 7
fi

if [[ "${calibration_status}" -eq 0 ]]; then
  sed -i 's/^status=RUNNING$/status=REVIEW_REQUIRED/' \
    "${run_directory}/metadata.txt"
else
  sed -i 's/^status=RUNNING$/status=FAILED_OR_REJECTED/' \
    "${run_directory}/metadata.txt"
fi

echo "Results: ${run_directory}"
exit "${calibration_status}"
