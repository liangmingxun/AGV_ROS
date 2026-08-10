#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
OUTPUT_ROOT="${WORKSPACE}/experiment_data/robot1_camera_s_closed_loop"
ROBOT1_IP="192.168.6.101"

if [[ $# -ne 2 || "$1" != "--confirm-test-area-clear" ||
      "$2" != "--confirm-wheels-on-floor" ]]; then
  echo "Usage: $0 --confirm-test-area-clear --confirm-wheels-on-floor" >&2
  exit 2
fi

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
if ! timeout 5 rosnode list 2>/dev/null | grep -Fxq "/camera_odom_fusion"; then
  echo "ERROR: /camera_odom_fusion is absent; restart camera_formal.launch." >&2
  exit 2
fi
for topic in /vision/aruco/calibration_epoch \
             /pose_provider/agv1/base_pose_raw; do
  if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "ERROR: ${topic} has no fresh message; no motion sent." >&2
    exit 2
  fi
done

if timeout 3 rostopic info /agv1/chassis_feedback 2>/dev/null |
   sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '; then
  echo "ERROR: a Robot1 chassis node is already running; stop it first." >&2
  exit 2
fi
if timeout 3 rostopic info /agv1/chassis_command 2>/dev/null |
   sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '; then
  echo "ERROR: /agv1/chassis_command already has a publisher; no motion sent." >&2
  exit 2
fi

run_id="$(date +%Y%m%d_%H%M%S)"
run_directory="${OUTPUT_ROOT}/${run_id}"
mkdir -p "${run_directory}"
bag_path="${run_directory}/robot1_camera_s_${run_id}.bag"

epoch_before="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"

{
  echo "run_id=${run_id}"
  echo "feedback_authority=/pose_provider/agv1/base_pose_fused"
  echo "absolute_pose_authority=/pose_provider/agv1/base_pose_raw"
  echo "propagation=wheel_translation_plus_corrected_imu_yaw"
  echo "calibration_epoch_before=${epoch_before}"
  echo "path=anchored_single_period_s"
  echo "path_speed_mps=0.05"
  echo "angular_feedforward_scale=1.0"
  echo "curvature_preview_seconds=0.0"
  echo "completion=immediate_stop_then_passive_endpoint_record"
  echo "git_commit=$(git rev-parse HEAD 2>/dev/null || echo unknown)"
} >"${run_directory}/metadata.txt"

echo "Robot1 will follow an automatically anchored fused-feedback S path."
echo "Required clear area: at least 1.5 m forward and 0.4 m to Robot1's right."
echo "Bag: ${bag_path}"
echo "Motion begins after the node's 3-second zero-speed countdown."

set +e
roslaunch multi_agv_bringup robot1_camera_s_capture.launch \
  bag_path:="${bag_path}" \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  2>&1 | tee "${run_directory}/run.log"
launch_status=${PIPESTATUS[0]}
set -e

epoch_after="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
echo "calibration_epoch_after=${epoch_after}" >>"${run_directory}/metadata.txt"

if [[ "${epoch_after}" != "${epoch_before}" ]]; then
  echo "ERROR: calibration epoch changed; the retained run is invalid." >&2
  exit 6
fi
if [[ ! -s "${bag_path}" ]]; then
  echo "ERROR: rosbag is missing or empty." >&2
  exit 7
fi

rosbag info "${bag_path}" >"${run_directory}/rosbag_info.txt"
result_status="$(
  { rostopic echo -b "${bag_path}" /agv1/s_pretest/result 2>/dev/null || true; } |
    awk -F': ' '/^data: / {value=$2} END {gsub(/"/, "", value); print value}'
)"
echo "result_status=${result_status:-MISSING}" >>"${run_directory}/metadata.txt"
if [[ "${launch_status}" -ne 0 ]]; then
  echo "ERROR: camera closed-loop launch failed with status ${launch_status}." >&2
  echo "Recorded result: ${result_status:-MISSING}" >&2
  echo "Inspect ${run_directory}/run.log" >&2
  exit "${launch_status}"
fi
if [[ "${result_status}" != "STOP_CONFIRMED" ]]; then
  echo "ERROR: Robot1 did not publish STOP_CONFIRMED." >&2
  echo "Recorded result: ${result_status:-MISSING}" >&2
  echo "Inspect ${run_directory}/run.log" >&2
  exit 8
fi

echo "PASS: Robot1 camera-feedback S run completed and stopped."
echo "Results: ${run_directory}"
