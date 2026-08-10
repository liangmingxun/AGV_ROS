#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
ROBOT1_IP="192.168.6.101"

usage() {
  echo "Usage: $0 --robot-index {1|2|3} --chassis-mode {local|remote} --confirm-test-area-clear --confirm-wheels-on-floor" >&2
}

if [[ $# -ne 6 || "$1" != "--robot-index" ||
      "$3" != "--chassis-mode" ||
      "$5" != "--confirm-test-area-clear" ||
      "$6" != "--confirm-wheels-on-floor" ]]; then
  usage
  exit 2
fi
ROBOT_INDEX="$2"
CHASSIS_MODE="$4"
if [[ ! "${ROBOT_INDEX}" =~ ^[123]$ ]] ||
   [[ "${CHASSIS_MODE}" != "local" && "${CHASSIS_MODE}" != "remote" ]]; then
  usage
  exit 2
fi
if [[ "${ROBOT_INDEX}" == "1" && "${CHASSIS_MODE}" != "local" ]] ||
   [[ "${ROBOT_INDEX}" != "1" && "${CHASSIS_MODE}" != "remote" ]]; then
  echo "ERROR: Robot1 must use local chassis mode; Robot2/3 must use remote mode." >&2
  exit 2
fi

ROBOT_NAME="agv${ROBOT_INDEX}"
ROBOT_LABEL="Robot${ROBOT_INDEX}"
ROBOT_NS="/${ROBOT_NAME}"
POSE_NS="/pose_provider/${ROBOT_NAME}"
OUTPUT_ROOT="${WORKSPACE}/experiment_data/robot${ROBOT_INDEX}_camera_s_closed_loop"

cd "${WORKSPACE}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source devel/setup.bash
export ROS_MASTER_URI="http://${ROBOT1_IP}:11311"
export ROS_IP="${ROBOT1_IP}"
unset ROS_HOSTNAME || true

has_publisher() {
  timeout 3 rostopic info "$1" 2>/dev/null |
    sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '
}

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
             "${POSE_NS}/base_pose_raw"; do
  if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "ERROR: ${topic} has no fresh message; no motion sent." >&2
    exit 2
  fi
done

if has_publisher "${ROBOT_NS}/chassis_command"; then
  echo "ERROR: ${ROBOT_NS}/chassis_command already has a publisher." >&2
  exit 2
fi
for other in 1 2 3; do
  if [[ "${other}" != "${ROBOT_INDEX}" ]] &&
     has_publisher "/agv${other}/chassis_command"; then
    echo "ERROR: /agv${other}/chassis_command has a publisher; single-car isolation failed." >&2
    exit 2
  fi
done

if [[ "${CHASSIS_MODE}" == "local" ]]; then
  if has_publisher "${ROBOT_NS}/chassis_feedback"; then
    echo "ERROR: a local ${ROBOT_LABEL} chassis node is already running; stop it first." >&2
    exit 2
  fi
else
  if ! has_publisher "${ROBOT_NS}/chassis_feedback"; then
    echo "ERROR: ${ROBOT_LABEL} remote chassis is absent; start car${ROBOT_INDEX}_client.launch on ${ROBOT_LABEL}." >&2
    exit 2
  fi
  if ! timeout 5 rostopic echo -n 1 "${ROBOT_NS}/chassis_feedback" 2>/dev/null |
       grep -Eq "robot_id: ${ROBOT_INDEX}$"; then
    echo "ERROR: ${ROBOT_LABEL} chassis feedback is missing or has the wrong robot_id." >&2
    exit 2
  fi
  if ! timeout 5 rostopic echo -n 1 "${POSE_NS}/base_pose_fused" >/dev/null 2>&1; then
    echo "ERROR: ${ROBOT_LABEL} fused pose is absent; check remote odometry and camera tag." >&2
    exit 2
  fi
fi

run_id="$(date +%Y%m%d_%H%M%S)"
run_directory="${OUTPUT_ROOT}/${run_id}"
mkdir -p "${run_directory}"
bag_path="${run_directory}/robot${ROBOT_INDEX}_camera_s_${run_id}.bag"
epoch_before="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"

{
  echo "run_id=${run_id}"
  echo "robot_index=${ROBOT_INDEX}"
  echo "chassis_mode=${CHASSIS_MODE}"
  echo "feedback_authority=${POSE_NS}/base_pose_fused"
  echo "absolute_pose_authority=${POSE_NS}/base_pose_raw"
  echo "propagation=wheel_translation_plus_corrected_imu_yaw"
  echo "calibration_epoch_before=${epoch_before}"
  echo "path=anchored_single_period_s"
  echo "path_speed_mps=0.05"
  echo "angular_feedforward_scale=1.0"
  echo "curvature_preview_seconds=0.0"
  echo "completion=immediate_stop_then_passive_endpoint_record"
  echo "git_commit=$(git rev-parse HEAD 2>/dev/null || echo unknown)"
} >"${run_directory}/metadata.txt"

echo "${ROBOT_LABEL} will follow an automatically anchored fused-feedback S path."
echo "Required clear area: at least 1.5 m forward and 0.4 m to the vehicle's right."
echo "Bag: ${bag_path}"
echo "Motion begins after the node's 3-second zero-speed countdown."

set +e
if [[ "${CHASSIS_MODE}" == "local" ]]; then
  roslaunch multi_agv_bringup robot1_camera_s_capture.launch \
    bag_path:="${bag_path}" \
    confirm_test_area_clear:=true \
    confirm_wheels_on_floor:=true \
    2>&1 | tee "${run_directory}/run.log"
else
  roslaunch multi_agv_bringup single_car_camera_s_capture.launch \
    robot_index:="${ROBOT_INDEX}" \
    bag_path:="${bag_path}" \
    confirm_test_area_clear:=true \
    confirm_wheels_on_floor:=true \
    2>&1 | tee "${run_directory}/run.log"
fi
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
result_topic="${ROBOT_NS}/s_pretest/result"
result_status="$(
  { rostopic echo -b "${bag_path}" "${result_topic}" 2>/dev/null || true; } |
    awk -F': ' '/^data: / {value=$2} END {gsub(/"/, "", value); print value}'
)"
echo "result_status=${result_status:-MISSING}" >>"${run_directory}/metadata.txt"
if [[ "${launch_status}" -ne 0 ]]; then
  echo "ERROR: ${ROBOT_LABEL} closed-loop launch failed with status ${launch_status}." >&2
  echo "Recorded result: ${result_status:-MISSING}" >&2
  exit "${launch_status}"
fi
if [[ "${result_status}" != "STOP_CONFIRMED" ]]; then
  echo "ERROR: ${ROBOT_LABEL} did not publish STOP_CONFIRMED." >&2
  echo "Recorded result: ${result_status:-MISSING}" >&2
  exit 8
fi

echo "PASS: ${ROBOT_LABEL} fused-camera S run completed and stopped."
echo "Results: ${run_directory}"
