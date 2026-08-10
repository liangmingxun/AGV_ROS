#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WORKSPACE_SETUP="${WORKSPACE}/devel/setup.bash"
ROBOT1_IP="192.168.6.101"
MASTER_URI="http://${ROBOT1_IP}:11311"
OUTPUT_ROOT="${WORKSPACE}/experiment_data/task15_camera_dynamic"
SPEED_MPS="0.03"
LEG_DURATION_SECONDS="3.0"
SETTLE_SECONDS="1"

CONFIRM_AREA=0
CONFIRM_FLOOR=0
START_CHASSIS=1

usage() {
  cat <<'EOF'
Usage:
  run_robot1_camera_dynamic_capture.sh \
    --confirm-test-area-clear \
    --confirm-wheels-on-floor

Optional:
  --reuse-running-chassis  Require an already-running Robot1 chassis node.
  --output-root PATH       Override the recording directory.
  -h, --help               Show this help.

The frozen motion is Robot1 only: +0.03 m/s for 3 s, stop, then -0.03 m/s
for 3 s and stop. Camera, Task15 and chassis data are recorded automatically.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-test-area-clear)
      CONFIRM_AREA=1
      shift
      ;;
    --confirm-wheels-on-floor)
      CONFIRM_FLOOR=1
      shift
      ;;
    --reuse-running-chassis)
      START_CHASSIS=0
      shift
      ;;
    --output-root)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --output-root requires a path." >&2
        exit 2
      fi
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${CONFIRM_AREA}" -ne 1 || "${CONFIRM_FLOOR}" -ne 1 ]]; then
  echo "ERROR: physical safety confirmations are required; no motion sent." >&2
  usage >&2
  exit 2
fi

if [[ ! -f "${ROS_SETUP}" || ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: ROS or workspace setup is missing; build the workspace first." >&2
  exit 2
fi

cd "${WORKSPACE}"
# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"
export ROS_MASTER_URI="${MASTER_URI}"
export ROS_IP="${ROBOT1_IP}"
unset ROS_HOSTNAME || true

if ! timeout 5 rosparam get /rosversion >/dev/null 2>&1; then
  echo "ERROR: ROS master is not responding at ${ROS_MASTER_URI}." >&2
  exit 2
fi

if ! timeout 5 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null |
     grep -Fq "data: True"; then
  echo "ERROR: /vision/aruco/alive is not True; no motion sent." >&2
  exit 2
fi

for required_topic in \
  /vision/aruco/calibration_epoch \
  /camera/world/agv1_tag_pose \
  /pose_provider/agv1/base_pose_raw \
  /pose_provider/agv1/base_pose_filtered; do
  if ! timeout 5 rostopic echo -n 1 "${required_topic}" >/dev/null 2>&1; then
    echo "ERROR: no fresh message on ${required_topic}; no motion sent." >&2
    exit 2
  fi
done

epoch_before="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
if [[ -z "${epoch_before}" || "${epoch_before}" == "0" ]]; then
  echo "ERROR: calibration epoch is missing or zero; no motion sent." >&2
  exit 2
fi

run_id="$(date +%Y%m%d_%H%M%S)"
run_directory="${OUTPUT_ROOT}/${run_id}"
mkdir -p "${run_directory}"
bag_path="${run_directory}/robot1_camera_dynamic_${run_id}.bag"
chassis_log="${run_directory}/car1_chassis.log"
motion_log="${run_directory}/motion.log"
metadata_path="${run_directory}/metadata.txt"

bag_pid=""
chassis_pid=""
motion_pid=""
started_chassis=0
cleanup_started=0

stop_pid() {
  local signal_name="$1"
  local pid="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  if [[ "${cleanup_started}" -eq 1 ]]; then
    return
  fi
  cleanup_started=1

  # The bounded motion publisher handles SIGINT by sending repeated zero-speed
  # commands before it exits. Stop it before stopping the chassis subscriber.
  stop_pid INT "${motion_pid}"
  stop_pid INT "${bag_pid}"
  if [[ "${started_chassis}" -eq 1 ]]; then
    stop_pid INT "${chassis_pid}"
  fi
  if [[ "${status}" -ne 0 ]]; then
    echo "FAILED: automatic capture exited with status ${status}." >&2
    echo "Inspect logs in: ${run_directory}" >&2
  fi
}
trap cleanup EXIT INT TERM HUP

feedback_ready() {
  timeout 3 rostopic echo -n 1 /agv1/chassis_feedback >/dev/null 2>&1
}

if feedback_ready; then
  echo "Using the already-running Robot1 chassis node."
elif [[ "${START_CHASSIS}" -eq 1 ]]; then
  echo "Starting Robot1 serial chassis node..."
  roslaunch multi_agv_bringup car1_master.launch transport_type:=serial \
    >"${chassis_log}" 2>&1 &
  chassis_pid=$!
  started_chassis=1
  chassis_deadline=$((SECONDS + 20))
  while (( SECONDS < chassis_deadline )); do
    if ! kill -0 "${chassis_pid}" 2>/dev/null; then
      echo "ERROR: Robot1 chassis launch exited; see ${chassis_log}." >&2
      exit 3
    fi
    if feedback_ready; then
      break
    fi
    sleep 0.25
  done
  if ! feedback_ready; then
    echo "ERROR: no Robot1 chassis feedback after 20 seconds." >&2
    echo "Inspect: ${chassis_log}" >&2
    exit 3
  fi
else
  echo "ERROR: Robot1 chassis feedback is absent and automatic startup was disabled." >&2
  exit 3
fi

cat >"${metadata_path}" <<EOF
run_id=${run_id}
robot_id=1
ros_master_uri=${ROS_MASTER_URI}
ros_ip=${ROS_IP}
calibration_epoch_before=${epoch_before}
speed_mps=${SPEED_MPS}
leg_duration_seconds=${LEG_DURATION_SECONDS}
motion=forward_stop_reverse_stop
git_commit=$(git rev-parse HEAD 2>/dev/null || echo unknown)
EOF

echo "Starting rosbag: ${bag_path}"
rosbag record -O "${bag_path}" \
  /vision/aruco/alive \
  /vision/aruco/calibration_epoch \
  /vision/aruco/diagnostics \
  /camera/world/agv1_tag_pose \
  /camera/world/agv1_confidence \
  /pose_provider/agv1/base_pose_raw \
  /pose_provider/agv1/base_pose_filtered \
  /multi_agv/cooperative_state \
  /agv1/chassis_command \
  /agv1/chassis_feedback \
  /agv1/capability_report \
  /agv1/odom \
  /agv1/imu \
  >"${run_directory}/rosbag.log" 2>&1 &
bag_pid=$!
sleep 2
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "ERROR: rosbag failed to start; see ${run_directory}/rosbag.log." >&2
  exit 4
fi

# Wall-clock seconds produce a monotonically increasing sequence across normal
# reruns while leaving room for the motion helper's repeated stop commands.
command_sequence=$(( $(date +%s) % 2000000000 ))
if (( command_sequence < 100000 )); then
  command_sequence=$((command_sequence + 100000))
fi

run_leg() {
  local speed="$1"
  local sequence="$2"
  local label="$3"
  echo "Starting ${label}: speed=${speed} m/s duration=${LEG_DURATION_SECONDS} s"
  rosrun multi_agv_bringup run_timed_straight_test.py \
    --robot-id 1 \
    --speed "${speed}" \
    --duration "${LEG_DURATION_SECONDS}" \
    --command-seq "${sequence}" \
    --experiment-id "task15_camera_dynamic_${run_id}" \
    --required-command-subscribers 2 \
    --confirm-wheels-on-floor \
    >>"${motion_log}" 2>&1 &
  motion_pid=$!
  set +e
  wait "${motion_pid}"
  local leg_status=$?
  set -e
  motion_pid=""
  if [[ "${leg_status}" -ne 0 ]]; then
    echo "ERROR: ${label} failed with status ${leg_status}; see ${motion_log}." >&2
    exit "${leg_status}"
  fi
}

echo "Safety checks passed. Motion starts now. Keep physical power-off reachable."
run_leg "${SPEED_MPS}" "${command_sequence}" "forward leg"
sleep "${SETTLE_SECONDS}"

if ! timeout 5 rostopic echo -n 1 /pose_provider/agv1/base_pose_filtered \
     >/dev/null 2>&1; then
  echo "ERROR: camera feedback disappeared after the forward leg; reverse leg cancelled." >&2
  exit 5
fi

run_leg "-${SPEED_MPS}" "$((command_sequence + 100))" "reverse leg"
sleep "${SETTLE_SECONDS}"

epoch_after="$({ timeout 5 rostopic echo -n 1 \
  /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
echo "calibration_epoch_after=${epoch_after}" >>"${metadata_path}"

stop_pid INT "${bag_pid}"
bag_pid=""
if [[ "${started_chassis}" -eq 1 ]]; then
  stop_pid INT "${chassis_pid}"
  chassis_pid=""
fi

if [[ "${epoch_after}" != "${epoch_before}" ]]; then
  echo "ERROR: calibration epoch changed (${epoch_before} -> ${epoch_after}); bag retained but run is invalid." >&2
  exit 6
fi

if [[ ! -s "${bag_path}" ]]; then
  echo "ERROR: rosbag output is missing or empty." >&2
  exit 7
fi

rosbag info "${bag_path}" >"${run_directory}/rosbag_info.txt"
echo "PASS: bounded Robot1 motion and camera recording completed."
echo "Bag:      ${bag_path}"
echo "Metadata: ${metadata_path}"
echo "Details:  ${run_directory}/rosbag_info.txt"

