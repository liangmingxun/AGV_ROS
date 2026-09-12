#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_wheel_speed_scale_calibration.sh \
    --robot-id 1|2|3 \
    --operator NAME \
    --confirm-area-clear \
    --confirm-wheels-on-floor \
    --confirm-emergency-stop-ready \
    --confirm-camera-physical-truth \
    --confirm-calibration-only-inverse-scale-excitation

Optional:
  --output-root PATH       Override experiment_data/wheel_speed_scale_calibration.
  --speeds LIST            Default: 0.06,0.08,0.10,0.12,0.14,0.16
  --repetitions N          Default: 3
  --steady-seconds SEC     Default: 3.0
  --ramp-seconds SEC       Default: 1.5

The selected robot's serial chassis node and Robot1 camera-fusion chain must
already be running.  This command records a bag and runs paired forward/reverse
single-car motions.  It never changes production calibration parameters.
EOF
}

robot_id=""
operator=""
speeds="0.06,0.08,0.10,0.12,0.14,0.16"
repetitions="3"
steady_seconds="3.0"
ramp_seconds="1.5"
confirm_area=0
confirm_floor=0
confirm_estop=0
confirm_camera=0
confirm_excitation=0

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root="${workspace}/experiment_data/wheel_speed_scale_calibration"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id) robot_id="${2:-}"; shift 2 ;;
    --operator) operator="${2:-}"; shift 2 ;;
    --output-root) output_root="${2:-}"; shift 2 ;;
    --speeds) speeds="${2:-}"; shift 2 ;;
    --repetitions) repetitions="${2:-}"; shift 2 ;;
    --steady-seconds) steady_seconds="${2:-}"; shift 2 ;;
    --ramp-seconds) ramp_seconds="${2:-}"; shift 2 ;;
    --confirm-area-clear) confirm_area=1; shift ;;
    --confirm-wheels-on-floor) confirm_floor=1; shift ;;
    --confirm-emergency-stop-ready) confirm_estop=1; shift ;;
    --confirm-camera-physical-truth) confirm_camera=1; shift ;;
    --confirm-calibration-only-inverse-scale-excitation) confirm_excitation=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! "${robot_id}" =~ ^[123]$ || -z "${operator}" ]]; then
  echo "ERROR: --robot-id 1|2|3 and --operator are required." >&2
  usage >&2
  exit 2
fi
if (( confirm_area != 1 || confirm_floor != 1 || confirm_estop != 1 ||
      confirm_camera != 1 || confirm_excitation != 1 )); then
  echo "ERROR: all five physical/calibration confirmations are required; no motion sent." >&2
  usage >&2
  exit 2
fi

case "${robot_id}" in
  1) local_ip="192.168.6.101" ;;
  2) local_ip="192.168.6.102" ;;
  3) local_ip="192.168.6.103" ;;
esac
robot="agv${robot_id}"

cd "${workspace}"
if [[ ! -f /opt/ros/noetic/setup.bash || ! -f devel/setup.bash ]]; then
  echo "ERROR: ROS or ${workspace}/devel/setup.bash is missing; build this checkout first." >&2
  exit 3
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source devel/setup.bash
# shellcheck disable=SC1091
source "${script_dir}/setup_ros_network.sh" "${local_ip}" "192.168.6.101"
# shellcheck disable=SC1091
source "${script_dir}/check_local_clock_sync.sh"
check_local_clock_sync

if ! timeout 5 rosparam get /rosversion >/dev/null 2>&1; then
  echo "ERROR: ROS master is not responding at ${ROS_MASTER_URI}; no motion sent." >&2
  exit 4
fi
if ! rosnode list 2>/dev/null | grep -Fqx "/${robot}/chassis_controller"; then
  echo "ERROR: /${robot}/chassis_controller is not running; no motion sent." >&2
  echo "Start this robot with start_three_car_chassis.sh and the confirmed 0.16 envelope." >&2
  exit 4
fi

transport="$(rosparam get "/${robot}/chassis_controller/transport_type" 2>/dev/null || true)"
formal_limit="$(rosparam get "/${robot}/chassis_controller/formal_available_wheel_limit" 2>/dev/null || true)"
if [[ "${transport}" != "serial" ]] ||
   ! awk -v value="${formal_limit:-nan}" \
     'BEGIN {exit !(value + 0.0 == value && (value - 0.16 < 1e-9) && (0.16 - value < 1e-9))}'; then
  echo "ERROR: ${robot} must use serial transport and formal_available_wheel_limit=0.16." >&2
  echo "Observed transport=${transport:-missing}, limit=${formal_limit:-missing}; no motion sent." >&2
  exit 4
fi

if rosnode list 2>/dev/null | grep -Eq '/(formal_algorithm|wheel_speed_scale_calibration)($|_)'; then
  echo "ERROR: another formal/calibration command authority is running; no motion sent." >&2
  exit 4
fi
for topic in \
  /vision/aruco/alive \
  /vision/aruco/calibration_epoch \
  "/pose_provider/${robot}/base_pose_raw" \
  "/camera/world/${robot}_confidence" \
  "/${robot}/chassis_feedback" \
  "/${robot}/capability_report"; do
  if ! timeout 5 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "ERROR: no fresh message on ${topic}; no motion sent." >&2
    exit 4
  fi
done
if ! timeout 5 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null | grep -Fq 'data: True'; then
  echo "ERROR: /vision/aruco/alive is not True; no motion sent." >&2
  exit 4
fi

epoch_before="$({ timeout 5 rostopic echo -n 1 /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
if [[ -z "${epoch_before}" || "${epoch_before}" == "0" ]]; then
  echo "ERROR: camera calibration epoch is missing or zero; no motion sent." >&2
  exit 4
fi

run_id="${robot}_wheel_speed_scale_$(date +%Y%m%d_%H%M%S)"
run_dir="${output_root}/${run_id}"
mkdir -p "${run_dir}"
bag_path="${run_dir}/${run_id}.bag"

cat >"${run_dir}/metadata.txt" <<EOF
run_id=${run_id}
robot_id=${robot}
operator=${operator}
speeds_mps=${speeds}
repetitions=${repetitions}
ramp_seconds=${ramp_seconds}
steady_seconds=${steady_seconds}
camera_calibration_epoch=${epoch_before}
git_commit=$(git rev-parse HEAD)
git_branch=$(git branch --show-current)
hostname=$(hostname)
ros_master_uri=${ROS_MASTER_URI}
ros_ip=${ROS_IP}
production_parameters_modified=false
excitation_mapping=physical_target_divided_by_active_feedback_scale_calibration_only
EOF

bag_pid=""
runner_pid=""
cleanup() {
  local status=$?
  if [[ -n "${runner_pid}" ]] && kill -0 "${runner_pid}" 2>/dev/null; then
    kill -INT "${runner_pid}" 2>/dev/null || true
    wait "${runner_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

echo "Recording ${robot} multi-speed calibration to ${bag_path}"
rosbag record -O "${bag_path}" \
  /vision/aruco/alive \
  /vision/aruco/calibration_epoch \
  /vision/aruco/diagnostics \
  "/camera/world/${robot}_tag_pose" \
  "/camera/world/${robot}_confidence" \
  "/pose_provider/${robot}/base_pose_raw" \
  "/pose_provider/${robot}/base_pose_filtered" \
  "/${robot}/chassis_command" \
  "/${robot}/chassis_feedback" \
  "/${robot}/capability_report" \
  "/${robot}/odom" \
  "/${robot}/imu" \
  >"${run_dir}/rosbag.log" 2>&1 &
bag_pid=$!
sleep 2
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "ERROR: rosbag failed to start; see ${run_dir}/rosbag.log." >&2
  exit 5
fi

command_sequence=$(( $(date +%s) % 2000000000 ))
if (( command_sequence < 100000 )); then
  command_sequence=$((command_sequence + 100000))
fi

set +e
rosrun multi_agv_bringup wheel_speed_scale_calibration.py \
  --robot-id "${robot_id}" \
  --operator "${operator}" \
  --output-dir "${run_dir}" \
  --speeds "${speeds}" \
  --repetitions "${repetitions}" \
  --ramp-seconds "${ramp_seconds}" \
  --steady-seconds "${steady_seconds}" \
  --command-seq "${command_sequence}" \
  --required-command-subscribers 2 \
  > >(tee "${run_dir}/calibration.log") 2>&1 &
runner_pid=$!
wait "${runner_pid}"
runner_status=$?
runner_pid=""
set -e

kill -INT "${bag_pid}" 2>/dev/null || true
wait "${bag_pid}" 2>/dev/null || true
bag_pid=""

epoch_after="$({ timeout 5 rostopic echo -n 1 /vision/aruco/calibration_epoch 2>/dev/null || true; } |
  awk '/data:/ {print $2; exit}')"
printf 'camera_calibration_epoch_after=%s\n' "${epoch_after:-missing}" >>"${run_dir}/metadata.txt"

if [[ "${epoch_after}" != "${epoch_before}" ]]; then
  echo "ERROR: camera calibration epoch changed (${epoch_before} -> ${epoch_after}); data retained." >&2
  exit 6
fi
if (( runner_status != 0 )); then
  echo "ERROR: calibration acquisition failed (status=${runner_status}); bag and partial data retained:" >&2
  echo "${run_dir}" >&2
  exit "${runner_status}"
fi

echo "WHEEL SPEED SCALE CALIBRATION ACQUISITION COMPLETE"
echo "Results: ${run_dir}"
echo "Bag: ${bag_path}"
echo "Candidate is review-only and has not modified any production parameter."
