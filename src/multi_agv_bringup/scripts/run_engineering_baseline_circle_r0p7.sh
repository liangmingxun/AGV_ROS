#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_engineering_baseline_circle_r0p7.sh \
  --operator NAME --pair-block ID \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture

Runs the frozen CAMERA_IMU_WHEEL_FUSED_CLOSED_LOOP engineering controller on
the same R=0.7 m clockwise smooth-entry/smooth-exit circle as M1+R1. Robot2
derating and every M1/M2 upper/lower algorithm are disabled. Raw and 0.8 s
display-smoothed paper figures are produced from the same recording pipeline.
EOF
}

operator=""; pair_block=""
confirm_area=false; confirm_floor=false; confirm_fixture=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --operator) operator="$2"; shift 2 ;;
    --pair-block) pair_block="$2"; shift 2 ;;
    --confirm-area-clear) confirm_area=true; shift ;;
    --confirm-wheels-on-floor) confirm_floor=true; shift ;;
    --confirm-unloaded-30cm-fixture) confirm_fixture=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ -z "$operator" || -z "$pair_block" ||
      "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true ]]; then
  echo "ERROR: operator, pair block and all confirmations are required" >&2
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101

[[ "$(hostname)" == robot1 ]] || {
  echo "ERROR: this serial orchestration must run on Robot1" >&2; exit 3; }

local_sha="$(git rev-parse HEAD)"
for index in 1 2 3; do
  node="/agv${index}/chassis_controller"
  rosnode list | grep -Fqx "$node" || {
    echo "ERROR: missing ${node}" >&2; exit 4; }
  transport="$(rosparam get "${node}/transport_type" 2>/dev/null || true)"
  [[ "$transport" == serial ]] || {
    echo "ERROR: agv${index} is not a serial deployment" >&2; exit 4; }
  available_limit="$(rosparam get \
    "${node}/formal_available_wheel_limit" 2>/dev/null || true)"
  if ! awk -v value="$available_limit" \
      'BEGIN {exit !(value >= 0.159999 && value <= 0.160001)}'; then
    echo "ERROR: agv${index} available wheel limit is ${available_limit:-missing}, expected 0.16 m/s" >&2
    exit 4
  fi
  deployed_sha="$(rosparam get "/agv${index}/deployment/git_sha" 2>/dev/null || true)"
  if [[ -z "$deployed_sha" ]] ||
     ! git cat-file -e "${deployed_sha}^{commit}" 2>/dev/null; then
    echo "ERROR: agv${index} deployment Git SHA is missing or unavailable" >&2
    exit 4
  fi
  if [[ "$index" == 1 ]]; then
    chassis_launch="src/multi_agv_bringup/launch/car1_master.launch"
  else
    chassis_launch="src/multi_agv_bringup/launch/car${index}_client.launch"
  fi
  chassis_paths=(
    src/agv_msgs src/common src/chassis_controller
    src/multi_agv_bringup/config/common_platform.yaml
    "src/multi_agv_bringup/config/agv${index}_chassis.yaml"
    src/multi_agv_bringup/launch/chassis_single.launch
    "$chassis_launch"
    src/multi_agv_bringup/scripts/start_three_car_chassis.sh
  )
  if ! git diff --quiet "$deployed_sha" "$local_sha" -- "${chassis_paths[@]}"; then
    echo "ERROR: agv${index} chassis deployment is incompatible with ${local_sha}" >&2
    git diff --name-only "$deployed_sha" "$local_sha" -- \
      "${chassis_paths[@]}" >&2
    exit 4
  fi
done
for node in /pose_provider /camera_odom_fusion; do
  rosnode list | grep -Fqx "$node" || {
    echo "ERROR: missing camera fusion node ${node}" >&2; exit 5; }
done
for forbidden in /formal_algorithm /formal_fake_algorithm \
                 /three_car_unloaded_bounded_pretest /multi_agv_controller \
                 /path_state_estimator /experiment_recorder; do
  if rosnode list | grep -Fqx "$forbidden"; then
    echo "ERROR: conflicting node is running: ${forbidden}" >&2
    exit 5
  fi
done
alive="$(timeout 3 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null |
  awk '/data:/ {print $2; exit}' || true)"
[[ "$alive" == True ]] || { echo "ERROR: ArUco stream is not alive" >&2; exit 5; }

estimator_pid=""; motion_pid=""; recorder_pid=""
cleanup() {
  for pid in "$recorder_pid" "$motion_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$recorder_pid" "$motion_pid" "$estimator_pid"; do
    [[ -z "$pid" ]] || wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM HUP

path_config="${workspace}/src/multi_agv_bringup/config/path_circle_r0p7_cw_smooth_exit.yaml"
baseline_config="${workspace}/src/multi_agv_bringup/config/three_car_engineering_baseline_circle_r0p7.yaml"
roslaunch multi_agv_bringup camera_fused_virtual_load_state_estimator.launch \
  path_config:="$path_config" &
estimator_pid="$!"
deadline=$((SECONDS + 15))
until rosnode list 2>/dev/null | grep -Fqx /path_state_estimator; do
  kill -0 "$estimator_pid" 2>/dev/null || wait "$estimator_pid"
  (( SECONDS < deadline )) || { echo "ERROR: state estimator timeout" >&2; exit 6; }
  sleep 0.2
done
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5 --require-fused-cooperative-state

run_id="engineering_baseline_circle_r0p7_cw_smooth_exit_$(date +%Y%m%d_%H%M%S)"
output_root="${workspace}/experiment_data/formal_serial_unloaded"

# The motion node waits for the recorder's command subscribers, so both can
# be started without an unrecorded command interval.
roslaunch multi_agv_bringup three_car_engineering_baseline_circle_r0p7.launch \
  run_id:="$run_id" block_id:="$pair_block" \
  platform_transport_type:=serial enable_commands:=true \
  confirm_readonly_gate_passed:=true confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true confirm_unloaded_30cm_fixture:=true &
motion_pid="$!"
roslaunch multi_agv_bringup engineering_baseline_experiment.launch \
  output_root:="$output_root" run_id:="$run_id" operator:="$operator" \
  pair_block_id:="$pair_block" &
recorder_pid="$!"

# The final PathReference is published only after repeated zero commands and
# zero-wheel confirmation. Close the recorder while the controller retains
# its sole command authority during the configured terminal hold.
target_progress="5.178229715025710"
deadline=$((SECONDS + 100))
reached=false
while (( SECONDS < deadline )); do
  if ! kill -0 "$recorder_pid" 2>/dev/null; then
    wait "$recorder_pid" || true
    echo "ERROR: experiment recorder exited before task completion" >&2
    exit 7
  fi
  if ! kill -0 "$motion_pid" 2>/dev/null; then
    set +e; wait "$motion_pid"; motion_status="$?"; set -e
    motion_pid=""
    echo "ERROR: engineering baseline controller exited before recorder stop (status=${motion_status})" >&2
    exit 7
  fi
  progress="$(timeout 1 rostopic echo -n 1 /multi_agv/path_reference 2>/dev/null |
    awk '/load_path_progress_reference:/ {print $2; exit}' || true)"
  if [[ -n "$progress" ]] && awk -v value="$progress" -v target="$target_progress" \
      'BEGIN {exit !(value >= target - 0.000001)}'; then
    reached=true
    break
  fi
  sleep 0.05
done
if [[ "$reached" != true ]]; then
  echo "ERROR: engineering baseline did not reach the bounded target" >&2
  exit 7
fi
if ! rosservice list 2>/dev/null | grep -Fqx /experiment_recorder/stop; then
  echo "ERROR: experiment recorder stop service disappeared before terminal handoff" >&2
  exit 8
fi
rosservice call /experiment_recorder/stop >/dev/null
wait "$recorder_pid" || true
recorder_pid=""
set +e
wait "$motion_pid"
motion_status="$?"
set -e
motion_pid=""
if [[ "$motion_status" -ne 0 ]]; then
  echo "ENGINEERING BASELINE PHYSICAL TASK FAILED (status=${motion_status})" >&2
  exit "$motion_status"
fi
kill -INT "$estimator_pid" 2>/dev/null || true
wait "$estimator_pid" 2>/dev/null || true
estimator_pid=""

run_dir="${output_root}/${run_id}"
echo "PHYSICAL_TASK_STATUS=PASSED run_dir=${run_dir}"
validation="$(rospack find multi_agv_analysis)/config/validation_engineering_baseline.yaml"
if rosrun multi_agv_analysis process_experiment_run.py "$run_dir" "$validation"; then
  echo "POSTPROCESS_STATUS=PASSED"
  echo "SUMMARY: ${run_dir}/summary_metrics.json"
  echo "VALIDATION: ${run_dir}/validation.json"
  echo "RAW FIGURES: ${run_dir}/plots_raw"
  echo "SMOOTHED FIGURES: ${run_dir}/plots_smoothed_0p8s"
else
  result="$?"
  echo "POSTPROCESS_STATUS=FAILED; retained: ${run_dir}" >&2
  echo "Offline retry:" >&2
  echo "rosrun multi_agv_analysis process_experiment_run.py '${run_dir}' '${validation}'" >&2
  exit $((20 + result))
fi
echo "ENGINEERING BASELINE CIRCLE RUN AND FIGURES COMPLETE: ${run_dir}"
