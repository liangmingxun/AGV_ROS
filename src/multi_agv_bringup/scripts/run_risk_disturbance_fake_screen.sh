#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root="${workspace}/experiment_data/risk_disturbance_fake_screen/$(date +%Y%m%d_%H%M%S)"
ros_port=11631

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    -h|--help)
      echo "usage: $0 [--output-root DIR] [--ros-port PORT]"
      echo "Runs M1/M1b at 0p30, 0p45, 0p60 and 0p70 on local fake chassis only."
      exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done

cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP || true
mkdir -p "$output_root"

if rosnode list >/dev/null 2>&1; then
  echo "ERROR: ROS master already responds on ${ROS_MASTER_URI}; choose another --ros-port" >&2
  exit 3
fi

roscore -p "$ros_port" >"${output_root}/roscore.log" 2>&1 &
roscore_pid=$!
algorithm_pid=""
evaluation_pid=""
recorder_pid=""
cleanup_run() {
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  algorithm_pid=""; evaluation_pid=""; recorder_pid=""
}
cleanup_all() {
  cleanup_run
  if kill -0 "$roscore_pid" 2>/dev/null; then
    kill -INT "$roscore_pid" 2>/dev/null || true
    wait "$roscore_pid" 2>/dev/null || true
  fi
}
trap cleanup_all EXIT INT TERM HUP

deadline=$((SECONDS + 15))
until rosnode list >/dev/null 2>&1; do
  if ! kill -0 "$roscore_pid" 2>/dev/null; then wait "$roscore_pid"; fi
  if (( SECONDS >= deadline )); then echo "ERROR: private roscore failed" >&2; exit 4; fi
  sleep 0.2
done

path_config="$(rospack find multi_agv_bringup)/config/path_circle_r0p7_cw_smooth_exit.yaml"
runtime_config="$(rospack find multi_agv_bringup)/config/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
lower_config="$(rospack find multi_agv_bringup)/config/exp3_R1.yaml"
evaluation_config="$(rospack find multi_agv_bringup)/config/formal_evaluation_circle_r0p7_smooth_exit.yaml"
validation_config="$(rospack find multi_agv_analysis)/config/validation_defaults.yaml"
localization_config="$(rospack find multi_agv_bringup)/config/localization_odom_exp2c_fake.yaml"

run_one() {
  local method="$1" level_name="$2" level_value="$3"
  local method_id upper_config authorization_config run_dir log_dir
  if [[ "$method" == M1 ]]; then
    method_id=M1_R1
    upper_config="$(rospack find multi_agv_bringup)/config/exp2c_M1_risk_disturbance_v1.yaml"
    authorization_config="$(rospack find multi_agv_bringup)/config/formal_exp2c_M1_risk_disturbance_authorization.yaml"
  else
    method_id=M1b_R1
    upper_config="$(rospack find multi_agv_bringup)/config/exp2c_M1b_risk_disturbance_v1.yaml"
    authorization_config="$(rospack find multi_agv_bringup)/config/formal_exp2c_M1b_risk_disturbance_authorization.yaml"
  fi
  run_dir="${output_root}/${level_name}/${method_id}"
  log_dir="${output_root}/logs/${level_name}/${method_id}"
  mkdir -p "${output_root}/${level_name}" "$log_dir"
  echo "FAKE SCREEN: method=${method_id} level=${level_name} master=${ROS_MASTER_URI}"

  roslaunch multi_agv_bringup formal_fake_algorithm.launch \
    upper_config:="$upper_config" lower_config:="$lower_config" \
    runtime_config:="$runtime_config" path_config:="$path_config" \
    localization_config:="$localization_config" \
    require_recorder_armed:=true command_publication_authorized:=true \
    formal_available_wheel_limit:=0.16 \
    risk_disturbance_enabled:=true \
    risk_disturbance_peak_fraction:="$level_value" \
    >"${log_dir}/algorithm.log" 2>&1 &
  algorithm_pid=$!

  roslaunch multi_agv_bringup formal_evaluation_window.launch \
    platform_transport_type:=fake derating_publication_authorized:=false \
    authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" run_id:="${level_name}_${method_id}" \
    method_id:="$method_id" experiment_id:=exp2c_risk_disturbance_v1_fake \
    block_id:="${level_name}_fake_pair" \
    >"${log_dir}/evaluation.log" 2>&1 &
  evaluation_pid=$!

  deadline=$((SECONDS + 20))
  until rosnode list 2>/dev/null | grep -Fqx /formal_fake_algorithm; do
    if ! kill -0 "$algorithm_pid" 2>/dev/null; then wait "$algorithm_pid"; fi
    if (( SECONDS >= deadline )); then echo "ERROR: ${method_id} algorithm failed to start" >&2; exit 5; fi
    sleep 0.2
  done

  roslaunch multi_agv_bringup experiment.launch \
    arming_authorized:=true output_root:="${output_root}/${level_name}" \
    run_id:="$method_id" experiment_id:=exp2c_risk_disturbance_v1_fake \
    method_id:="$method_id" pair_block_id:="${level_name}_fake_pair" \
    payload_state:=unloaded localization_source:=odom_fake \
    require_windows_sender_manifest:=false path_config:="$path_config" \
    localization_config:="$localization_config" \
    runtime_config:="$runtime_config" upper_config:="$upper_config" \
    lower_config:="$lower_config" execution_authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" evaluation_target:=5.178229715025710 \
    >"${log_dir}/recorder.log" 2>&1 &
  recorder_pid=$!

  deadline=$((SECONDS + 25))
  until rosservice list 2>/dev/null | grep -Fqx /experiment_recorder/stop; do
    if ! kill -0 "$recorder_pid" 2>/dev/null; then wait "$recorder_pid"; fi
    if (( SECONDS >= deadline )); then echo "ERROR: ${method_id} recorder failed to arm" >&2; exit 6; fi
    sleep 0.2
  done

  deadline=$((SECONDS + 95))
  while (( SECONDS < deadline )); do
    progress="$(timeout 2 rostopic echo -n 1 /multi_agv/path_reference 2>/dev/null |
      awk '/load_path_progress_reference:/ {print $2; exit}' || true)"
    if [[ -n "$progress" ]] && awk -v value="$progress" 'BEGIN {exit !(value >= 5.177)}'; then
      break
    fi
    if ! kill -0 "$algorithm_pid" 2>/dev/null; then wait "$algorithm_pid"; fi
    sleep 0.2
  done
  if (( SECONDS >= deadline )); then echo "ERROR: ${method_id} did not complete" >&2; exit 7; fi
  sleep 1
  rosservice call /experiment_recorder/stop >/dev/null
  wait "$recorder_pid" || true
  recorder_pid=""
  cleanup_run

  rosrun multi_agv_analysis process_experiment_run.py \
    "$run_dir" "$validation_config" --skip-plots
}

for level_name in 0p30 0p45 0p60 0p70; do
  case "$level_name" in
    0p30) level_value=0.30 ;;
    0p45) level_value=0.45 ;;
    0p60) level_value=0.60 ;;
    0p70) level_value=0.70 ;;
  esac
  run_one M1 "$level_name" "$level_value"
  run_one M1b "$level_name" "$level_value"
done

set +e
rosrun multi_agv_analysis select_risk_disturbance_fake_pairs.py "$output_root"
selection_status=$?
set -e
echo "EXP2C_FAKE_SCREEN_RESULTS=${output_root}"
exit "$selection_status"
