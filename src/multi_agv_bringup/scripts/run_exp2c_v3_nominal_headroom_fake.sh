#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
ros_port=11643

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    -h|--help)
      echo "usage: $0 [--output-root DIR] [--ros-port PORT]"
      echo "Runs the Exp2c-v3 no-disturbance gate, then a fresh A=0.020 paired fake comparison."
      exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "$output_root" ]]; then
  output_root="${workspace}/experiment_data/exp2c_v3_nominal_headroom_fake/$(date +%Y%m%d_%H%M%S)"
fi

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

bringup_share="$(rospack find multi_agv_bringup)"
analysis_share="$(rospack find multi_agv_analysis)"
path_config="${bringup_share}/config/path_circle_r0p7_cw_smooth_exit.yaml"
runtime_config="${bringup_share}/config/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
lower_config="${bringup_share}/config/exp3_R1.yaml"
evaluation_config="${bringup_share}/config/formal_evaluation_circle_r0p7_smooth_exit.yaml"
validation_config="${analysis_share}/config/validation_defaults.yaml"
localization_config="${bringup_share}/config/localization_odom_exp2c_fake.yaml"

run_one() {
  local stage="$1" method="$2" yaw_enabled="$3"
  local method_id upper_config authorization_config
  if [[ "$method" == M1 ]]; then
    method_id=M1_R1
    upper_config="${bringup_share}/config/exp2c_v3_M1_nominal_headroom.yaml"
    authorization_config="${bringup_share}/config/formal_exp2c_v3_M1_authorization.yaml"
  else
    method_id=M1b_R1
    upper_config="${bringup_share}/config/exp2c_v3_M1b_nominal_headroom.yaml"
    authorization_config="${bringup_share}/config/formal_exp2c_v3_M1b_authorization.yaml"
  fi
  local run_dir="${output_root}/${stage}/${method_id}"
  local log_dir="${output_root}/logs/${stage}/${method_id}"
  mkdir -p "$(dirname "$run_dir")" "$log_dir"
  echo "EXP2C-V3 FAKE: stage=${stage} method=${method_id} yaw=${yaw_enabled} A=0.020"

  roslaunch multi_agv_bringup formal_fake_algorithm.launch \
    upper_config:="$upper_config" lower_config:="$lower_config" \
    runtime_config:="$runtime_config" path_config:="$path_config" \
    localization_config:="$localization_config" \
    require_recorder_armed:=true command_publication_authorized:=true \
    formal_available_wheel_limit:=0.16 \
    risk_disturbance_enabled:=false \
    yaw_drive_disturbance_enabled:="$yaw_enabled" \
    yaw_drive_disturbance_amplitude:=0.020 \
    yaw_drive_disturbance_experiment_id:=exp2c_v3_nominal_headroom_fake \
    >"${log_dir}/algorithm.log" 2>&1 &
  algorithm_pid=$!

  roslaunch multi_agv_bringup formal_evaluation_window.launch \
    platform_transport_type:=fake derating_publication_authorized:=false \
    authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" run_id:="${stage}_${method_id}" \
    method_id:="$method_id" experiment_id:=exp2c_v3_nominal_headroom_fake \
    block_id:="${stage}_${method_id}" \
    >"${log_dir}/evaluation.log" 2>&1 &
  evaluation_pid=$!

  deadline=$((SECONDS + 20))
  until rosnode list 2>/dev/null | grep -Fqx /formal_fake_algorithm; do
    if ! kill -0 "$algorithm_pid" 2>/dev/null; then wait "$algorithm_pid"; fi
    if (( SECONDS >= deadline )); then echo "ERROR: ${method_id} algorithm failed to start" >&2; exit 5; fi
    sleep 0.2
  done

  roslaunch multi_agv_bringup experiment.launch \
    arming_authorized:=true output_root:="$(dirname "$run_dir")" \
    run_id:="$method_id" experiment_id:=exp2c_v3_nominal_headroom_fake \
    method_id:="$method_id" pair_block_id:="$stage" \
    payload_state:=unloaded localization_source:=odom_fake \
    require_windows_sender_manifest:=false path_config:="$path_config" \
    localization_config:="$localization_config" \
    runtime_config:="$runtime_config" upper_config:="$upper_config" \
    lower_config:="$lower_config" execution_authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" evaluation_target:=5.178229715025710 \
    nominal_common_velocity:=0.10 \
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
  if (( SECONDS >= deadline )); then
    echo "ERROR: ${stage}/${method_id} did not complete" >&2
    exit 7
  fi
  sleep 1
  rosservice call /experiment_recorder/stop >/dev/null
  wait "$recorder_pid" || true
  recorder_pid=""
  cleanup_run

  rosrun multi_agv_analysis process_experiment_run.py \
    "$run_dir" "$validation_config" --skip-plots
}

# Stage 1: fresh no-disturbance M1 then M1b. The analyzer returns 10 and the
# script stops before stage 2 if the reduced nominal headroom is intrusive.
run_one stage1_no_disturbance M1 false
run_one stage1_no_disturbance M1b false
rosrun multi_agv_analysis analyze_exp2c_v3_nominal_headroom.py \
  "$output_root" --stage1-only | tee "${output_root}/stage1_analyzer_stdout.json"

# Stage 2: fresh paired counterfactual first, then M1, both at frozen A=0.020.
run_one stage2_yaw_0p020 M1b true
run_one stage2_yaw_0p020 M1 true
rosrun multi_agv_analysis analyze_exp2c_v3_nominal_headroom.py \
  "$output_root" | tee "${output_root}/paired_analyzer_stdout.json"

echo "EXP2C_V3_FAKE_RESULTS=${output_root}"
echo "No serial execution occurred and no hardware authorization was changed."
