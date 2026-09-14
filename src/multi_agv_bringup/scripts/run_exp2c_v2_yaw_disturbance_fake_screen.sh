#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
ros_port=11641
candidate_set=round1
run_mode=screen

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    --candidate-set) candidate_set="$2"; shift 2 ;;
    --paired-frozen-0p020) run_mode=paired; shift ;;
    -h|--help)
      echo "usage: $0 [--candidate-set round1|round2|--paired-frozen-0p020] [--output-root DIR] [--ros-port PORT]"
      echo "Runs only M1b fake candidates; round2 is 0.016, 0.020 and 0.024 m/s."
      exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ "$run_mode" == screen && "$candidate_set" != round1 && "$candidate_set" != round2 ]]; then
  echo "ERROR: --candidate-set must be round1 or round2" >&2
  exit 2
fi
if [[ -z "$output_root" ]]; then
  if [[ "$run_mode" == paired ]]; then
    output_root="${workspace}/experiment_data/exp2c_v2_yaw_paired_fake/$(date +%Y%m%d_%H%M%S)"
  elif [[ "$candidate_set" == round1 ]]; then
    output_root="${workspace}/experiment_data/exp2c_v2_yaw_fake_screen/$(date +%Y%m%d_%H%M%S)"
  else
    output_root="${workspace}/experiment_data/exp2c_v2_yaw_fake_screen_round2/$(date +%Y%m%d_%H%M%S)"
  fi
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
  local level_name="$1" amplitude="$2" method="$3"
  local method_id upper_config authorization_config
  if [[ "$method" == M1 ]]; then
    method_id=M1_R1
    upper_config="${bringup_share}/config/exp2c_v2_M1_yaw_disturbance.yaml"
    authorization_config="${bringup_share}/config/formal_exp2c_v2_M1_yaw_disturbance_authorization.yaml"
  else
    method_id=M1b_R1
    upper_config="${bringup_share}/config/exp2c_v2_M1b_yaw_disturbance.yaml"
    authorization_config="${bringup_share}/config/formal_exp2c_v2_M1b_yaw_disturbance_authorization.yaml"
  fi
  local run_dir
  if [[ "$run_mode" == paired ]]; then
    run_dir="${output_root}/${method_id}"
  else
    run_dir="${output_root}/${level_name}/${method_id}"
  fi
  local log_dir="${output_root}/logs/${level_name}/${method_id}"
  mkdir -p "$(dirname "$run_dir")" "$log_dir"
  echo "EXP2C-V2 FAKE: mode=${run_mode} method=${method_id} amplitude=${amplitude} master=${ROS_MASTER_URI}"

  roslaunch multi_agv_bringup formal_fake_algorithm.launch \
    upper_config:="$upper_config" lower_config:="$lower_config" \
    runtime_config:="$runtime_config" path_config:="$path_config" \
    localization_config:="$localization_config" \
    require_recorder_armed:=true command_publication_authorized:=true \
    formal_available_wheel_limit:=0.16 \
    risk_disturbance_enabled:=false \
    yaw_drive_disturbance_enabled:=true \
    yaw_drive_disturbance_amplitude:="$amplitude" \
    >"${log_dir}/algorithm.log" 2>&1 &
  algorithm_pid=$!

  roslaunch multi_agv_bringup formal_evaluation_window.launch \
    platform_transport_type:=fake derating_publication_authorized:=false \
    authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" run_id:="${level_name}_${method_id}" \
    method_id:="$method_id" experiment_id:=exp2c_v2_yaw_disturbance_fake \
    block_id:="${level_name}_${method_id}_${run_mode}" \
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
    run_id:="$method_id" experiment_id:=exp2c_v2_yaw_disturbance_fake \
    method_id:="$method_id" pair_block_id:="${level_name}_${run_mode}" \
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

  local completed=true
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
    echo "WARN: ${method_id} run did not complete; retaining it as rejected" >&2
    completed=false
  fi
  sleep 1
  rosservice call /experiment_recorder/stop >/dev/null
  wait "$recorder_pid" || true
  recorder_pid=""
  cleanup_run

  set +e
  rosrun multi_agv_analysis process_experiment_run.py \
    "$run_dir" "$validation_config" --skip-plots
  local processing_status=$?
  set -e
  if [[ "$completed" == true && $processing_status -ne 0 ]]; then
    echo "ERROR: completed ${method_id} run failed postprocessing" >&2
    exit 8
  fi
  if [[ "$completed" == false && $processing_status -eq 0 ]]; then
    echo "ERROR: timed-out candidate was unexpectedly accepted as valid" >&2
    exit 9
  fi
}

if [[ "$run_mode" == paired ]]; then
  run_one paired_0p020 0.020 M1b
  run_one paired_0p020 0.020 M1
  rosrun multi_agv_analysis analyze_exp2c_v2_yaw_paired.py "$output_root"
  echo "EXP2C_V2_PAIRED_FAKE_RESULTS=${output_root}"
  echo "Frozen amplitude=0.020 m/s. No hardware authorization was changed."
  exit 0
elif [[ "$candidate_set" == round1 ]]; then
  levels=(0p004 0p007 0p010 0p012)
else
  levels=(0p016 0p020 0p024)
fi
for level_name in "${levels[@]}"; do
  case "$level_name" in
    0p004) amplitude=0.004 ;;
    0p007) amplitude=0.007 ;;
    0p010) amplitude=0.010 ;;
    0p012) amplitude=0.012 ;;
    0p016) amplitude=0.016 ;;
    0p020) amplitude=0.020 ;;
    0p024) amplitude=0.024 ;;
  esac
  run_one "$level_name" "$amplitude" M1b
done

set +e
rosrun multi_agv_analysis screen_exp2c_v2_yaw_disturbance.py \
  "$output_root" --candidate-set "$candidate_set"
selection_status=$?
set -e
echo "EXP2C_V2_FAKE_SCREEN_RESULTS=${output_root}"
echo "No M1 paired run was started. No hardware authorization was changed."
exit "$selection_status"
