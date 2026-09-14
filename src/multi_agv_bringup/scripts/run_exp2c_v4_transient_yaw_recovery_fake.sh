#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
ros_port=11644

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) [[ $# -ge 2 ]] || exit 2; output_root="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || exit 2; ros_port="$2"; shift 2 ;;
    -h|--help)
      echo "usage: $0 [--output-root DIR] [--ros-port PORT]"
      echo "Screens frozen transient candidate A (B only if safety invalid), then a fresh fake pair."
      exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ ! "$ros_port" =~ ^[0-9]+$ ]] || (( ros_port < 1024 || ros_port > 65535 )); then
  echo "ERROR: --ros-port must be a private port in 1024..65535" >&2; exit 2
fi

if [[ -z "$output_root" ]]; then
  output_root="${workspace}/experiment_data/exp2c_v4_transient_yaw_recovery/$(date +%Y%m%d_%H%M%S)"
fi

cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP || true
if [[ -e "$output_root" ]]; then
  echo "ERROR: output root already exists; fresh v4 evidence is required" >&2; exit 2
fi
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
  local stage="$1" method="$2" candidate="$3"
  local method_id upper_config authorization_config
  if [[ "$method" == M1 ]]; then
    method_id=M1_R1
    upper_config="${bringup_share}/config/exp2c_v4_M1_candidate_${candidate}.yaml"
    authorization_config="${bringup_share}/config/formal_exp2c_v4_M1_${candidate}_authorization.yaml"
  else
    method_id=M1b_R1
    upper_config="${bringup_share}/config/exp2c_v4_M1b_candidate_${candidate}.yaml"
    authorization_config="${bringup_share}/config/formal_exp2c_v4_M1b_${candidate}_authorization.yaml"
  fi
  local run_dir="${output_root}/${stage}/${method_id}"
  local log_dir="${output_root}/logs/${stage}/${method_id}"
  mkdir -p "$(dirname "$run_dir")" "$log_dir"
  echo "EXP2C-V4 FAKE: stage=${stage} method=${method_id} candidate=${candidate}"

  roslaunch multi_agv_bringup formal_fake_algorithm.launch \
    upper_config:="$upper_config" lower_config:="$lower_config" \
    runtime_config:="$runtime_config" path_config:="$path_config" \
    localization_config:="$localization_config" \
    require_recorder_armed:=true command_publication_authorized:=true \
    formal_available_wheel_limit:=0.16 \
    risk_disturbance_enabled:=false \
    yaw_drive_disturbance_enabled:=false \
    >"${log_dir}/algorithm.log" 2>&1 &
  algorithm_pid=$!

  roslaunch multi_agv_bringup formal_evaluation_window.launch \
    platform_transport_type:=fake derating_publication_authorized:=false \
    authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" run_id:="${stage}_${method_id}" \
    method_id:="$method_id" experiment_id:=exp2c_v4_transient_yaw_recovery \
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
    run_id:="$method_id" experiment_id:=exp2c_v4_transient_yaw_recovery \
    method_id:="$method_id" pair_block_id:="$stage" \
    payload_state:=unloaded localization_source:=odom_fake \
    require_windows_sender_manifest:=false path_config:="$path_config" \
    localization_config:="$localization_config" \
    runtime_config:="$runtime_config" upper_config:="$upper_config" \
    lower_config:="$lower_config" execution_authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" evaluation_target:=5.178229715025710 \
    nominal_common_velocity:=0.10 \
    record_topics_config:="${bringup_share}/config/record_topics_exp2c_v4.yaml" \
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
    if [[ -n "$progress" ]] && awk -v value="$progress" 'BEGIN {exit !(value >= 5.178229715025710)}'; then
      break
    fi
    for pid in "$algorithm_pid" "$evaluation_pid" "$recorder_pid"; do
      if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: required fake pipeline process exited during motion" >&2; exit 7
      fi
    done
    if grep -Eq "Runtime rigid-fit hard gate rejected|safety abort latched" "${log_dir}/algorithm.log"; then
      echo "EXP2C-V4: safety-latched run retained for screening failure analysis" >&2
      break
    fi
    sleep 0.2
  done
  if (( SECONDS >= deadline )); then
    echo "EXP2C-V4: incomplete run retained; no gate is relaxed" >&2
  fi
  sleep 1
  rosservice call /experiment_recorder/stop >/dev/null
  wait "$recorder_pid"
  recorder_pid=""
  cleanup_run

  if ! rosrun multi_agv_analysis process_experiment_run.py \
    "$run_dir" "$validation_config" --skip-plots; then
    echo "EXP2C-V4: postprocess invalid; retain original bag/CSV/validation for classification" >&2
  fi
}

# Screening is separate evidence. B is permitted only after safety/rigidity/limiter failure of A.
selected=""
for candidate in A B; do
  run_one "screen_${candidate}" M1b "$candidate"
  if rosrun multi_agv_analysis analyze_exp2c_v4_transient_yaw.py \
      --screen "${output_root}/screen_${candidate}/M1b_R1" --candidate "$candidate" \
      --log "${output_root}/logs/screen_${candidate}/M1b_R1/algorithm.log" \
      --report "${output_root}/screen_${candidate}.json" \
      --freeze-output "${output_root}/selected_candidate.json" \
      >"${output_root}/screen_${candidate}_stdout.json"; then
    selected="$candidate"
    break
  else
    screen_status=$?
    if [[ "$candidate" == A && "$screen_status" == 10 ]]; then
      echo "EXP2C-V4: A safety/rigidity/limiter invalid; testing ONLY frozen backup B"
    else
      echo "EXP2C_V4_STOP: candidate=$candidate status=$screen_status; no further tuning" >&2
      exit "$screen_status"
    fi
  fi
done
[[ -n "$selected" ]] || { echo "EXP2C_V4_STOP: no admissible candidate" >&2; exit 10; }
echo "EXP2C-V4 frozen candidate=$selected; fresh paired runs, screening not reused"
run_one paired M1b "$selected"
run_one paired M1 "$selected"
rosrun multi_agv_analysis analyze_exp2c_v4_transient_yaw.py \
  --paired-root "$output_root" --candidate "$selected" \
  >"${output_root}/paired_analyzer_stdout.json"
echo "EXP2C_V4_FAKE_RESULTS=$output_root"
echo "No serial, hardware authorization change, commit/push or extra parameter search."
