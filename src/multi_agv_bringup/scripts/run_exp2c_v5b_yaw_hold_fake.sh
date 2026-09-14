#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
ros_port=11646
candidate=S1_gamma0p20_down0p6_hold2p5_up0p6_R10
method=pair
baseline=""
baseline_log=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --candidate) candidate="$2"; shift 2 ;;
    --method) method="$2"; shift 2 ;;
    --output-root) [[ $# -ge 2 ]] || exit 2; output_root="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || exit 2; ros_port="$2"; shift 2 ;;
    -h|--help)
      echo "usage: $0 --candidate S1_gamma0p20_down0p6_hold2p5_up0p6_R10|S1R12_gamma0p20_down0p6_hold2p5_up0p6_R12 --method M1|M1b|pair [--output-root DIR] [--ros-port PORT]"
      echo "EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE; fresh M1b/M1 comparison only."
      exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ ! "$ros_port" =~ ^[0-9]+$ ]] || (( ros_port < 1024 || ros_port > 65535 )); then
  echo "ERROR: --ros-port must be a private port in 1024..65535" >&2; exit 2
fi

if [[ -z "$output_root" ]]; then
  output_root="${workspace}/experiment_data/exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_$(date +%Y%m%d_%H%M%S)"
fi

[[ "$candidate" == S1_gamma0p20_down0p6_hold2p5_up0p6_R10 || "$candidate" == S1R12_gamma0p20_down0p6_hold2p5_up0p6_R12 ]] || { echo "Invalid exploration candidate" >&2; exit 2; }
[[ "$method" == M1 || "$method" == M1b || "$method" == pair ]] || { echo "Invalid exploration method" >&2; exit 2; }
cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP || true
if [[ -e "$output_root" ]]; then
  echo "ERROR: output root already exists; fresh exploration evidence is required" >&2; exit 2
fi
mkdir -p "$output_root"
python3 "${workspace}/src/multi_agv_analysis/scripts/analyze_exp2c_v5b_yaw_hold.py" --prepare "$output_root"
echo "EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE"

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
localization_config="${output_root}/configs/localization_v5_quality_observation.yaml"

run_one() {
  local stage="$1" method="$2" candidate="$3"
  local method_id upper_config authorization_config
  if [[ "$method" == M1 ]]; then
    method_id=M1_R1
    upper_config="${output_root}/configs/${candidate}_M1.yaml"
    authorization_config="${output_root}/configs/${candidate}_M1_authorization.yaml"
  else
    method_id=M1b_R1
    upper_config="${output_root}/configs/${candidate}_M1b.yaml"
    authorization_config="${output_root}/configs/${candidate}_M1b_authorization.yaml"
  fi
  local run_dir="${output_root}/${stage}/${method_id}"
  local log_dir="${output_root}/logs/${stage}/${method_id}"
  mkdir -p "$(dirname "$run_dir")" "$log_dir"
  echo "EFFECT_EXPLORATION_ONLY NOT_FORMAL_PAPER_EVIDENCE FAKE: stage=${stage} method=${method_id} candidate=${candidate}"

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
    method_id:="$method_id" experiment_id:=exp2c_v5b_yaw_effectiveness_hold_exploration \
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
    run_id:="$method_id" experiment_id:=exp2c_v5b_yaw_effectiveness_hold_exploration \
    method_id:="$method_id" pair_block_id:="$stage" \
    payload_state:=unloaded localization_source:=odom_fake \
    require_windows_sender_manifest:=false path_config:="$path_config" \
    localization_config:="$localization_config" \
    runtime_config:="$runtime_config" upper_config:="$upper_config" \
    lower_config:="$lower_config" execution_authorization_config:="$authorization_config" \
    evaluation_config:="$evaluation_config" evaluation_target:=5.178229715025710 \
    nominal_common_velocity:=0.10 \
    record_topics_config:="${output_root}/configs/record_topics_v5.yaml" \
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
        echo "STATE_CHAIN_INVALID: required fake pipeline process exited during motion; retaining failure report" >&2
        break 2
      fi
    done
    if grep -Eq "Runtime rigid-fit hard gate rejected|safety abort latched|held fail-zero:|STRUCTURAL_GEOMETRY_INVALID|NUMERICAL_INVALID|STATE_CHAIN_INVALID" "${log_dir}/algorithm.log"; then
      echo "EXPLORATION_HARD_STOP: motion fail-zero or safety latch; retaining evidence" >&2
      break
    fi
    sleep 0.2
  done
  if (( SECONDS >= deadline )); then
    echo "EXP2C-V5B TASK_TIMEOUT: incomplete run retained for full failure report" >&2
  fi
  sleep 1
  rosservice call /experiment_recorder/stop >/dev/null || true
  wait "$recorder_pid" || true
  recorder_pid=""
  cleanup_run

  if ! rosrun multi_agv_analysis process_experiment_run.py \
    "$run_dir" "$validation_config" --skip-plots; then
    echo "EXP2C-V5B: postprocess invalid; retain original bag/CSV/validation for classification" >&2
  fi
  python3 "${workspace}/src/multi_agv_analysis/scripts/analyze_exp2c_v5b_yaw_hold.py" \
    --candidate "$candidate" --check-run "$run_dir" --check-log "${log_dir}/algorithm.log"

}

# Direct exploratory execution: no screen_passed, selected_candidate or formal paired dependency.
if [[ "$method" == pair ]]; then
  run_one "$candidate" M1b "$candidate"
  run_one "$candidate" M1 "$candidate"
  python3 "${workspace}/src/multi_agv_analysis/scripts/analyze_exp2c_v5b_yaw_hold.py" \
    --root "$output_root" --candidate "$candidate"
  exit $?
fi
run_one "$candidate" "$method" "$candidate"
echo "EXPLORATION_RUN_DIR=${output_root}/${candidate}/${method}_R1"
