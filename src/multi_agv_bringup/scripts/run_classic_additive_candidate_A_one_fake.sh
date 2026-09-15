#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
ros_port=11670
method=M1
stage=Smoke
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method) method="$2"; shift 2 ;;
    --stage) stage="$2"; shift 2 ;;
    --output-root) output_root="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    *) echo "ERROR: unknown argument $1" >&2; exit 2 ;;
  esac
done
[[ "$method" == M1 || "$method" == M1b || "$method" == M2b ]] || exit 2
[[ -n "$output_root" ]] || { echo "ERROR: --output-root required" >&2; exit 2; }
[[ "$output_root" == /* ]] || output_root="$workspace/${output_root#./}"
[[ ! -e "$output_root" ]] || { echo "ERROR: fresh output root required" >&2; exit 2; }

cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP || true
mkdir -p "$output_root/logs"
python3 "$workspace/src/multi_agv_analysis/scripts/analyze_classic_additive_candidate_A.py" \
  --prepare "$output_root" --method "$method"

if rosnode list >/dev/null 2>&1; then
  echo "ERROR: ROS master already responds on $ROS_MASTER_URI" >&2; exit 3
fi
roscore -p "$ros_port" >"$output_root/logs/roscore.log" 2>&1 &
roscore_pid=$!
algorithm_pid=""; evaluation_pid=""; recorder_pid=""
cleanup_run() {
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid"; do
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
}
cleanup_all() {
  cleanup_run
  kill -0 "$roscore_pid" 2>/dev/null && kill -INT "$roscore_pid" 2>/dev/null || true
  wait "$roscore_pid" 2>/dev/null || true
}
trap cleanup_all EXIT INT TERM HUP
deadline=$((SECONDS+15))
until rosnode list >/dev/null 2>&1; do
  kill -0 "$roscore_pid" 2>/dev/null || wait "$roscore_pid"
  (( SECONDS < deadline )) || { echo "ERROR: private roscore failed" >&2; exit 4; }
  sleep .2
done

candidate=S1_gamma0p20_down0p6_hold2p5_up0p6_R10
upper_config="$output_root/configs/${candidate}_${method}.yaml"
runtime_config="$(rospack find multi_agv_bringup)/config/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
lower_config="$(rospack find multi_agv_bringup)/config/exp3_R1.yaml"
method_id="${method}_R1"
if [[ "$method" == M2b ]]; then
  upper_config="$output_root/configs/${candidate}_M2b.yaml"
  lower_config="$upper_config"
  runtime_config="$output_root/configs/M2b_runtime.yaml"
  method_id=M2b_M2b
fi
authorization_config="$output_root/configs/${candidate}_${method}_authorization.yaml"
path_config="$(rospack find multi_agv_bringup)/config/path_circle_r0p7_cw_smooth_exit.yaml"
evaluation_config="$(rospack find multi_agv_bringup)/config/formal_evaluation_circle_r0p7_smooth_exit.yaml"
validation_config="$output_root/configs/validation_classic.yaml"
localization_config="$output_root/configs/localization_v5_quality_observation.yaml"

roslaunch multi_agv_bringup formal_fake_algorithm.launch \
  upper_config:="$upper_config" lower_config:="$lower_config" \
  runtime_config:="$runtime_config" path_config:="$path_config" \
  localization_config:="$localization_config" require_recorder_armed:=true \
  command_publication_authorized:=true formal_available_wheel_limit:=0.16 \
  risk_disturbance_enabled:=false yaw_drive_disturbance_enabled:=false \
  >"$output_root/logs/algorithm.log" 2>&1 &
algorithm_pid=$!
roslaunch multi_agv_bringup formal_evaluation_window.launch \
  platform_transport_type:=fake derating_publication_authorized:=false \
  authorization_config:="$authorization_config" evaluation_config:="$evaluation_config" \
  run_id:="${stage}_${method_id}" method_id:="$method_id" \
  experiment_id:=classic_additive_disturbance_candidate_A_validation \
  block_id:="${stage}_${method_id}" >"$output_root/logs/evaluation.log" 2>&1 &
evaluation_pid=$!

deadline=$((SECONDS+20))
until rosnode list 2>/dev/null | grep -Fqx /formal_fake_algorithm; do
  kill -0 "$algorithm_pid" 2>/dev/null || wait "$algorithm_pid"
  (( SECONDS < deadline )) || { echo "ERROR: algorithm startup failed" >&2; exit 5; }
  sleep .2
done
roslaunch multi_agv_bringup experiment.launch arming_authorized:=true \
  output_root:="$output_root" run_id:=run \
  experiment_id:=classic_additive_disturbance_candidate_A_validation \
  method_id:="$method_id" pair_block_id:="$stage" payload_state:=unloaded \
  localization_source:=odom_fake require_windows_sender_manifest:=false \
  path_config:="$path_config" localization_config:="$localization_config" \
  runtime_config:="$runtime_config" upper_config:="$upper_config" lower_config:="$lower_config" \
  execution_authorization_config:="$authorization_config" evaluation_config:="$evaluation_config" \
  evaluation_target:=5.178229715025710 nominal_common_velocity:=0.10 \
  record_topics_config:="$output_root/configs/record_topics_v5.yaml" \
  >"$output_root/logs/recorder.log" 2>&1 &
recorder_pid=$!
deadline=$((SECONDS+25))
until rosservice list 2>/dev/null | grep -Fqx /experiment_recorder/stop; do
  kill -0 "$recorder_pid" 2>/dev/null || wait "$recorder_pid"
  (( SECONDS < deadline )) || { echo "ERROR: recorder startup failed" >&2; exit 6; }
  sleep .2
done

deadline=$((SECONDS+100))
while (( SECONDS < deadline )); do
  progress="$(timeout 2 rostopic echo -n 1 /multi_agv/path_reference 2>/dev/null |
    awk '/load_path_progress_reference:/ {print $2; exit}' || true)"
  if [[ -n "$progress" ]] && awk -v v="$progress" 'BEGIN {exit !(v>=5.178229715025710)}'; then break; fi
  for pid in "$algorithm_pid" "$evaluation_pid" "$recorder_pid"; do
    kill -0 "$pid" 2>/dev/null || { echo "STATE_CHAIN_INVALID: process exited" >&2; break 2; }
  done
  if grep -Eq "NUMERICAL_INVALID|STATE_CHAIN_INVALID|held fail-zero:|safety abort latched" "$output_root/logs/algorithm.log"; then
    echo "INVALID_RUN: hard failure detected" >&2; break
  fi
  sleep .2
done
sleep 1
rosservice call /experiment_recorder/stop >/dev/null || true
wait "$recorder_pid" || true
recorder_pid=""
cleanup_run
algorithm_pid=""; evaluation_pid=""
rosrun multi_agv_analysis process_experiment_run.py "$output_root/run" "$validation_config" --skip-plots || true
python3 "$workspace/src/multi_agv_analysis/scripts/analyze_classic_additive_candidate_A.py" \
  --method "$method" --check-run "$output_root/run" --check-log "$output_root/logs/algorithm.log"
echo "CLASSIC_ADDITIVE_RUN_DIR=$output_root/run"
