#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root="${workspace}/experiment_data/formal_fake_observer"
run_id="m1_r1_fake_$(date +%Y%m%d_%H%M%S)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --run-id) run_id="$2"; shift 2 ;;
    -h|--help)
      echo "usage: $0 [--output-root DIR] [--run-id ID]"
      exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; exit 2 ;;
  esac
done

cd "$workspace"
source /opt/ros/noetic/setup.bash
source devel/setup.bash

for topic in /agv1/chassis_command /agv2/chassis_command /agv3/chassis_command; do
  publishers="$(rostopic info "$topic" 2>/dev/null |
    awk '/^Publishers:/{inside=1; next} /^Subscribers:/{inside=0} inside && /^ \* \/.*http:/{print}' || true)"
  if [[ -n "$publishers" ]]; then
    echo "ERROR: command graph is not empty: ${topic}" >&2
    echo "$publishers" >&2
    exit 3
  fi
done

algorithm_pid=""
evaluation_pid=""
recorder_pid=""
cleanup() {
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM HUP

roslaunch multi_agv_bringup formal_fake_algorithm.launch \
  require_recorder_armed:=true &
algorithm_pid="$!"

roslaunch multi_agv_bringup formal_evaluation_window.launch \
  method_id:=M1_R1 &
evaluation_pid="$!"

deadline=$((SECONDS + 20))
until rosnode list 2>/dev/null | grep -Fqx /formal_fake_algorithm; do
  if ! kill -0 "$algorithm_pid" 2>/dev/null; then wait "$algorithm_pid"; fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: fake formal algorithm did not become ready" >&2
    exit 4
  fi
  sleep 0.2
done

roslaunch multi_agv_bringup experiment.launch \
  arming_authorized:=true \
  output_root:="$output_root" \
  run_id:="$run_id" \
  experiment_id:=formal_algorithm_fake_integration \
  method_id:=M1_R1 \
  pair_block_id:=observer \
  payload_state:=unloaded \
  localization_source:=odom_fake &
recorder_pid="$!"

deadline=$((SECONDS + 20))
until rosservice list 2>/dev/null | grep -Fqx /experiment_recorder/stop; do
  if ! kill -0 "$recorder_pid" 2>/dev/null; then wait "$recorder_pid"; fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: experiment recorder did not arm" >&2
    exit 5
  fi
  sleep 0.2
done

deadline=$((SECONDS + 45))
while (( SECONDS < deadline )); do
  progress="$(timeout 2 rostopic echo -n 1 /multi_agv/path_reference 2>/dev/null |
    awk '/load_path_progress_reference:/ {print $2; exit}' || true)"
  if [[ -n "$progress" ]] && awk -v value="$progress" \
      'BEGIN {exit !(value >= 0.999)}'; then
    break
  fi
  sleep 0.2
done
if (( SECONDS >= deadline )); then
  echo "ERROR: fake M1+R1 did not reach the evaluation target" >&2
  exit 6
fi

sleep 1
rosservice call /experiment_recorder/stop >/dev/null
wait "$recorder_pid" || true
recorder_pid=""

run_dir="${output_root}/${run_id}"
rosrun multi_agv_analysis process_experiment_run.py \
  "$run_dir" \
  "$(rospack find multi_agv_analysis)/config/validation_defaults.yaml"
echo "M1+R1 FAKE/OBSERVER COMPLETE: ${run_dir}"
