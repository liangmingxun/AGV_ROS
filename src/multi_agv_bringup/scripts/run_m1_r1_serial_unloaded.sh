#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_m1_r1_serial_unloaded.sh \
  --windows-manifest FILE --operator NAME --pair-block ID \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-40cm-fixture

Runs the first paper M1+R1 serial entry on Robot1. The three chassis,
vision bridge and camera fusion must already be running. This command starts
the virtual-load CooperativeState estimator, formal algorithm, evaluation
window and recorder. Any missing recorder heartbeat makes the algorithm
publish zero commands.
EOF
}

windows_manifest=""
operator=""
pair_block=""
confirm_area=false
confirm_floor=false
confirm_fixture=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --windows-manifest) windows_manifest="$2"; shift 2 ;;
    --operator) operator="$2"; shift 2 ;;
    --pair-block) pair_block="$2"; shift 2 ;;
    --confirm-area-clear) confirm_area=true; shift ;;
    --confirm-wheels-on-floor) confirm_floor=true; shift ;;
    --confirm-unloaded-40cm-fixture) confirm_fixture=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ ! -f "$windows_manifest" || -z "$operator" || -z "$pair_block" ||
      "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true ]]; then
  echo "ERROR: manifest, operator, pair block and all confirmations are required" >&2
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

if [[ "$(hostname)" != robot1 ]]; then
  echo "ERROR: formal serial orchestration must run on Robot1" >&2
  exit 3
fi
dirty="$(git status --porcelain --untracked-files=normal |
  awk '$2 !~ /^experiment_data\// {print}' || true)"
if [[ -n "$dirty" ]]; then
  echo "ERROR: formal serial evidence requires a committed, clean worktree" >&2
  echo "$dirty" >&2
  exit 3
fi
local_sha="$(git rev-parse HEAD)"
for index in 1 2 3; do
  node="/agv${index}/chassis_controller"
  if ! rosnode list | grep -Fqx "$node"; then
    echo "ERROR: missing ${node}" >&2; exit 4
  fi
  transport="$(rosparam get "${node}/transport_type" 2>/dev/null || true)"
  deployed_sha="$(rosparam get "/agv${index}/deployment/git_sha" 2>/dev/null || true)"
  if [[ "$transport" != serial || "$deployed_sha" != "$local_sha" ]]; then
    echo "ERROR: agv${index} is not the current serial deployment" >&2
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

estimator_pid=""; algorithm_pid=""; evaluation_pid=""; recorder_pid=""
cleanup() {
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$recorder_pid" "$evaluation_pid" "$algorithm_pid" "$estimator_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM HUP

roslaunch multi_agv_bringup camera_fused_virtual_load_state_estimator.launch &
estimator_pid="$!"
deadline=$((SECONDS + 15))
until rosnode list 2>/dev/null | grep -Fqx /path_state_estimator; do
  if ! kill -0 "$estimator_pid" 2>/dev/null; then wait "$estimator_pid"; fi
  (( SECONDS < deadline )) || { echo "ERROR: state estimator timeout" >&2; exit 6; }
  sleep 0.2
done
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5 --require-fused-cooperative-state

run_id="m1_r1_serial_$(date +%Y%m%d_%H%M%S)"
roslaunch multi_agv_bringup formal_serial_m1_r1.launch \
  enable_commands:=true \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  confirm_unloaded_40cm_fixture:=true &
algorithm_pid="$!"
roslaunch multi_agv_bringup formal_evaluation_window.launch \
  platform_transport_type:=serial \
  run_id:="$run_id" method_id:=M1_R1 \
  experiment_id:=exp2a_m1_r1_unloaded_serial \
  block_id:="$pair_block" &
evaluation_pid="$!"

output_root="${workspace}/experiment_data/formal_serial_unloaded"
roslaunch multi_agv_bringup experiment.launch \
  arming_authorized:=true \
  output_root:="$output_root" run_id:="$run_id" \
  experiment_id:=exp2a_m1_r1_unloaded_serial method_id:=M1_R1 \
  pair_block_id:="$pair_block" payload_state:=unloaded \
  localization_source:=camera_imu_wheel_fused operator:="$operator" \
  camera_mode:=true virtual_load_from_robots:=true \
  windows_sender_manifest:="$windows_manifest" \
  localization_config:="${workspace}/src/multi_agv_bringup/config/localization_camera_three_car_closed_loop.yaml" \
  runtime_config:="${workspace}/src/multi_agv_bringup/config/formal_serial_m1_r1_runtime.yaml" \
  execution_authorization_config:="${workspace}/src/multi_agv_bringup/config/formal_serial_m1_r1_authorization.yaml" \
  nominal_common_velocity:=0.05 evaluation_target:=1.0 &
recorder_pid="$!"

deadline=$((SECONDS + 50))
reached=false
while (( SECONDS < deadline )); do
  for pid in "$algorithm_pid" "$recorder_pid"; do
    if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; fi
  done
  progress="$(timeout 2 rostopic echo -n 1 /multi_agv/path_reference 2>/dev/null |
    awk '/load_path_progress_reference:/ {print $2; exit}' || true)"
  if [[ -n "$progress" ]] && awk -v value="$progress" \
      'BEGIN {exit !(value >= 0.999)}'; then
    reached=true; break
  fi
  sleep 0.2
done
if [[ "$reached" != true ]]; then
  echo "ERROR: M1+R1 did not reach the bounded target" >&2
  exit 7
fi

stable=0
deadline=$((SECONDS + 6))
while (( SECONDS < deadline && stable < 10 )); do
  stopped=true
  for index in 1 2 3; do
    values="$(timeout 2 rostopic echo -n 1 "/agv${index}/chassis_feedback" 2>/dev/null |
      awk '/wheel_linear_velocity_left_actual:/ {l=$2} /wheel_linear_velocity_right_actual:/ {r=$2} END {print l, r}' || true)"
    if ! awk -v values="$values" 'BEGIN {split(values,a," "); l=a[1]+0; r=a[2]+0;
        if (l<0) l=-l; if (r<0) r=-r; exit !(l<=0.01 && r<=0.01)}'; then
      stopped=false
    fi
  done
  if [[ "$stopped" == true ]]; then stable=$((stable + 1)); else stable=0; fi
  sleep 0.05
done
(( stable >= 10 )) || { echo "ERROR: wheel stop was not confirmed" >&2; exit 8; }

rosservice call /experiment_recorder/stop >/dev/null
wait "$recorder_pid" || true
recorder_pid=""
run_dir="${output_root}/${run_id}"
rosrun multi_agv_analysis process_experiment_run.py "$run_dir" \
  "$(rospack find multi_agv_analysis)/config/validation_defaults.yaml"
echo "M1+R1 SERIAL RUN COMPLETE: ${run_dir}"
