#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_robot2_0p15_derating_pretest.sh \
  --confirm-wheels-raised --confirm-emergency-stop-ready

Runs and records the bounded Robot2 raised-wheel 0.15 -> 0.06 -> 0.15 m/s
capability pretest. Robot2's serial chassis must already be running with the
confirmed 0.15 m/s envelope. This script never starts a chassis node.
EOF
}

confirm_raised=false
confirm_stop=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-wheels-raised) confirm_raised=true; shift ;;
    --confirm-emergency-stop-ready) confirm_stop=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ "$confirm_raised" != true || "$confirm_stop" != true ]]; then
  echo "ERROR: both physical safety confirmations are required" >&2
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
cd "$workspace"
if [[ "$(hostname)" != robot2 ]]; then
  echo "ERROR: this raised-wheel pretest must run on Robot2" >&2
  exit 3
fi
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source "${script_dir}/setup_ros_network.sh" 192.168.6.102 192.168.6.101

if ! rosnode list 2>/dev/null | grep -Fqx /agv2/chassis_controller; then
  echo "ERROR: /agv2/chassis_controller is not running" >&2
  exit 4
fi
transport="$(rosparam get /agv2/chassis_controller/transport_type 2>/dev/null || true)"
wheel_limit="$(rosparam get /agv2/chassis_controller/formal_available_wheel_limit 2>/dev/null || true)"
if [[ "$transport" != serial ]]; then
  echo "ERROR: Robot2 chassis transport is ${transport:-missing}, not serial" >&2
  exit 4
fi
if ! awk -v value="$wheel_limit" \
    'BEGIN {exit !(value >= 0.149999 && value <= 0.150001)}'; then
  echo "ERROR: Robot2 wheel capability is ${wheel_limit:-missing}; expected 0.15 m/s" >&2
  exit 4
fi
for node in /formal_algorithm /formal_evaluation_supervisor \
            /robot2_raised_derating_pretest /timed_straight_test; do
  if rosnode list | grep -Fqx "$node"; then
    echo "ERROR: conflicting node is running: ${node}" >&2
    exit 5
  fi
done
existing_derating_publishers="$(rostopic info /agv2/derating_command 2>/dev/null |
  awk '/Publishers:/,/Subscribers:/' | rg '^ \* /' || true)"
if [[ -n "$existing_derating_publishers" ]]; then
  echo "ERROR: /agv2/derating_command already has a publisher" >&2
  echo "$existing_derating_publishers" >&2
  exit 5
fi

stamp="$(date +%Y%m%d_%H%M%S)"
run_id="robot2_0p15_derating_pretest_${stamp}"
result_dir="${workspace}/experiment_data/robot2_raised_derating_pretest/${run_id}"
mkdir -p "$result_dir"
bag_path="${result_dir}/${run_id}.bag"
bag_pid=""
pretest_pid=""
cleanup() {
  for pid in "$pretest_pid" "$bag_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$pretest_pid" "$bag_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM HUP

rosbag record -O "$bag_path" \
  /agv2/chassis_command /agv2/derating_command \
  /agv2/chassis_feedback /agv2/capability_report \
  /agv2/odom /agv2/imu \
  /agv2/raised_derating_pretest/experiment_state &
bag_pid="$!"
sleep 1
kill -0 "$bag_pid" 2>/dev/null || { echo "ERROR: rosbag failed to start" >&2; exit 6; }

roslaunch multi_agv_bringup robot2_raised_derating_pretest.launch \
  platform_transport_type:=serial enable_derating:=true \
  confirm_wheels_raised:=true required:=false &
pretest_pid="$!"
deadline=$((SECONDS + 12))
until rosnode list 2>/dev/null | grep -Fqx /robot2_raised_derating_pretest; do
  kill -0 "$pretest_pid" 2>/dev/null || { wait "$pretest_pid"; exit $?; }
  (( SECONDS < deadline )) || { echo "ERROR: pretest node startup timeout" >&2; exit 6; }
  sleep 0.2
done

last_command_seq="$(timeout 3 rostopic echo -n 1 /agv2/chassis_feedback 2>/dev/null |
  awk '/command_seq_applied:/ {print $2; exit}' || true)"
if [[ ! "$last_command_seq" =~ ^[0-9]+$ ]] ||
   (( last_command_seq > 4294967195 )); then
  echo "ERROR: cannot reserve a safe Robot2 chassis command sequence" >&2
  exit 7
fi
command_seq=$((last_command_seq + 100))

rosrun multi_agv_bringup run_timed_straight_test.py \
  --robot-id 2 --speed 0.05 --duration 14 \
  --command-seq "$command_seq" --experiment-id "$run_id" \
  --required-command-subscribers 2 --confirm-wheels-raised \
  --confirm-extended-test

if wait "$pretest_pid"; then
  pretest_pid=""
else
  result="$?"
  pretest_pid=""
  echo "ERROR: Robot2 0.15 -> 0.06 derating validation failed (result=${result})" >&2
  echo "Results retained: ${result_dir}" >&2
  exit "$result"
fi
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
bag_pid=""
{
  printf '%s\n' 'status=PASSED'
  printf '%s\n' 'nominal_wheel_limit_mps=0.15'
  printf '%s\n' 'derated_wheel_limit_mps=0.06'
  printf '%s\n' 'restored_wheel_limit_mps=0.15'
  printf 'bag=%s\n' "$bag_path"
} >"${result_dir}/result.txt"
echo "ROBOT2_DERATING_PRETEST_STATUS=PASSED"
echo "RESULTS=${result_dir}"
