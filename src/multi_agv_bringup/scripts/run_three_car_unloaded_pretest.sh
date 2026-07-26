#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  run_three_car_unloaded_pretest.sh \
    --confirm-area-clear \
    --confirm-wheels-on-floor \
    --confirm-unloaded-40cm-fixture

This command performs topology checks, resets all three odometers, requires
two consecutive valid-state gates, records a bag and runs the bounded
0.05 m/s by 1.00 m unloaded S pretest.
EOF
}

confirm_area=false
confirm_floor=false
confirm_fixture=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-area-clear) confirm_area=true ;;
    --confirm-wheels-on-floor) confirm_floor=true ;;
    --confirm-unloaded-40cm-fixture) confirm_fixture=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
  shift
done
if [[ "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true ]]; then
  echo "ERROR: all three physical confirmation flags are required" >&2
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
cd "$workspace"

if [[ "$(hostname)" != "robot1" ]]; then
  echo "ERROR: this orchestration command must run on Robot1" >&2
  exit 3
fi
if [[ ! -f devel/setup.bash ]]; then
  echo "ERROR: ${workspace}/devel/setup.bash is missing" >&2
  exit 4
fi
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
  echo "WARNING: working tree contains uncommitted changes; continuing by request" >&2
  git status --short >&2
fi

source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  10.134.37.53 10.134.37.53

local_sha="$(git rev-parse HEAD)"
for index in 1 2 3; do
  node="/agv${index}/chassis_controller"
  if ! rosnode list | grep -Fqx "$node"; then
    echo "ERROR: ${node} is not running" >&2
    exit 6
  fi
  deployed_sha="$(rosparam get "/agv${index}/deployment/git_sha" 2>/dev/null || true)"
  deployed_host="$(rosparam get "/agv${index}/deployment/hostname" 2>/dev/null || true)"
  if [[ "$deployed_sha" != "$local_sha" ]]; then
    echo "ERROR: agv${index} Git SHA ${deployed_sha:-<missing>} != ${local_sha}" >&2
    echo "Start every chassis with start_three_car_chassis.sh." >&2
    exit 7
  fi
  if [[ "$deployed_host" != "robot${index}" ]]; then
    echo "ERROR: agv${index} deployment hostname is ${deployed_host:-<missing>}" >&2
    exit 8
  fi
done

for forbidden in /multi_agv_controller /experiment_supervisor \
                 /three_car_unloaded_bounded_pretest /path_state_estimator; do
  if rosnode list | grep -Fqx "$forbidden"; then
    echo "ERROR: conflicting central node is already running: ${forbidden}" >&2
    exit 9
  fi
done

estimator_pid=""
bag_pid=""
motion_pid=""
cleanup() {
  for pid in "$motion_pid" "$estimator_pid" "$bag_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$motion_pid" "$estimator_pid" "$bag_pid"; do
    if [[ -n "$pid" ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM HUP

roslaunch multi_agv_bringup odom_state_estimator.launch &
estimator_pid="$!"
deadline=$((SECONDS + 15))
until rostopic info /multi_agv/cooperative_state 2>/dev/null |
      grep -q '^Publishers:'; do
  if ! kill -0 "$estimator_pid" 2>/dev/null; then
    wait "$estimator_pid"
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: cooperative state estimator did not start" >&2
    exit 10
  fi
  sleep 0.2
done

rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5
for index in 1 2 3; do
  rosservice call "/agv${index}/reset_odometry"
done
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 10 --require-valid-state
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 10 --require-valid-state

kill -INT "$estimator_pid"
wait "$estimator_pid" || true
estimator_pid=""
sleep 1

repo_root="$(realpath "$(git rev-parse --git-common-dir)/..")"
bag_dir="${AGV_BAG_DIR:-${repo_root}}"
mkdir -p "$bag_dir"
run_stamp="$(date +%Y%m%d_%H%M%S)"
run_id="three_car_unloaded_s_1m_${run_stamp}"
bag_path="${bag_dir}/${run_id}.bag"
manifest_path="${bag_dir}/${run_id}_manifest.txt"
params_path="${bag_dir}/${run_id}_params.yaml"

{
  echo "run_id=${run_id}"
  echo "git_sha=${local_sha}"
  echo "git_branch=$(git branch --show-current)"
  echo "robot1_ip=10.134.37.53"
  echo "robot2_ip=10.134.37.114"
  echo "robot3_ip=10.134.37.239"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "fixture=unloaded_equilateral_0.40m"
  echo "path=continuous_S_1.00m"
  echo "speed=0.05m/s"
} > "$manifest_path"
rosparam dump "$params_path"

rosbag record -O "$bag_path" \
  /agv1/chassis_command /agv2/chassis_command /agv3/chassis_command \
  /agv1/chassis_feedback /agv2/chassis_feedback /agv3/chassis_feedback \
  /agv1/capability_report /agv2/capability_report /agv3/capability_report \
  /agv1/odom /agv2/odom /agv3/odom \
  /multi_agv/cooperative_state \
  /multi_agv/bounded_pretest/path_reference \
  /multi_agv/bounded_pretest/controller_state &
bag_pid="$!"
sleep 2
if ! kill -0 "$bag_pid" 2>/dev/null; then
  wait "$bag_pid"
fi

roslaunch multi_agv_bringup three_car_unloaded_bounded_pretest.launch \
  platform_transport_type:=serial \
  enable_commands:=true \
  confirm_readonly_gate_passed:=true \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  confirm_unloaded_40cm_fixture:=true &
motion_pid="$!"
set +e
wait "$motion_pid"
motion_status="$?"
set -e
motion_pid=""

kill -INT "$bag_pid"
wait "$bag_pid" || true
bag_pid=""

echo "finished_at=$(date --iso-8601=seconds)" >> "$manifest_path"
echo "motion_exit_status=${motion_status}" >> "$manifest_path"
echo
if [[ "$motion_status" -eq 0 ]]; then
  echo "THREE-CAR PRETEST COMPLETED"
else
  echo "THREE-CAR PRETEST FAILED OR ABORTED (status=${motion_status})" >&2
fi
echo "bag=${bag_path}"
echo "manifest=${manifest_path}"
echo "params=${params_path}"
exit "$motion_status"
