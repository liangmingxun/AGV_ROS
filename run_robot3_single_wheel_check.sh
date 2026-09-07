#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: ./run_robot3_single_wheel_check.sh \
  [--wheel left|right|both] \
  --confirm-wheels-lifted \
  --confirm-emergency-stop-ready

Robot3 only. Starts the chassis with the default 0.08 m/s envelope, records a
bag, checks the selected wheel(s) forward/reverse at 0.01 m/s for 0.5 s each,
repeatedly commands zero, exports CSV, and stops the chassis. Never run with
wheels on the floor.
EOF
}

wheel="both"
confirm_lifted=false
confirm_estop=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --wheel)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      wheel="$2"
      shift 2
      ;;
    --confirm-wheels-lifted)
      confirm_lifted=true
      shift
      ;;
    --confirm-emergency-stop-ready)
      confirm_estop=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done
if [[ "$wheel" != "left" && "$wheel" != "right" && "$wheel" != "both" ]] ||
   [[ "$confirm_lifted" != true || "$confirm_estop" != true ]]; then
  usage
  exit 2
fi
if [[ "$(hostname)" != "robot3" ]]; then
  echo "ERROR: this diagnostic must run on Robot3" >&2
  exit 3
fi

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
source ./setup_robot_ros.sh 3

if rosnode list 2>/dev/null | grep -Fqx /agv3/chassis_controller; then
  echo "ERROR: stop the existing Robot3 chassis terminal before this command" >&2
  exit 4
fi
if timeout 3 rostopic info /agv3/chassis_command 2>/dev/null |
   sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '; then
  echo "ERROR: /agv3/chassis_command already has a publisher" >&2
  exit 4
fi

run_id="$(date +%Y%m%d_%H%M%S)"
run_dir="$workspace/experiment_data/robot3_single_wheel_check/${run_id}_${wheel}"
bag="$run_dir/robot3_single_wheel_check_${run_id}_${wheel}.bag"
mkdir -p "$run_dir"
chassis_pid=""
bag_pid=""

cleanup() {
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  if [[ -n "$chassis_pid" ]] && kill -0 "$chassis_pid" 2>/dev/null; then
    kill -INT "$chassis_pid" 2>/dev/null || true
    wait "$chassis_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

./src/multi_agv_bringup/scripts/start_three_car_chassis.sh 3 \
  >"$run_dir/chassis.log" 2>&1 &
chassis_pid="$!"
deadline=$((SECONDS + 20))
until rosnode list 2>/dev/null | grep -Fqx /agv3/chassis_controller; do
  if ! kill -0 "$chassis_pid" 2>/dev/null; then
    wait "$chassis_pid"
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: Robot3 chassis did not start" >&2
    exit 5
  fi
  sleep 0.2
done

limit="$(rosparam get /agv3/chassis_controller/formal_available_wheel_limit)"
if [[ "$limit" != "0.08" ]]; then
  echo "ERROR: expected the diagnostic 0.08 m/s envelope, got $limit" >&2
  exit 6
fi

rosbag record -O "$bag" \
  /agv3/chassis_command /agv3/chassis_feedback /agv3/capability_report \
  /agv3/odom /agv3/imu \
  >"$run_dir/rosbag_record.log" 2>&1 &
bag_pid="$!"
sleep 2

set +e
python3 src/multi_agv_bringup/scripts/robot3_single_wheel_check.py \
  --output "$run_dir/summary.json" \
  --wheel "$wheel" \
  --confirm-wheels-lifted \
  --confirm-emergency-stop-ready \
  2>&1 | tee "$run_dir/run.log"
check_status="${PIPESTATUS[0]}"
set -e

kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
bag_pid=""
if [[ -s "$bag" ]]; then
  rosbag info "$bag" >"$run_dir/rosbag_info.txt"
  rostopic echo -b "$bag" -p /agv3/chassis_command \
    >"$run_dir/chassis_command.csv"
  rostopic echo -b "$bag" -p /agv3/chassis_feedback \
    >"$run_dir/chassis_feedback.csv"
else
  echo "ERROR: rosbag is missing or empty" >&2
  exit 7
fi

kill -INT "$chassis_pid" 2>/dev/null || true
wait "$chassis_pid" 2>/dev/null || true
chassis_pid=""

echo "Results: $run_dir"
if [[ "$check_status" -eq 0 ]]; then
  echo "PASS: Robot3 lifted single-wheel check completed"
else
  echo "ERROR: Robot3 lifted single-wheel check failed; inspect retained data" >&2
fi
exit "$check_status"
