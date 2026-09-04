#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage:
  ./run_three_car_camera_formation_init.sh \
    --confirm-area-clear \
    --confirm-wheels-on-floor \
    --confirm-automatic-formation

Robot1 remains fixed and defines formation forward. Robot2 then Robot3 move
sequentially at no more than 0.025 m/s into a 0.30 m equilateral triangle of
support centres. This setup step does not record a rosbag; it aborts all cars
on any safety or convergence gate.
EOF
}

confirm_area=false
confirm_floor=false
confirm_auto=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-area-clear) confirm_area=true ;;
    --confirm-wheels-on-floor) confirm_floor=true ;;
    --confirm-automatic-formation) confirm_auto=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
  shift
done
if [[ "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_auto" != true ]]; then
  echo "ERROR: all three physical confirmation flags are required" >&2
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"
if [[ "$(hostname)" != "robot1" ]]; then
  echo "ERROR: run this fleet orchestration command on Robot1" >&2
  exit 3
fi
if [[ ! -f devel/setup.bash ]]; then
  echo "ERROR: devel/setup.bash is missing; build this worktree first" >&2
  exit 4
fi

source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101

nodes="$(rosnode list)"
for required in /agv1/chassis_controller /agv2/chassis_controller \
                /agv3/chassis_controller /pose_provider /camera_odom_fusion; do
  if ! grep -Fqx "$required" <<<"$nodes"; then
    echo "ERROR: required node is not running: ${required}" >&2
    exit 5
  fi
done
for forbidden in /multi_agv_controller /experiment_supervisor \
                 /three_car_unloaded_bounded_pretest \
                 /three_car_camera_formation_init; do
  if grep -Fqx "$forbidden" <<<"$nodes"; then
    echo "ERROR: conflicting command node is running: ${forbidden}" >&2
    exit 6
  fi
done

alive_value="$(timeout 4 rostopic echo -n 1 /vision/aruco/alive 2>/dev/null |
  awk '/data:/ {print $2; exit}' || true)"
if [[ "$alive_value" != "True" ]]; then
  echo "ERROR: Windows ArUco UDP stream is not alive" >&2
  exit 7
fi
for index in 1 2 3; do
  if ! timeout 4 rostopic echo -n 1 \
       "/pose_provider/agv${index}/base_pose_fused" >/dev/null 2>&1; then
    echo "ERROR: no fresh camera-fused pose for agv${index}" >&2
    exit 7
  fi
done

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

rosparam set /multi_agv/formation_init/result_code -1
set +e
roslaunch multi_agv_bringup three_car_camera_formation_init.launch \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  confirm_automatic_formation:=true &
launch_pid="$!"
wait "$launch_pid"
launch_status="$?"
launch_pid=""
set -e

result_code="$(rosparam get /multi_agv/formation_init/result_code 2>/dev/null || true)"

if [[ "$result_code" == "0" ]]; then
  echo "PASS: camera-fused 0.30 m three-car formation initialization confirmed."
  status=0
else
  echo "ERROR: formation initialization aborted (result=${result_code:-missing}, launch=${launch_status})." >&2
  status=1
fi
exit "$status"
