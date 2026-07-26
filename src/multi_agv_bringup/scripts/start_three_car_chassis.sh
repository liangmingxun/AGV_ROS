#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 <robot-index: 1|2|3>" >&2
}

if [[ $# -ne 1 || ! "$1" =~ ^[123]$ ]]; then
  usage
  exit 2
fi

robot_index="$1"
robot_name="robot${robot_index}"
agv_name="agv${robot_index}"
master_ip="10.134.37.53"
case "$robot_index" in
  1) local_ip="10.134.37.53"; launch_file="car1_master.launch" ;;
  2) local_ip="10.134.37.114"; launch_file="car2_client.launch" ;;
  3) local_ip="10.134.37.239"; launch_file="car3_client.launch" ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
cd "$workspace"

if [[ "$(hostname)" != "$robot_name" ]]; then
  echo "ERROR: this command is for ${robot_name}, current hostname is $(hostname)" >&2
  exit 3
fi
if [[ ! -f devel/setup.bash ]]; then
  echo "ERROR: ${workspace}/devel/setup.bash is missing; compile this checkout first" >&2
  exit 4
fi
working_tree_clean=true
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
  working_tree_clean=false
  echo "WARNING: working tree contains uncommitted changes; continuing by request" >&2
  git status --short >&2
fi
if command -v timedatectl >/dev/null 2>&1 &&
   [[ "$(timedatectl show -p NTPSynchronized --value)" != "yes" ]]; then
  echo "ERROR: system clock is not NTP-synchronised" >&2
  exit 12
fi
if ! ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 |
     grep -Fqx "$local_ip"; then
  echo "ERROR: configured address ${local_ip} is not assigned on this host" >&2
  exit 6
fi

serial_device="$(
  awk '/^serial_device:/ {print $2; exit}' \
    "src/multi_agv_bringup/config/agv${robot_index}_chassis.yaml"
)"
if [[ -z "$serial_device" || ! -e "$serial_device" ]]; then
  echo "ERROR: configured serial device ${serial_device:-<empty>} does not exist" >&2
  echo "Check the STM32 cable and this robot's serial_device configuration." >&2
  exit 7
fi
if [[ ! -r "$serial_device" || ! -w "$serial_device" ]]; then
  echo "ERROR: ${serial_device} is not readable and writable by $(id -un)" >&2
  exit 8
fi
if fuser "$serial_device" >/dev/null 2>&1; then
  echo "ERROR: ${serial_device} is already occupied:" >&2
  fuser -v "$serial_device" >&2 || true
  exit 9
fi

source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  "$local_ip" "$master_ip"

if [[ "$robot_index" != "1" ]] &&
   ! timeout 3 rosnode list >/dev/null 2>&1; then
  echo "ERROR: Robot1 ROS master is unreachable at ${ROS_MASTER_URI}" >&2
  echo "Start Robot1 with this script before Robot2 and Robot3." >&2
  exit 10
fi

git_sha="$(git rev-parse HEAD)"
git_branch="$(git branch --show-current)"
launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

roslaunch multi_agv_bringup "$launch_file" transport_type:=serial &
launch_pid="$!"

deadline=$((SECONDS + 20))
until rosnode list 2>/dev/null | grep -Fqx "/${agv_name}/chassis_controller"; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid"
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: /${agv_name}/chassis_controller did not start within 20 s" >&2
    exit 11
  fi
  sleep 0.2
done

rosparam set "/${agv_name}/deployment/git_sha" "$git_sha"
rosparam set "/${agv_name}/deployment/git_branch" "$git_branch"
rosparam set "/${agv_name}/deployment/hostname" "$(hostname)"
rosparam set "/${agv_name}/deployment/ros_ip" "$local_ip"
rosparam set "/${agv_name}/deployment/clean_worktree" \
  "$working_tree_clean"

echo
echo "${agv_name} READY"
echo "git=${git_sha}"
echo "serial=${serial_device}"
echo "Keep this terminal open. Stop with Ctrl+C only after the experiment."
wait "$launch_pid"
