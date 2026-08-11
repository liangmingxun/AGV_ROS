#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# Robot1 owns its chassis serial port locally. Unlike Robot2/3, start the
# chassis only for this bounded calibration and always stop it on exit.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/setup_robot_ros.sh" 1

has_publisher() {
  timeout 3 rostopic info "$1" 2>/dev/null |
    sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '
}

if has_publisher /agv1/chassis_feedback; then
  echo "ERROR: Robot1 chassis is already running; stop its launch terminal first." >&2
  exit 2
fi

chassis_pid=""
cleanup() {
  if [[ -n "${chassis_pid}" ]] && kill -0 "${chassis_pid}" 2>/dev/null; then
    kill -INT "${chassis_pid}" 2>/dev/null || true
    wait "${chassis_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

echo "Starting the Robot1 local chassis for bounded camera calibration."
roslaunch multi_agv_bringup car1_master.launch transport_type:=serial &
chassis_pid="$!"

deadline=$((SECONDS + 20))
until has_publisher /agv1/chassis_feedback; do
  if ! kill -0 "${chassis_pid}" 2>/dev/null; then
    wait "${chassis_pid}"
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: Robot1 chassis feedback did not start within 20 seconds." >&2
    exit 3
  fi
  sleep 0.2
done

set +e
"${SCRIPT_DIR}/run_single_car_camera_wheel_calibration.sh" \
  --robot-index 1 "$@"
calibration_status="$?"
set -e

cleanup
trap - EXIT INT TERM HUP
exit "${calibration_status}"
