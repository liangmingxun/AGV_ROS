#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
Usage: ./run_robot3_imu_static_capture.sh \
  --confirm-robot-stationary \
  --confirm-no-motion-command \
  --confirm-emergency-stop-ready
EOF
}

if [[ $# -ne 3 ||
      "$1" != "--confirm-robot-stationary" ||
      "$2" != "--confirm-no-motion-command" ||
      "$3" != "--confirm-emergency-stop-ready" ]]; then
  usage
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${script_dir}"
duration_seconds=120

if [[ "$(hostname)" != "robot3" ]]; then
  echo "ERROR: this stationary capture must run on robot3; current hostname is $(hostname)" >&2
  exit 3
fi

cd "${workspace}"
# shellcheck disable=SC1091
source src/multi_agv_bringup/scripts/setup_local_ros.sh

run_id="$(date +%Y%m%d_%H%M%S)"
run_directory="${workspace}/experiment_data/robot3_imu_calibration/${run_id}"
bag_path="${run_directory}/robot3_imu_static_${run_id}.bag"
mkdir -p "${run_directory}"

master_pid=""
chassis_pid=""
bag_pid=""
cleanup() {
  if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
  if [[ -n "${chassis_pid}" ]] && kill -0 "${chassis_pid}" 2>/dev/null; then
    kill -INT "${chassis_pid}" 2>/dev/null || true
    wait "${chassis_pid}" 2>/dev/null || true
  fi
  if [[ -n "${master_pid}" ]] && kill -0 "${master_pid}" 2>/dev/null; then
    kill -INT "${master_pid}" 2>/dev/null || true
    wait "${master_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

if ! timeout 5 rosparam get /rosversion >/dev/null 2>&1; then
  echo "Local ROS master is absent; starting a private roscore."
  roscore >"${run_directory}/roscore.log" 2>&1 &
  master_pid="$!"
  deadline=$((SECONDS + 10))
  until timeout 1 rosparam get /rosversion >/dev/null 2>&1; do
    if ! kill -0 "${master_pid}" 2>/dev/null; then
      wait "${master_pid}" || true
      echo "ERROR: the local roscore exited during startup" >&2
      exit 4
    fi
    if (( SECONDS >= deadline )); then
      echo "ERROR: the local roscore did not become ready within 10 seconds" >&2
      exit 4
    fi
    sleep 0.2
  done
fi
if ! timeout 5 rosnode list 2>/dev/null | grep -Fxq /agv3/chassis_controller; then
  if [[ ! -e /dev/chassis_driver ]]; then
    echo "ERROR: /dev/chassis_driver is absent; restore the STM32 udev binding" >&2
    exit 5
  fi
  if [[ ! -r /dev/chassis_driver || ! -w /dev/chassis_driver ]]; then
    echo "ERROR: /dev/chassis_driver is not readable and writable by $(id -un)" >&2
    exit 5
  fi
  if fuser /dev/chassis_driver >/dev/null 2>&1; then
    echo "ERROR: /dev/chassis_driver is occupied by another process" >&2
    fuser -v /dev/chassis_driver >&2 || true
    exit 5
  fi
  echo "Robot3 chassis node is absent; starting it with the 0.08 m/s limit."
  roslaunch multi_agv_bringup car3_client.launch \
    transport_type:=serial \
    formal_available_wheel_limit:=0.08 \
    >"${run_directory}/chassis.log" 2>&1 &
  chassis_pid="$!"
  deadline=$((SECONDS + 20))
  until rosnode list 2>/dev/null | grep -Fxq /agv3/chassis_controller; do
    if ! kill -0 "${chassis_pid}" 2>/dev/null; then
      wait "${chassis_pid}" || true
      echo "ERROR: Robot3 chassis node exited during startup" >&2
      echo "Inspect: ${run_directory}/chassis.log" >&2
      exit 5
    fi
    if (( SECONDS >= deadline )); then
      echo "ERROR: Robot3 chassis node did not become ready within 20 seconds" >&2
      exit 5
    fi
    sleep 0.2
  done
fi

has_publisher() {
  timeout 3 rostopic info "$1" 2>/dev/null |
    sed -n '/^Publishers:/,/^Subscribers:/p' | grep -q '^ \* '
}

if has_publisher /agv3/chassis_command; then
  echo "ERROR: /agv3/chassis_command already has a publisher; no capture started" >&2
  rostopic info /agv3/chassis_command >&2 || true
  exit 6
fi

feedback="$(timeout 10 rostopic echo -n 1 /agv3/chassis_feedback 2>/dev/null || true)"
imu_sample="$(timeout 10 rostopic echo -n 1 /agv3/imu 2>/dev/null || true)"
if [[ -z "${feedback}" || -z "${imu_sample}" ]]; then
  echo "ERROR: fresh Robot3 chassis feedback or IMU data is missing" >&2
  exit 7
fi

robot_id="$(awk '/^robot_id:/ {print $2; exit}' <<<"${feedback}")"
left_actual="$(awk '/^wheel_linear_velocity_left_actual:/ {print $2; exit}' <<<"${feedback}")"
right_actual="$(awk '/^wheel_linear_velocity_right_actual:/ {print $2; exit}' <<<"${feedback}")"
if [[ "${robot_id}" != "3" || -z "${left_actual}" || -z "${right_actual}" ]]; then
  echo "ERROR: Robot3 feedback identity or wheel fields are invalid" >&2
  exit 8
fi
python3 - "${left_actual}" "${right_actual}" <<'PY'
import math
import sys

values = [float(value) for value in sys.argv[1:]]
if not all(math.isfinite(value) for value in values):
    raise SystemExit("ERROR: non-finite actual wheel velocity before capture")
if max(abs(value) for value in values) > 0.01:
    raise SystemExit(
        "ERROR: Robot3 is not stationary; actual wheel velocity exceeds 0.01 m/s"
    )
PY

loaded_gyro_bias="$(rosparam get /agv3/chassis_controller/imu/gyro_bias 2>/dev/null || true)"
if [[ -z "${loaded_gyro_bias}" ]]; then
  echo "ERROR: loaded Robot3 gyro_bias is unavailable; restart the chassis node" >&2
  exit 9
fi

{
  echo "run_id=${run_id}"
  echo "robot_id=agv3"
  echo "capture_type=stationary_imu_bias_validation"
  echo "duration_seconds=${duration_seconds}"
  echo "loaded_gyro_bias=${loaded_gyro_bias}"
  echo "roscore_started_by_script=$([[ -n "${master_pid}" ]] && echo true || echo false)"
  echo "chassis_started_by_script=$([[ -n "${chassis_pid}" ]] && echo true || echo false)"
  echo "git_commit=$(git rev-parse HEAD 2>/dev/null || echo unknown)"
} >"${run_directory}/metadata.txt"

echo "Recording Robot3 stationary IMU data for ${duration_seconds} seconds."
echo "Do not touch or move the vehicle."
echo "Bag: ${bag_path}"
rosbag record -O "${bag_path}" \
  /agv3/imu \
  /agv3/chassis_feedback \
  /rosout >"${run_directory}/rosbag.log" 2>&1 &
bag_pid="$!"
capture_start_seconds="${SECONDS}"

sleep 2
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  wait "${bag_pid}" || true
  echo "ERROR: rosbag recorder exited during startup" >&2
  exit 10
fi

for elapsed in 30 60 90 120; do
  remaining=$((elapsed - (SECONDS - capture_start_seconds)))
  if (( remaining > 0 )); then
    sleep "${remaining}"
  fi
  if ! kill -0 "${bag_pid}" 2>/dev/null; then
    wait "${bag_pid}" || true
    echo "ERROR: rosbag recorder exited before ${duration_seconds} seconds" >&2
    exit 11
  fi
  echo "Stationary capture: ${elapsed}/${duration_seconds} s"
done

kill -INT "${bag_pid}" 2>/dev/null || true
wait "${bag_pid}" 2>/dev/null || true
bag_pid=""

if [[ ! -s "${bag_path}" ]]; then
  echo "ERROR: output bag is missing or empty" >&2
  exit 12
fi
rosbag info "${bag_path}" >"${run_directory}/rosbag_info.txt"

python3 - "${bag_path}" >"${run_directory}/imu_static_summary.txt" <<'PY'
import math
import sys

import numpy as np
import rosbag

imu = []
wheels = []
robot_ids = set()
with rosbag.Bag(sys.argv[1]) as bag:
    for topic, message, stamp in bag.read_messages(
        topics=["/agv3/imu", "/agv3/chassis_feedback"]
    ):
        if topic == "/agv3/imu":
            imu.append([
                stamp.to_sec(),
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ])
        else:
            robot_ids.add(int(message.robot_id))
            wheels.append([
                message.wheel_linear_velocity_left_actual,
                message.wheel_linear_velocity_right_actual,
            ])

if len(imu) < 6000 or len(wheels) < 6000:
    raise SystemExit("ERROR: fewer than 60 seconds of IMU or feedback samples")
imu = np.asarray(imu, dtype=float)
wheels = np.asarray(wheels, dtype=float)
if not np.isfinite(imu).all() or not np.isfinite(wheels).all():
    raise SystemExit("ERROR: captured data contains non-finite values")
duration = float(imu[-1, 0] - imu[0, 0])
gyro = imu[:, 1:4]
accel = imu[:, 4:7]
gyro_mean = gyro.mean(axis=0)
gyro_std = gyro.std(axis=0)
drift = gyro_mean * 180.0 / math.pi * 60.0
accel_norm = np.linalg.norm(accel, axis=1)
wheel_peak = np.max(np.abs(wheels), axis=0)

print(f"duration_seconds: {duration:.6f}")
print(f"imu_samples: {len(imu)}")
print(f"feedback_samples: {len(wheels)}")
print(f"robot_ids: {sorted(robot_ids)}")
print("gyro_mean_radps: [{:.9f}, {:.9f}, {:.9f}]".format(*gyro_mean))
print("gyro_std_radps: [{:.9f}, {:.9f}, {:.9f}]".format(*gyro_std))
print("equivalent_drift_deg_per_min: [{:.6f}, {:.6f}, {:.6f}]".format(*drift))
print("accel_mean_mps2: [{:.9f}, {:.9f}, {:.9f}]".format(*accel.mean(axis=0)))
print(f"accel_norm_mean_mps2: {accel_norm.mean():.9f}")
print(f"accel_norm_std_mps2: {accel_norm.std():.9f}")
print("peak_actual_wheel_mps: [{:.9f}, {:.9f}]".format(*wheel_peak))

if robot_ids != {3}:
    raise SystemExit("ERROR: feedback contains an unexpected robot_id")
if duration < 115.0:
    raise SystemExit("ERROR: stationary recording is shorter than 115 seconds")
if float(wheel_peak.max()) > 0.01:
    raise SystemExit("ERROR: wheel motion occurred during the stationary capture")
print("capture_status: VALID_STATIONARY_CAPTURE")
PY

cat "${run_directory}/imu_static_summary.txt"
echo "Results: ${run_directory}"
