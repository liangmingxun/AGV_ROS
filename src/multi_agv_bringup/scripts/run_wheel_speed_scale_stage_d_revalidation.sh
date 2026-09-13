#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_wheel_speed_scale_stage_d_revalidation.sh \
    --robot-id 1|2|3 \
    --operator NAME \
    --confirm-area-clear \
    --confirm-wheels-on-floor \
    --confirm-emergency-stop-ready \
    --confirm-camera-physical-truth \
    --confirm-stage-d-production-mapping

Runs two forward/reverse repetitions at 0.06, 0.08 and 0.10 m/s, records a
bag, and automatically validates the installed command and feedback scales.
It never modifies production parameters.
EOF
}

robot_id=""
operator=""
confirm_area=0
confirm_floor=0
confirm_estop=0
confirm_camera=0
confirm_mapping=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id) robot_id="${2:-}"; shift 2 ;;
    --operator) operator="${2:-}"; shift 2 ;;
    --confirm-area-clear) confirm_area=1; shift ;;
    --confirm-wheels-on-floor) confirm_floor=1; shift ;;
    --confirm-emergency-stop-ready) confirm_estop=1; shift ;;
    --confirm-camera-physical-truth) confirm_camera=1; shift ;;
    --confirm-stage-d-production-mapping) confirm_mapping=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! "$robot_id" =~ ^[123]$ || -z "$operator" ]]; then
  echo "ERROR: --robot-id 1|2|3 and --operator are required." >&2
  exit 2
fi
if (( confirm_area != 1 || confirm_floor != 1 || confirm_estop != 1 ||
      confirm_camera != 1 || confirm_mapping != 1 )); then
  echo "ERROR: all five physical/revalidation confirmations are required." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
session_root="${workspace}/experiment_data/wheel_speed_scale_revalidation/stage_d_agv${robot_id}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$session_root"

"${script_dir}/run_wheel_speed_scale_calibration.sh" \
  --robot-id "$robot_id" \
  --operator "$operator" \
  --output-root "$session_root" \
  --speeds 0.06,0.08,0.10 \
  --repetitions 2 \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-emergency-stop-ready \
  --confirm-camera-physical-truth \
  --confirm-bounded-scale-excitation

mapfile -t run_dirs < <(find "$session_root" -mindepth 1 -maxdepth 1 -type d)
if [[ "${#run_dirs[@]}" -ne 1 ]]; then
  echo "ERROR: expected exactly one acquired run under ${session_root}." >&2
  exit 5
fi

source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"
rosrun multi_agv_bringup validate_wheel_speed_scale_revalidation.py \
  "${run_dirs[0]}"

echo "STAGE D WHEEL SCALE REVALIDATION COMPLETE: ${run_dirs[0]}"
