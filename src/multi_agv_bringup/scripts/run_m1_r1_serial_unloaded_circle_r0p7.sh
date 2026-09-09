#!/usr/bin/env bash
set -euo pipefail

confirmed=false
forwarded=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-r0p7-circle-footprint-clear) confirmed=true ;;
    *) forwarded+=("$1") ;;
  esac
  shift
done
if [[ "$confirmed" != true ]]; then
  echo "ERROR: --confirm-r0p7-circle-footprint-clear is required" >&2
  echo "Relative load-centre bounds are approximately:" >&2
  echo "  x=-0.301..+1.100 m, y=-1.406..0.000 m" >&2
  echo "Leave additional clearance for the 30 cm formation and vehicle bodies." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE="${FORMAL_UPPER_MODE:-M1}"
export FORMAL_ENABLE_ROBOT2_DERATING="${FORMAL_ENABLE_ROBOT2_DERATING:-false}"
export FORMAL_RUNTIME_CONFIG=src/multi_agv_bringup/config/formal_serial_m1_r1_circle_r0p7_runtime.yaml
export FORMAL_PATH_CONFIG=src/multi_agv_bringup/config/path_circle_r0p7_cw_m1_r1.yaml
export FORMAL_EVALUATION_CONFIG=src/multi_agv_bringup/config/formal_evaluation_circle_r0p7.yaml
export FORMAL_TARGET_PROGRESS=4.998229715025710
export FORMAL_RUN_TIMEOUT_SECONDS=90
export FORMAL_EXPERIMENT_ID="${FORMAL_EXPERIMENT_ID:-m1_r1_circle_r0p7_cw_serial}"
export FORMAL_RUN_PREFIX="${FORMAL_RUN_PREFIX:-m1_r1_circle_r0p7_cw}"
exec "${script_dir}/run_m1_r1_serial_unloaded.sh" "${forwarded[@]}"
