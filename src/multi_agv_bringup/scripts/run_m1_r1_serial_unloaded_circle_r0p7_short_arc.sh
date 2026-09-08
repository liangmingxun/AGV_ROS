#!/usr/bin/env bash
set -euo pipefail

confirmed=false
forwarded=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-r0p7-short-arc-footprint-clear) confirmed=true ;;
    *) forwarded+=("$1") ;;
  esac
  shift
done
if [[ "$confirmed" != true ]]; then
  echo "ERROR: --confirm-r0p7-short-arc-footprint-clear is required" >&2
  echo "Clear the straight entry, curvature ramp and first 90-degree clockwise arc." >&2
  echo "Leave additional clearance for the 30 cm formation and vehicle bodies." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M1
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_RUNTIME_CONFIG=src/multi_agv_bringup/config/formal_serial_m1_r1_circle_r0p7_runtime.yaml
export FORMAL_PATH_CONFIG=src/multi_agv_bringup/config/path_circle_r0p7_cw_m1_r1.yaml
export FORMAL_EVALUATION_CONFIG=src/multi_agv_bringup/config/formal_evaluation_circle_r0p7.yaml
export FORMAL_TARGET_PROGRESS=1.699557428756428
export FORMAL_RUN_TIMEOUT_SECONDS=45
export FORMAL_EXPERIMENT_ID=m1_r1_circle_r0p7_cw_short_arc_serial
export FORMAL_RUN_PREFIX=m1_r1_circle_r0p7_cw_short_arc
exec "${script_dir}/run_m1_r1_serial_unloaded.sh" "${forwarded[@]}"
