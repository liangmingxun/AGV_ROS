#!/usr/bin/env bash
set -euo pipefail

confirmed=false
forwarded=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --confirm-x2p3-arc3p0-reduced-margin-path) confirmed=true ;;
    *) forwarded+=("$1") ;;
  esac
  shift
done
if [[ "$confirmed" != true ]]; then
  echo "ERROR: --confirm-x2p3-arc3p0-reduced-margin-path is required" >&2
  echo "This path spans x=2.30 m, y=-0.413..+0.413 m and 3.00 m of arc." >&2
  echo "Robot2 mapped minimum is only about 0.08422 m/s." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M1
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_PATH_CONFIG=src/multi_agv_bringup/config/path_s_curve_x2p3_arc3p0.yaml
export FORMAL_EVALUATION_CONFIG=src/multi_agv_bringup/config/formal_evaluation_3m.yaml
export FORMAL_TARGET_PROGRESS=3.0
export FORMAL_RUN_TIMEOUT_SECONDS=75
export FORMAL_EXPERIMENT_ID=m1_r1_x2p3_arc3p0_serial
export FORMAL_RUN_PREFIX=m1_r1_x2p3_arc3p0
exec "${script_dir}/run_m1_r1_serial_unloaded.sh" "${forwarded[@]}"
