#!/usr/bin/env bash
set -euo pipefail

# Reuse the exact M1 circle path and planar execution runtime. Only the formal
# upper configuration/method identity changes; Robot2 derating stays off in
# this unloaded method pilot.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M2a
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_EXPERIMENT_ID=m2a_r1_circle_r0p7_cw_serial
export FORMAL_RUN_PREFIX=m2a_r1_circle_r0p7_cw
exec "${script_dir}/run_m1_r1_serial_unloaded_circle_r0p7.sh" "$@"
