#!/usr/bin/env bash
set -euo pipefail

# M2b keeps its complete upper/lower equations while reusing the exact M1
# circle path, planar execution adapter, chassis limiter and recorder chain.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M2b
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_EXPERIMENT_ID=m2b_circle_r0p7_cw_serial
export FORMAL_RUN_PREFIX=m2b_circle_r0p7_cw
exec "${script_dir}/run_m1_r1_serial_unloaded_circle_r0p7.sh" "$@"
