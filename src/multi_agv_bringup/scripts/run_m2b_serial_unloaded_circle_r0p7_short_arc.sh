#!/usr/bin/env bash
set -euo pipefail

# M2b keeps its complete upper/lower equations while reusing the exact M1
# short-arc path and planar execution runtime. Its hardware gate remains off.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M2b
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_EXPERIMENT_ID=m2b_circle_r0p7_cw_short_arc_serial
export FORMAL_RUN_PREFIX=m2b_circle_r0p7_cw_short_arc
exec "${script_dir}/run_m1_r1_serial_unloaded_circle_r0p7_short_arc.sh" "$@"
