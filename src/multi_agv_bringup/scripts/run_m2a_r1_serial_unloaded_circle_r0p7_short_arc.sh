#!/usr/bin/env bash
set -euo pipefail

# Reuse the exact M1 short-arc path and planar execution runtime. The separate
# M2a hardware-authorization overlay remains fail-closed by default.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M2a
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_EXPERIMENT_ID=m2a_r1_circle_r0p7_cw_short_arc_serial
export FORMAL_RUN_PREFIX=m2a_r1_circle_r0p7_cw_short_arc
exec "${script_dir}/run_m1_r1_serial_unloaded_circle_r0p7_short_arc.sh" "$@"
