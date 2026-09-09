#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${WORKSPACE}/src/multi_agv_bringup/scripts/run_m2b_serial_unloaded_circle_r0p7_short_arc.sh" "$@"
