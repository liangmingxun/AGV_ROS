#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M1
export FORMAL_ENABLE_ROBOT2_DERATING=true
exec "${script_dir}/run_m1_r1_serial_unloaded.sh" "$@"
