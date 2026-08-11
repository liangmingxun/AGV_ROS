#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AGV_THREE_CAR_PATH_MODE=s
exec "${script_dir}/src/multi_agv_bringup/scripts/run_three_car_unloaded_pretest.sh" "$@"
