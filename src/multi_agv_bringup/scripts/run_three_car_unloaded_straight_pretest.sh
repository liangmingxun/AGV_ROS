#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AGV_THREE_CAR_PATH_MODE=straight
exec "${script_dir}/run_three_car_unloaded_pretest.sh" "$@"
