#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SINGLE_CAR_TEST_PROFILE=circle_r0p7_cw
export SINGLE_CAR_CONFIG_FILE="${SCRIPT_DIR}/src/multi_agv_bringup/config/single_car_circle_r0p7_cw_calibration.yaml"
exec "${SCRIPT_DIR}/run_single_car_camera_s_closed_loop.sh" \
  --robot-index 2 --chassis-mode remote "$@"
