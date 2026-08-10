#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_single_car_camera_s_closed_loop.sh" \
  --robot-index 2 --chassis-mode remote "$@"
