#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FORMAL_UPPER_MODE=M1
export FORMAL_ENABLE_ROBOT2_DERATING=true
export FORMAL_EVALUATION_CONFIG=src/multi_agv_bringup/config/formal_evaluation_circle_r0p7_smooth_exit_derating.yaml
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG=src/multi_agv_bringup/config/formal_serial_m1_r1_circle_r0p7_smooth_exit_derating_authorization.yaml
export FORMAL_DERATING_AUTHORIZATION_CONFIG=src/multi_agv_bringup/config/formal_exp2a_derating_authorization.yaml
export FORMAL_RUN_TIMEOUT_SECONDS=140
export FORMAL_EXPERIMENT_ID=m1_r1_circle_r0p7_cw_smooth_exit_robot2_derating_serial
export FORMAL_RUN_PREFIX=m1_r1_circle_r0p7_cw_smooth_exit_derating
exec "${script_dir}/run_m1_r1_serial_unloaded_circle_r0p7_smooth_exit.sh" "$@"
