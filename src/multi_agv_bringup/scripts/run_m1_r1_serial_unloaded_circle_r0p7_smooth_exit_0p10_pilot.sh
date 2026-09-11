#!/usr/bin/env bash
set -euo pipefail

# Select only the no-derating 0.10 m/s pilot; the frozen 0.08 m/s entry is
# intentionally left unchanged.
export FORMAL_UPPER_MODE=M1
export FORMAL_UPPER_CONFIG=src/multi_agv_bringup/config/exp2a_M1_serial_0p10_pilot.yaml
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_RUNTIME_CONFIG=src/multi_agv_bringup/config/formal_serial_m1_r1_circle_r0p7_smooth_exit_0p10_pilot_runtime.yaml
export FORMAL_PATH_CONFIG=src/multi_agv_bringup/config/path_circle_r0p7_cw_smooth_exit.yaml
export FORMAL_PATH_VERSION=circle_r0p7_cw_smooth_exit_v1
export FORMAL_EVALUATION_CONFIG=src/multi_agv_bringup/config/formal_evaluation_circle_r0p7_smooth_exit.yaml
export FORMAL_TARGET_PROGRESS=5.178229715025710
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG=src/multi_agv_bringup/config/formal_serial_m1_r1_circle_r0p7_smooth_exit_0p10_pilot_authorization.yaml
export FORMAL_EXPERIMENT_ID=m1_r1_circle_r0p7_cw_smooth_exit_0p10_no_derating_pilot
export FORMAL_RUN_PREFIX=m1_r1_circle_r0p7_cw_smooth_exit_0p10_no_derating_pilot
export FORMAL_RUN_TIMEOUT_SECONDS=85

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${script_dir}/run_m1_r1_serial_unloaded_circle_r0p7_smooth_exit.sh" "$@"
