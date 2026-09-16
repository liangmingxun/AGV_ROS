#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_candidate_B_spatial_gradient15_physical.sh \
  --method M1|M1b|M2b [--run-id ID] --operator NAME --pair-block ID \
  [--software-only-dry-run] \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture --confirm-emergency-stop-ready \
  --confirm-vision-valid --confirm-recorder-ready \
  --confirm-hardware-authorization --confirm-physical-execution

This is one full-strength run of the frozen Candidate B spatial gradient15
profile.  It has no scale ladder.  The software-only dry run reads and checks
the frozen profile and exits before sourcing ROS or contacting a chassis.
EOF
}

method=""; run_id=""; operator=""; pair_block=""; dry_run=false
confirm_area=false; confirm_floor=false; confirm_fixture=false
confirm_estop=false; confirm_vision=false; confirm_recorder=false
confirm_hardware=false; confirm_execution=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method) method="$2"; shift 2 ;;
    --run-id) run_id="$2"; shift 2 ;;
    --operator) operator="$2"; shift 2 ;;
    --pair-block) pair_block="$2"; shift 2 ;;
    --software-only-dry-run) dry_run=true; shift ;;
    --confirm-area-clear) confirm_area=true; shift ;;
    --confirm-wheels-on-floor) confirm_floor=true; shift ;;
    --confirm-unloaded-30cm-fixture) confirm_fixture=true; shift ;;
    --confirm-emergency-stop-ready) confirm_estop=true; shift ;;
    --confirm-vision-valid) confirm_vision=true; shift ;;
    --confirm-recorder-ready) confirm_recorder=true; shift ;;
    --confirm-hardware-authorization) confirm_hardware=true; shift ;;
    --confirm-physical-execution) confirm_execution=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ "$method" == M1 || "$method" == M1b || "$method" == M2b ]] || {
  echo "ERROR: Candidate B gradient15 permits only M1, M1b or M2b" >&2
  exit 2
}
if [[ -n "$run_id" && ! "$run_id" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: invalid run id: ${run_id}" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
config_dir="src/multi_agv_bringup/config"
physical_config="$config_dir/formal_serial_candidate_B_spatial_gradient15.yaml"
authorization="$config_dir/formal_serial_candidate_B_spatial_gradient15_${method}_authorization.yaml"

read_config_value() {
  python3 - "$workspace/$1" "$2" <<'PY'
import sys, yaml
value = yaml.safe_load(open(sys.argv[1]))
for key in sys.argv[2].split('.'):
    value = value[key]
print(str(value).lower() if isinstance(value, bool) else value)
PY
}

python3 - "$workspace/$physical_config" <<'PY'
import sys, yaml
runtime = yaml.safe_load(open(sys.argv[1]))["formal_fake_runtime"]
p = runtime["candidate_b_spatial_composite"]
expected = {
    "enabled": False, "minimum_effectiveness": .85,
    "longitudinal_amplitude": .020, "longitudinal_frequency": 1.0,
    "longitudinal_phase": 0.0, "yaw_amplitude": .2625,
    "yaw_frequency": 1.0, "yaw_phase": 1.5707963267948966,
    "zone_start": 2.0, "zone_end": 2.8,
    "ramp_in_distance": .05, "ramp_out_distance": .05,
    "severity_scale": [1.0, 1.15, .85],
    "disturbance_model_version": "candidate_B_v2_spatial_composite",
    "freeze_id": "candidate_B_v2_spatial_gradient15_rho0p85_av0p020_aw0p2625",
}
assert p == expected, "physical Candidate B profile differs from freeze"
assert not runtime["candidate_b"]["enabled"]
assert not runtime["classic_additive_candidate_A"]["enabled"]
PY

if [[ "$dry_run" == true ]]; then
  echo "STAGE=SOFTWARE_ONLY_DRY_RUN"
  echo "METHOD=${method}"
  echo "PROFILE=candidate_B_v2_spatial_gradient15_rho0p85_av0p020_aw0p2625"
  echo "SEVERITY_SCALE=1.00,1.15,0.85"
  echo "RHO_BASE=0.85 Av_BASE=0.020 Aomega_BASE=0.2625"
  echo "SERIAL_COMMAND_PUBLISHED=NO"
  exit 0
fi

if [[ -z "$operator" || -z "$pair_block" ||
      "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true || "$confirm_estop" != true ||
      "$confirm_vision" != true || "$confirm_recorder" != true ||
      "$confirm_hardware" != true || "$confirm_execution" != true ]]; then
  echo "ERROR: operator, pair block and every physical confirmation are required" >&2
  usage
  exit 2
fi

physical_enabled="$(read_config_value "$physical_config" formal_fake_runtime.candidate_b_spatial_physical.enabled)"
hardware_authorized="$(read_config_value "$physical_config" formal_fake_runtime.candidate_b_spatial_physical.hardware_execution_authorized)"
upper_authorized="$(read_config_value "$authorization" formal_upper.hardware_execution_authorized)"
lower_authorized="$(read_config_value "$authorization" formal_lower.hardware_execution_authorized)"
if [[ "$physical_enabled" != true || "$hardware_authorized" != true ]]; then
  echo "ERROR: Candidate B gradient15 physical execution remains fail-closed in ${physical_config}" >&2
  exit 3
fi
if [[ "$upper_authorized" != true || "$lower_authorized" != true ]]; then
  echo "ERROR: ${method} method authorization remains fail-closed in ${authorization}" >&2
  exit 3
fi
if [[ "$method" == M1b ]]; then
  m1b_authorized="$(read_config_value "$authorization" formal_upper.m1b_hardware_execution_authorized)"
  [[ "$m1b_authorized" == true ]] || {
    echo "ERROR: M1b-specific hardware authorization remains false" >&2
    exit 3
  }
fi

case "$method" in
  M1) upper_config="$config_dir/exp2a_M1_serial_0p10_pilot.yaml" ;;
  M1b) upper_config="$config_dir/exp2c_M1b_risk_disturbance_v1.yaml" ;;
  M2b) upper_config="$config_dir/exp2b_M2b.yaml" ;;
esac
[[ -n "$run_id" ]] || run_id="candidate_B_spatial_gradient15_physical_${method}_$(date +%Y%m%d_%H%M%S)"

export FORMAL_UPPER_MODE="$method"
export FORMAL_UPPER_CONFIG="$upper_config"
export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
export FORMAL_PATH_CONFIG="$config_dir/path_circle_r0p7_cw_smooth_exit.yaml"
export FORMAL_PATH_VERSION="circle_r0p7_cw_smooth_exit_v1"
export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit.yaml"
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$authorization"
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_CANDIDATE_B_SPATIAL_CONFIG="$physical_config"
export FORMAL_CANDIDATE_B_SPATIAL_PHYSICAL_ENABLED=true
export FORMAL_RECORD_CANDIDATE_B_SPATIAL_PHYSICAL=true
export FORMAL_CANDIDATE_B_SPATIAL_PROFILE_ID="candidate_B_v2_spatial_gradient15_rho0p85_av0p020_aw0p2625"
export FORMAL_CANDIDATE_B_SPATIAL_PHYSICAL_AUTHORIZED=true
export FORMAL_EXPERIMENT_ID="candidate_B_spatial_gradient15_physical_validation"
export FORMAL_RUN_PREFIX="candidate_B_spatial_gradient15_physical_${method}"
export FORMAL_RUN_ID="$run_id"
export FORMAL_RUN_TIMEOUT_SECONDS=140

echo "CANDIDATE_B_GRADIENT15_PHYSICAL_EXECUTION=YES method=${method} run_id=${run_id}"
bash "$script_dir/run_m1_r1_serial_unloaded.sh" \
  --operator "$operator" --pair-block "$pair_block" \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture

run_dir="$workspace/experiment_data/formal_serial_unloaded/$run_id"
python3 "$workspace/src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_gradient15_physical.py" \
  --analyze-run "$run_dir" --method "$method"
echo "CANDIDATE_B_GRADIENT15_PHYSICAL_CAPTURE_COMPLETE_REVIEW_REQUIRED=${run_dir}"
