#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_m1_lower_comparison_physical.sh PaperNM|PDPenalty \
  [--run-id ID] [--software-only-dry-run] \
  --operator NAME --pair-block ID \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture \
  --confirm-r0p7-smooth-exit-footprint-clear \
  --confirm-emergency-stop-ready --confirm-vision-valid \
  --confirm-recorder-ready --confirm-hardware-authorization \
  --confirm-physical-execution

Runs the frozen normal-condition M1 upper with exactly one external-paper
lower controller. Reusable method authorization is tracked separately from
the controller configuration; every invocation still requires all physical
operator confirmations. No derating or disturbance is enabled.
EOF
}

[[ $# -gt 0 ]] || { usage; exit 2; }
lower_mode="$1"
shift
[[ "$lower_mode" == PaperNM || "$lower_mode" == PDPenalty ]] || {
  echo "ERROR: lower mode must be PaperNM or PDPenalty" >&2
  usage
  exit 2
}

run_id=""; operator=""; pair_block=""; dry_run=false
confirm_area=false; confirm_floor=false; confirm_fixture=false
confirm_footprint=false; confirm_estop=false; confirm_vision=false
confirm_recorder=false; confirm_hardware=false; confirm_execution=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) run_id="$2"; shift 2 ;;
    --operator) operator="$2"; shift 2 ;;
    --pair-block) pair_block="$2"; shift 2 ;;
    --software-only-dry-run) dry_run=true; shift ;;
    --confirm-area-clear) confirm_area=true; shift ;;
    --confirm-wheels-on-floor) confirm_floor=true; shift ;;
    --confirm-unloaded-30cm-fixture) confirm_fixture=true; shift ;;
    --confirm-r0p7-smooth-exit-footprint-clear) confirm_footprint=true; shift ;;
    --confirm-emergency-stop-ready) confirm_estop=true; shift ;;
    --confirm-vision-valid) confirm_vision=true; shift ;;
    --confirm-recorder-ready) confirm_recorder=true; shift ;;
    --confirm-hardware-authorization) confirm_hardware=true; shift ;;
    --confirm-physical-execution) confirm_execution=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ -n "$run_id" && ! "$run_id" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "ERROR: invalid run id: ${run_id}" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
config_dir="src/multi_agv_bringup/config"
case "$lower_mode" in
  PaperNM)
    lower_config="$config_dir/exp3_PaperNM.yaml"
    base_authorization="$config_dir/formal_serial_m1_paper_nm_normal_authorization.yaml"
    method_id="M1_PaperNM"
    ;;
  PDPenalty)
    lower_config="$config_dir/exp3_PDPenalty.yaml"
    base_authorization="$config_dir/formal_serial_m1_pd_penalty_normal_authorization.yaml"
    method_id="M1_PDPenalty"
    ;;
esac

python3 - "$workspace/$lower_config" "$workspace/$base_authorization" \
  "$lower_mode" <<'PY'
import sys
import yaml

lower = yaml.safe_load(open(sys.argv[1]))["formal_lower"]
authorization = yaml.safe_load(open(sys.argv[2]))
assert lower["mode"] == sys.argv[3]
assert lower["algorithm_execution_authorized"] is True
assert lower["golden_vectors_verified"] is True
assert lower["hardware_execution_authorized"] is False
assert lower["adaptation_enabled"] is False
assert lower["disturbance_compensation_enabled"] is False
assert authorization["formal_upper"]["hardware_execution_authorized"] is True
assert authorization["formal_lower"]["hardware_execution_authorized"] is True
assert authorization["authorization_scope"]["authorization_source"] == (
    "explicit_operator_reusable_method_authorization")
assert authorization["authorization_scope"]["lower_config"] == sys.argv[1].split(
    "/.worktrees/platform-foundation-linux/", 1)[-1]
PY

if [[ "$dry_run" == true ]]; then
  echo "STAGE=SOFTWARE_ONLY_DRY_RUN"
  echo "METHOD=${method_id}"
  echo "UPPER=M1"
  echo "LOWER=${lower_mode}"
  echo "DERATING=disabled DISTURBANCE=disabled"
  echo "REUSABLE_METHOD_HARDWARE_AUTHORIZATION=true"
  echo "SERIAL_COMMAND_PUBLISHED=NO"
  exit 0
fi

if [[ -z "$operator" || -z "$pair_block" ||
      "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true || "$confirm_footprint" != true ||
      "$confirm_estop" != true || "$confirm_vision" != true ||
      "$confirm_recorder" != true || "$confirm_hardware" != true ||
      "$confirm_execution" != true ]]; then
  echo "ERROR: operator, pair block and every physical confirmation are required" >&2
  usage
  exit 2
fi

[[ -n "$run_id" ]] || run_id="lower_layer_normal_physical_${lower_mode}_$(date +%Y%m%d_%H%M%S)"

export FORMAL_UPPER_MODE=M1
export FORMAL_UPPER_CONFIG="$config_dir/exp2a_M1_serial_0p10_pilot.yaml"
export FORMAL_LOWER_CONFIG="$lower_config"
export FORMAL_EXPECTED_LOWER_MODE="$lower_mode"
export FORMAL_METHOD_ID="$method_id"
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_m1_r1_circle_r0p7_smooth_exit_0p10_pilot_runtime.yaml"
export FORMAL_PATH_CONFIG="$config_dir/path_circle_r0p7_cw_smooth_exit.yaml"
export FORMAL_PATH_VERSION=circle_r0p7_cw_smooth_exit_v1
export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit.yaml"
export FORMAL_TARGET_PROGRESS=5.178229715025710
export FORMAL_EXECUTION_AUTHORIZATION_BASE_CONFIG="$base_authorization"
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$base_authorization"
export FORMAL_CANDIDATE_B_SPATIAL_CONFIG="$config_dir/formal_serial_candidate_B_spatial_disabled.yaml"
export FORMAL_CANDIDATE_B_SPATIAL_BASE_CONFIG="$config_dir/formal_serial_candidate_B_spatial_disabled.yaml"
export FORMAL_CANDIDATE_B_SPATIAL_PHYSICAL_ENABLED=false
export FORMAL_RECORD_CANDIDATE_B_SPATIAL_PHYSICAL=false
export FORMAL_CLASSIC_ADDITIVE_PHYSICAL_ENABLED=false
export FORMAL_RECORD_CLASSIC_ADDITIVE_PHYSICAL=false
export FORMAL_RISK_DISTURBANCE_ENABLED=false
export FORMAL_EXPERIMENT_ID=lower_layer_normal_physical_comparison
export FORMAL_RUN_PREFIX="lower_layer_normal_physical_${lower_mode}"
export FORMAL_RUN_ID="$run_id"
export FORMAL_RUN_TIMEOUT_SECONDS=85

echo "LOWER_COMPARISON_PHYSICAL_EXECUTION=YES method=${method_id} run_id=${run_id}"
runtime_log="$(mktemp)"
cleanup_log() { rm -f -- "$runtime_log"; }
trap cleanup_log EXIT
set +e
bash "$script_dir/run_m1_r1_serial_unloaded.sh" \
  --operator "$operator" --pair-block "$pair_block" \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture 2>&1 | tee "$runtime_log"
runner_status=${PIPESTATUS[0]}
set -e

run_dir="$workspace/experiment_data/formal_serial_unloaded/$run_id"
if [[ -d "$run_dir" ]]; then
  cp "$runtime_log" "$run_dir/physical_runtime.log"
fi
if [[ "$runner_status" -ne 0 ]]; then
  echo "ERROR: physical runner failed (status=${runner_status}); runtime log retained when a run directory exists" >&2
  exit "$runner_status"
fi
echo "LOWER_COMPARISON_CAPTURE_COMPLETE_REVIEW_REQUIRED=${run_dir}"
