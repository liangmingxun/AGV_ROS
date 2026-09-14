#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_circle_0p10_risk_comparison.sh \
  --method M1|M1b \
  (--risk-disturbance-v1 --disturbance-level 0p30|0p45|0p60|0p70 | --no-disturbance) \
  --operator NAME --pair-block ID --confirm-area-clear \
  --confirm-wheels-on-floor --confirm-unloaded-30cm-fixture

The authorization overlay is intentionally locked until manual physical review.
EOF
}

method=""; level=""; disturbance=""; forwarded=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method) [[ $# -ge 2 && -z "$method" ]] || { usage; exit 2; }; method="$2"; shift 2 ;;
    --risk-disturbance-v1) [[ -z "$disturbance" ]] || { usage; exit 2; }; disturbance=true; shift ;;
    --no-disturbance) [[ -z "$disturbance" ]] || { usage; exit 2; }; disturbance=false; shift ;;
    --disturbance-level) [[ $# -ge 2 && -z "$level" ]] || { usage; exit 2; }; level="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) forwarded+=("$1"); shift ;;
  esac
done
[[ "$method" == M1 || "$method" == M1b ]] || { usage; exit 2; }
[[ -n "$disturbance" ]] || { usage; exit 2; }
if [[ "$disturbance" == true ]]; then
  [[ "$level" =~ ^(0p30|0p45|0p60|0p70)$ ]] || { usage; exit 2; }
else
  [[ -z "$level" ]] || { echo "ERROR: --no-disturbance excludes --disturbance-level" >&2; exit 2; }
  level=0p00
fi

config_dir=src/multi_agv_bringup/config
export FORMAL_UPPER_MODE="$method"
export FORMAL_UPPER_CONFIG="$config_dir/exp2c_${method}_risk_disturbance_v1.yaml"
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_exp2c_${method}_risk_disturbance_authorization.yaml"
export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit.yaml"
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_RISK_DISTURBANCE_ENABLED="$disturbance"
export FORMAL_RISK_DISTURBANCE_PEAK_FRACTION="${level/0p/0.}"
method_tag="${method,,}_r1"
export FORMAL_EXPERIMENT_ID="${method_tag}_circle_r0p7_risk_disturbance_v1_${level}_pilot"
export FORMAL_RUN_PREFIX="$FORMAL_EXPERIMENT_ID"
export FORMAL_RUN_TIMEOUT_SECONDS=140

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/run_m1_r1_serial_unloaded_circle_r0p7_smooth_exit.sh" "${forwarded[@]}"
