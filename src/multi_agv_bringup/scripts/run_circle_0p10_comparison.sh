#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --method M1|M2a|M2b --derating|--derating-0p80|--derating-0p75|--no-derating [experiment confirmations]"
  echo "Shared 0.10 m/s reference, 0.16 m/s wheel capability, R0.7 CW smooth circle."
  echo "Robot2 derating: ratio=0.68, actual progress=1.50..3.50 m, ramps=1 s."
  echo "--derating-0p80 selects the separate M1/M2a paired pilot, ratio=0.80."
  echo "--derating-0p75 selects the separate M1/M2a paired pilot, ratio=0.75."
  echo "--tracking-v2: separate 0.75 paired pilot with shared longitudinal gain 1.3; original configs unchanged."
  echo "--reconciliation-v1: separate 0.75 M1/M2a candidate with slow bounded shared-R1 execution reconciliation."
}
method=""; condition=""; tracking_v2=false; reconciliation_v1=false; forwarded=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method)
      [[ $# -ge 2 && -z "$method" ]] || { usage; exit 2; }
      method="$2"; shift 2 ;;
    --derating|--derating-0p80|--derating-0p75|--no-derating)
      [[ -z "$condition" ]] || { echo "ERROR: select only one derating condition" >&2; exit 2; }
      condition="${1#--}"; shift ;;
    --tracking-v2) tracking_v2=true; shift ;;
    --reconciliation-v1) reconciliation_v1=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) forwarded+=("$1"); shift ;;
  esac
done
[[ "$method" =~ ^(M1|M2a|M2b)$ && -n "$condition" ]] || { usage; exit 2; }
if [[ ( "$condition" == derating-0p80 || "$condition" == derating-0p75 ) && "$method" == M2b ]]; then
  echo "ERROR: this paired pilot is scoped to M1 and M2a only" >&2
  exit 2
fi
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$tracking_v2" == true && ( "$condition" != derating-0p75 || "$method" == M2b ) ]]; then
  echo "ERROR: --tracking-v2 is scoped to the M1/M2a derating-0p75 paired pilot" >&2
  exit 2
fi
if [[ "$reconciliation_v1" == true && ( "$condition" != derating-0p75 || "$method" == M2b ) ]]; then
  echo "ERROR: --reconciliation-v1 is scoped to the M1/M2a derating-0p75 paired pilot" >&2
  exit 2
fi
if [[ "$tracking_v2" == true && "$reconciliation_v1" == true ]]; then
  echo "ERROR: --tracking-v2 and --reconciliation-v1 are mutually exclusive" >&2
  exit 2
fi
config_dir=src/multi_agv_bringup/config
export FORMAL_UPPER_MODE="$method"
case "$method" in
  M1) export FORMAL_UPPER_CONFIG="$config_dir/exp2a_M1_serial_0p10_pilot.yaml"; prefix=m1_r1 ;;
  M2a) export FORMAL_UPPER_CONFIG="$config_dir/exp2a_M2a_serial_0p10_pilot.yaml"; prefix=m2a_r1 ;;
  M2b) unset FORMAL_UPPER_CONFIG; prefix=m2b ;;
esac
export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_m1_r1_circle_r0p7_smooth_exit_0p10_pilot_runtime.yaml"
if [[ "$condition" == derating-0p80 || "$condition" == derating-0p75 ]]; then
  ratio_tag="${condition#derating-}"
  export FORMAL_ENABLE_ROBOT2_DERATING=true
  export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit_derating_${ratio_tag}_pilot.yaml"
  scope="derating_${ratio_tag}"
elif [[ "$condition" == derating ]]; then
  export FORMAL_ENABLE_ROBOT2_DERATING=true
  export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit_derating.yaml"
  scope=derating
else
  export FORMAL_ENABLE_ROBOT2_DERATING=false
  export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit.yaml"
  scope=normal
fi
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_${method}_${scope}_authorization.yaml"
export FORMAL_DERATING_AUTHORIZATION_CONFIG="$config_dir/formal_exp2a_derating_authorization.yaml"
if [[ "$condition" == derating-0p80 || "$condition" == derating-0p75 ]]; then
  export FORMAL_DERATING_AUTHORIZATION_CONFIG="$config_dir/formal_exp2a_derating_${ratio_tag}_pilot_authorization.yaml"
fi
export FORMAL_EXPERIMENT_ID="${prefix}_circle_r0p7_cw_smooth_exit_0p10_${scope}_pilot"
if [[ "$tracking_v2" == true ]]; then
  export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_circle_r0p7_smooth_exit_0p10_tracking_v2_pilot_runtime.yaml"
  export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_${method}_derating_0p75_tracking_v2_authorization.yaml"
  export FORMAL_EXPERIMENT_ID="${prefix}_circle_r0p7_cw_smooth_exit_0p10_${scope}_tracking_v2_pilot"
fi
if [[ "$reconciliation_v1" == true ]]; then
  export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_circle_r0p7_smooth_exit_0p10_reconciliation_v1_pilot_runtime.yaml"
  export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_${method}_derating_0p75_reconciliation_v1_authorization.yaml"
  export FORMAL_EXPERIMENT_ID="${prefix}_circle_r0p7_cw_smooth_exit_0p10_${scope}_reconciliation_v1_pilot"
fi
export FORMAL_RUN_PREFIX="$FORMAL_EXPERIMENT_ID"
export FORMAL_RUN_TIMEOUT_SECONDS=140
exec "$script_dir/run_m1_r1_serial_unloaded_circle_r0p7_smooth_exit.sh" "${forwarded[@]}"
