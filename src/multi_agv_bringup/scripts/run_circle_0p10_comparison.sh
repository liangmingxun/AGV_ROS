#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --method M1|M2a|M2b --derating|--derating-0p80|--derating-0p75|--no-derating [experiment confirmations]"
  echo "Shared 0.10 m/s reference, 0.16 m/s wheel capability, R0.7 CW smooth circle."
  echo "Robot2 derating: ratio=0.68, actual progress=1.50..3.50 m, ramps=1 s."
  echo "--derating-0p80 selects the shared three-method condition, ratio=0.80."
  echo "--derating-0p75 selects the shared three-method condition, ratio=0.75."
  echo "--tracking-v2: separate 0.75 paired pilot with shared longitudinal gain 1.3; original configs unchanged."
  echo "--reconciliation-v1: separate 0.75 M1/M2a candidate with slow bounded shared-R1 execution reconciliation."
  echo "--reference-v1: current 0.75/0.80 comparison baseline (also M2b normal). M1/M2a use the corrected shared-R1 acceleration; M2b retains its complete lower controller."
  echo "--formation-observation: M2a 0.80 / M2b normal or 0.80 reference-v1; geometric errors diagnostic, support-pair distance <=0.80 m; requires --confirm-support-range-0p80-clear."
}
method=""; condition=""; tracking_v2=false; reconciliation_v1=false; reference_v1=false; observation=false; range_clear=false; forwarded=()
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
    --reference-v1) reference_v1=true; shift ;;
    --formation-observation) observation=true; shift ;;
    --confirm-support-range-0p80-clear) range_clear=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) forwarded+=("$1"); shift ;;
  esac
done
[[ "$method" =~ ^(M1|M2a|M2b)$ && -n "$condition" ]] || { usage; exit 2; }
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
if [[ "$reference_v1" == true && ( ( "$condition" != derating-0p75 && "$condition" != derating-0p80 && ! ( "$method" == M2b && "$condition" == no-derating ) ) || "$tracking_v2" == true || "$reconciliation_v1" == true ) ]]; then
  echo "ERROR: --reference-v1 requires derating-0p75/0p80 or M2b no-derating, and excludes other candidates" >&2
  exit 2
fi
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
if [[ "$reference_v1" == true ]]; then
  if [[ "$method" == M2b ]]; then
    # reference-v1 corrects the acceleration interface consumed by shared R1.
    # M2b is a complete upper/lower literature method, so it shares the same
    # calibrated platform, startup ramp and sensing chain but must not enable
    # the R1-only consistent_reference_acceleration switch.
    export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_m1_r1_circle_r0p7_smooth_exit_0p10_pilot_runtime.yaml"
    export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_${method}_${scope}_authorization.yaml"
    export FORMAL_EXPERIMENT_ID="${prefix}_circle_r0p7_cw_smooth_exit_0p10_${scope}_reference_v1_platform_pilot"
  else
    export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
    export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_${method}_${scope}_reference_v1_authorization.yaml"
    export FORMAL_EXPERIMENT_ID="${prefix}_circle_r0p7_cw_smooth_exit_0p10_${scope}_reference_v1_pilot"
  fi
  export FORMAL_RUN_PREFIX="$FORMAL_EXPERIMENT_ID"
fi
if [[ "$observation" == true ]]; then
  if [[ ( "$method" != M2a && "$method" != M2b ) || ( "$condition" != derating-0p80 && ! ( "$method" == M2b && "$condition" == no-derating ) ) || "$reference_v1" != true || "$range_clear" != true ]]; then
    echo "ERROR: formation observation requires M2a/M2b --derating-0p80 (or M2b --no-derating), --reference-v1 --confirm-support-range-0p80-clear" >&2
    exit 2
  fi
  export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_${method,,}_circle_0p10_observation_runtime.yaml"
  export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_${method}_observation_authorization.yaml"
  if [[ "$condition" == no-derating ]]; then
    export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$config_dir/formal_circle_0p10_M2b_normal_observation_authorization.yaml"
  fi
  export FORMAL_EXPERIMENT_ID="${prefix}_circle_r0p7_cw_smooth_exit_0p10_${scope}_observation_range0p80_startup_v2_pilot"
  export FORMAL_RUN_PREFIX="$FORMAL_EXPERIMENT_ID"
fi
if [[ "$method" == M2b && "$observation" != true ]]; then
  export FORMAL_EXPERIMENT_ID="${FORMAL_EXPERIMENT_ID}_startup_v2"
  export FORMAL_RUN_PREFIX="$FORMAL_EXPERIMENT_ID"
fi
export FORMAL_RUN_TIMEOUT_SECONDS=140
exec bash "$script_dir/run_m1_r1_serial_unloaded_circle_r0p7_smooth_exit.sh" "${forwarded[@]}"
