#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_classic_additive_candidate_A_physical_commissioning.sh \
  --method M1|M1b --scale 0|0.25|0.50|0.75|1.00 \
  [--software-only-dry-run] \
  --operator NAME --pair-block ID \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture --confirm-vision-valid \
  --confirm-recorder-ready --confirm-hardware-authorization \
  --confirm-physical-execution

Scale 0 is the disturbance-disabled P1 baseline. Nonzero scales are one
manually started qualification run each. The script never chains levels.
--software-only-dry-run performs P0 parameter/mapping inspection and exits
before sourcing ROS or contacting a chassis.
EOF
}

method=""; scale=""; operator=""; pair_block=""; dry_run=false
confirm_area=false; confirm_floor=false; confirm_fixture=false
confirm_vision=false; confirm_recorder=false; confirm_hardware=false
confirm_execution=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method) method="$2"; shift 2 ;;
    --scale) scale="$2"; shift 2 ;;
    --software-only-dry-run) dry_run=true; shift ;;
    --operator) operator="$2"; shift 2 ;;
    --pair-block) pair_block="$2"; shift 2 ;;
    --confirm-area-clear) confirm_area=true; shift ;;
    --confirm-wheels-on-floor) confirm_floor=true; shift ;;
    --confirm-unloaded-30cm-fixture) confirm_fixture=true; shift ;;
    --confirm-vision-valid) confirm_vision=true; shift ;;
    --confirm-recorder-ready) confirm_recorder=true; shift ;;
    --confirm-hardware-authorization) confirm_hardware=true; shift ;;
    --confirm-physical-execution) confirm_execution=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ "$method" == M1 || "$method" == M1b ]] || {
  echo "ERROR: physical Candidate A permits only M1 or M1b" >&2; exit 2; }
case "$scale" in
  0|0.25|0.50|0.75|1.00) ;;
  *) echo "ERROR: scale must be 0, 0.25, 0.50, 0.75 or 1.00" >&2; exit 2 ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
config_dir="src/multi_agv_bringup/config"
physical_config="$config_dir/formal_serial_classic_additive_candidate_A_commissioning.yaml"

if [[ "$dry_run" == true ]]; then
  python3 - "$method" "$scale" <<'PY'
import math, sys
method, scale_text = sys.argv[1:]
scale = float(scale_text)
assert method in ("M1", "M1b")
assert scale in (0.0, .25, .50, .75, 1.0)
b = .139284482
left_raw, right_raw, t = .100, .100, 1.0
q = 1.0
dv = scale * .030 * q * math.sin(t)
dw = scale * .350 * q * math.sin(t + math.pi / 2.0)
left = left_raw + dv - b / 2.0 * dw
right = right_raw + dv + b / 2.0 * dw
print("STAGE=P0_SOFTWARE_ONLY")
print("METHOD={}".format(method))
print("SCALE={:.2f}".format(scale))
print("Av={:.6f} Aomega={:.6f}".format(scale*.030, scale*.350))
print("SYNTHETIC_MAPPING left={:.9f} right={:.9f}".format(left, right))
if scale == 0.0:
    assert left == left_raw and right == right_raw
    print("LAMBDA_ZERO_BASELINE_EXACT=YES")
print("SERIAL_COMMAND_PUBLISHED=NO")
PY
  exit 0
fi

if [[ -z "$operator" || -z "$pair_block" ||
      "$confirm_area" != true || "$confirm_floor" != true ||
      "$confirm_fixture" != true || "$confirm_vision" != true ||
      "$confirm_recorder" != true || "$confirm_hardware" != true ||
      "$confirm_execution" != true ]]; then
  echo "ERROR: operator, pair block and every physical confirmation are required" >&2
  usage
  exit 2
fi

read_config_value() {
  python3 - "$workspace/$physical_config" "$1" <<'PY'
import sys, yaml
value = yaml.safe_load(open(sys.argv[1]))
for key in sys.argv[2].split('.'):
    value = value[key]
if isinstance(value, bool):
    print(str(value).lower())
else:
    print(value)
PY
}

maximum_scale="$(read_config_value formal_fake_runtime.classic_additive_physical.maximum_qualified_scale)"
physical_enabled="$(read_config_value formal_fake_runtime.classic_additive_physical.enabled)"
hardware_authorized="$(read_config_value formal_fake_runtime.classic_additive_physical.hardware_execution_authorized)"

if [[ "$scale" != 0 && ( "$physical_enabled" != true || "$hardware_authorized" != true ) ]]; then
  echo "ERROR: physical Candidate A remains fail-closed in ${physical_config}" >&2
  echo "Both classic_additive_physical.enabled and hardware_execution_authorized must be true after human review." >&2
  exit 3
fi
if [[ "$scale" != 0 ]] && ! awk -v value="$scale" -v maximum="$maximum_scale" \
    'BEGIN {exit !(value <= maximum + 1e-12)}'; then
  echo "ERROR: scale ${scale} exceeds manually qualified maximum ${maximum_scale}; qualification levels cannot be skipped" >&2
  exit 3
fi

authorization="$config_dir/formal_serial_classic_additive_candidate_A_${method}_authorization.yaml"
upper_authorized="$(awk '/^formal_upper:/ {u=1; next} /^formal_lower:/ {u=0} u && /hardware_execution_authorized:/ {print $2; exit}' "$workspace/$authorization")"
lower_authorized="$(awk '/^formal_lower:/ {l=1; next} /^authorization_scope:/ {l=0} l && /hardware_execution_authorized:/ {print $2; exit}' "$workspace/$authorization")"
if [[ "$upper_authorized" != true || "$lower_authorized" != true ]]; then
  echo "ERROR: ${method}+R1 commissioning method authorization remains fail-closed: ${authorization}" >&2
  exit 3
fi

if [[ "$method" == M1 ]]; then
  upper_config="$config_dir/exp2a_M1_serial_0p10_pilot.yaml"
else
  upper_config="$config_dir/exp2c_M1b_risk_disturbance_v1.yaml"
fi

echo "METHOD=${method}"
echo "SCALE=${scale}"
awk -v scale="$scale" 'BEGIN {printf "Av=%.6f\nAomega=%.6f\n", .030*scale, .350*scale}'
echo "PHYSICAL EXECUTION=YES"
echo "This invocation runs exactly one method/level; stop immediately under the documented abort rules."

export FORMAL_UPPER_MODE="$method"
export FORMAL_UPPER_CONFIG="$upper_config"
export FORMAL_RUNTIME_CONFIG="$config_dir/formal_serial_circle_r0p7_smooth_exit_0p10_reference_v1_pilot_runtime.yaml"
export FORMAL_PATH_CONFIG="$config_dir/path_circle_r0p7_cw_smooth_exit.yaml"
export FORMAL_EVALUATION_CONFIG="$config_dir/formal_evaluation_circle_r0p7_smooth_exit.yaml"
export FORMAL_EXECUTION_AUTHORIZATION_CONFIG="$authorization"
export FORMAL_ENABLE_ROBOT2_DERATING=false
export FORMAL_CLASSIC_ADDITIVE_CONFIG="$physical_config"
export FORMAL_CLASSIC_ADDITIVE_SCALE="$scale"
export FORMAL_CLASSIC_ADDITIVE_PHYSICAL_ENABLED="$([[ "$scale" == 0 ]] && echo false || echo true)"
export FORMAL_RECORD_CLASSIC_ADDITIVE_PHYSICAL="$([[ "$scale" == 0 ]] && echo false || echo true)"
export FORMAL_EXPERIMENT_ID="$([[ "$scale" == 0 ]] && echo classic_additive_candidate_A_physical_baseline || echo classic_additive_candidate_A_physical_commissioning)"
export FORMAL_RUN_PREFIX="classic_additive_candidate_A_physical_${method}_lambda_${scale//./p}"
export FORMAL_RUN_TIMEOUT_SECONDS=140
if bash "$script_dir/run_m1_r1_serial_unloaded.sh" \
  --operator "$operator" --pair-block "$pair_block" \
  --confirm-area-clear --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture; then
  echo "PHYSICAL_COMMISSIONING_CAPTURE_COMPLETE_REVIEW_REQUIRED method=${method} scale=${scale}"
else
  status=$?
  echo "PHYSICAL_COMMISSIONING_LEVEL_FAILED method=${method} scale=${scale}" >&2
  exit "$status"
fi
