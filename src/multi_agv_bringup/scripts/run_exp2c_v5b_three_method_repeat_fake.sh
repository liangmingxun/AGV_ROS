#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
smoke_run=""
ros_port=11649
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --smoke-run) smoke_run="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    *) echo "ERROR: unknown option $1" >&2; exit 2 ;;
  esac
done
cd "$workspace"
[[ -n "$output_root" && -n "$smoke_run" && ! -e "$output_root" ]] || { echo "Fresh --output-root and verified --smoke-run required" >&2; exit 2; }
python3 - "$smoke_run" <<'PY'
import sys,json,pathlib
d=json.loads((pathlib.Path(sys.argv[1])/'repeat_metrics.json').read_text())
assert d['experiment_id']=='exp2c_v5b_s1_three_method_repeat_validation'
assert d['category']=='VALID_COMPLETED' and d['complete'] and d['method']=='M2b_COMPLETE'
assert d['M2b_method_specific_diagnostics'] and d['phases']['disturbance']['samples']>300
print('M2B_INTEGRATION_READY')
PY
mkdir -p "$output_root"
git rev-parse HEAD > "$output_root/start_head.txt"
git status --short > "$output_root/start_git_status.txt"
printf '%s\n' 'REPEAT_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE; frozen S1, gain=0.10, five triads, no parameter search' > "$output_root/SCOPE.txt"
orders=("M1b M1 M2b" "M2b M1b M1" "M1 M2b M1b" "M1b M2b M1" "M2b M1 M1b")
for index in 0 1 2 3 4; do
  triad="$(printf 'Triad%02d' "$((index + 1))")"
  mkdir -p "$output_root/$triad"
  read -r -a methods <<< "${orders[$index]}"
  python3 - "$output_root/$triad/order.json" "${methods[@]}" <<'PY'
import json,pathlib,sys
pathlib.Path(sys.argv[1]).write_text(json.dumps(sys.argv[2:])+'\n')
PY
  for method in "${methods[@]}"; do
    case "$method" in M1) folder=M1_R1 ;; M1b) folder=M1b_R1 ;; M2b) folder=M2b_COMPLETE ;; esac
    echo "REPEAT_VALIDATION: $triad $folder; fresh master, plant, controller and estimator"
    bash "$script_dir/run_exp2c_v5b_repeat_one_fake.sh" \
      --method "$method" --ros-port "$ros_port" \
      --output-root "$output_root/$triad/$folder"
  done
done
python3 "$workspace/src/multi_agv_analysis/scripts/analyze_exp2c_v5b_three_method_repeat.py" --aggregate "$output_root"
