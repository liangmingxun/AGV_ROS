#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""
smoke_run=""
ros_port=11670
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --smoke-run) smoke_run="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    *) echo "ERROR: unknown argument $1" >&2; exit 2 ;;
  esac
done
[[ -n "$output_root" ]] || output_root="$workspace/experiment_data/classic_additive_disturbance_candidate_A_validation/CLASSIC_ADDITIVE_DISTURBANCE_VALIDATION_NOT_FORMAL_PAPER_EVIDENCE_$(date +%Y%m%d_%H%M%S)"
[[ "$output_root" == /* ]] || output_root="$workspace/${output_root#./}"
[[ ! -e "$output_root" ]] || { echo "ERROR: fresh output root required" >&2; exit 2; }
mkdir -p "$output_root"
cd "$workspace"
git rev-parse HEAD >"$output_root/start_head.txt"
git status --short >"$output_root/start_git_status.txt"
printf '%s\n' 'CLASSIC_ADDITIVE_DISTURBANCE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE; Candidate A only; no parameter search' >"$output_root/SCOPE.txt"

if [[ -z "$smoke_run" ]]; then
  echo "Stage 0: fresh M1 smoke"
  bash "$script_dir/run_classic_additive_candidate_A_one_fake.sh" --method M1 \
    --stage Smoke --ros-port "$ros_port" --output-root "$output_root/Smoke/M1_R1"
  smoke_run="$output_root/Smoke/M1_R1/run"
fi
python3 - "$smoke_run/classic_metrics.json" <<'PY'
import json,sys
d=json.load(open(sys.argv[1]))
assert d['category']=='VALID_COMPLETED' and d['capability_report_unchanged']
assert d['profile_maximum_absolute_error']<1e-9 and d['mapping_maximum_absolute_error']<1e-9
assert d['M1_method_specific_mechanism'] is not None
print('CLASSIC_ADDITIVE_SMOKE_PASSED')
PY

orders=("M1b M1 M2b" "M2b M1b M1" "M1 M2b M1b")
for index in 0 1 2; do
  triad="$(printf 'Triad%02d' "$((index+1))")"
  mkdir -p "$output_root/$triad"
  read -r -a methods <<<"${orders[$index]}"
  python3 - "$output_root/$triad/order.json" "${methods[@]}" <<'PY'
import json,pathlib,sys
pathlib.Path(sys.argv[1]).write_text(json.dumps(sys.argv[2:])+"\n")
PY
  for method in "${methods[@]}"; do
    case "$method" in M1) folder=M1_R1;; M1b) folder=M1b_R1;; M2b) folder=M2b_COMPLETE;; esac
    bash "$script_dir/run_classic_additive_candidate_A_one_fake.sh" --method "$method" \
      --stage "$triad" --ros-port "$ros_port" --output-root "$output_root/$triad/$folder"
  done
done
python3 "$workspace/src/multi_agv_analysis/scripts/analyze_classic_additive_candidate_A.py" --aggregate "$output_root"
echo "CLASSIC_ADDITIVE_VALIDATION_ROOT=$output_root"
