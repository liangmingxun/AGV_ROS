#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""; ros_port=11710
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    *) echo "ERROR: unknown argument $1" >&2; exit 2 ;;
  esac
done
[[ -n "$output_root" ]] || output_root="$workspace/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_$(date +%Y%m%d_%H%M%S)"
[[ "$output_root" == /* ]] || output_root="$workspace/${output_root#./}"
[[ ! -e "$output_root" ]] || { echo "ERROR: fresh output root required" >&2; exit 2; }
mkdir -p "$output_root"
cd "$workspace"
git rev-parse HEAD >"$output_root/start_head.txt"
git status --short >"$output_root/start_git_status.txt"
printf '%s\n' 'EXPLORATORY_FAKE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE / Candidate B v2 frozen spatial composite / no serial' >"$output_root/SCOPE.txt"

echo "Candidate B v2 spatial geometry sanity run"
bash "$script_dir/run_candidate_B_spatial_composite_one_fake.sh" \
  --method M1b --stage SpatialSanity --target 3.0 --ros-port "$ros_port" \
  --output-root "$output_root/SPATIAL_SANITY_M1b_R1"
python3 src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_composite_fake.py \
  --sanity "$output_root/SPATIAL_SANITY_M1b_R1/run/candidate_b_spatial_metrics.json"

methods=(M1b M1 M2b)
python3 - "$output_root/order.json" "${methods[@]}" <<'PY'
import json, pathlib, sys
pathlib.Path(sys.argv[1]).write_text(json.dumps(sys.argv[2:]) + "\n")
PY
for method in "${methods[@]}"; do
  case "$method" in M1) folder=M1_R1 ;; M1b) folder=M1b_R1 ;; M2b) folder=M2b_M2b ;; esac
  echo "Candidate B v2 fresh fake run: ${method}"
  bash "$script_dir/run_candidate_B_spatial_composite_one_fake.sh" \
    --method "$method" --stage Triad01 --ros-port "$ros_port" \
    --output-root "$output_root/$folder"
done
python3 src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_composite_fake.py \
  --aggregate "$output_root"
mkdir -p docs/test-protocols
cp "$output_root/candidate-B-spatial-composite-rho0p85-av0p020-fake-results.md" \
  docs/test-protocols/candidate-B-spatial-composite-rho0p85-av0p020-fake-results.md
echo "CANDIDATE_B_SPATIAL_COMPARISON_COMPLETE=$output_root"
