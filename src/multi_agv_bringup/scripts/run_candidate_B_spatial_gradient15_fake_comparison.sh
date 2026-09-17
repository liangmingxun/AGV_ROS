#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd "${script_dir}/../../.." && pwd)"
output_root=""; ros_port=11710; m1_upper_rate_hz=25
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --ros-port) ros_port="$2"; shift 2 ;;
    --m1-upper-rate-hz) m1_upper_rate_hz="$2"; shift 2 ;;
    *) echo "ERROR: unknown argument $1" >&2; exit 2 ;;
  esac
done
[[ "$m1_upper_rate_hz" == 25 || "$m1_upper_rate_hz" == 100 ]] || {
  echo "ERROR: --m1-upper-rate-hz must be 25 or 100" >&2; exit 2; }
variant="M1UPPER${m1_upper_rate_hz}HZ"
[[ -n "$output_root" ]] || output_root="$workspace/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_${variant}_$(date +%Y%m%d_%H%M%S)"
[[ "$output_root" == /* ]] || output_root="$workspace/${output_root#./}"
[[ ! -e "$output_root" ]] || { echo "ERROR: fresh output root required" >&2; exit 2; }
mkdir -p "$output_root"
cd "$workspace"
git rev-parse HEAD >"$output_root/start_head.txt"
git status --short >"$output_root/start_git_status.txt"
printf '%s\n' "EXPLORATORY_FAKE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE / Candidate B v2 gradient15 / M1 upper ${m1_upper_rate_hz} Hz / no serial" >"$output_root/SCOPE.txt"

analysis="src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_gradient15_fake.py"
identity="candidate_B_spatial_gradient15_fake_validation"
methods=(M1b M1 M2b)
python3 - "$output_root/order.json" "${methods[@]}" <<'PY'
import json, pathlib, sys
pathlib.Path(sys.argv[1]).write_text(json.dumps(sys.argv[2:]) + "\n")
PY
for method in "${methods[@]}"; do
  case "$method" in M1) folder=M1_R1 ;; M1b) folder=M1b_R1 ;; M2b) folder=M2b_M2b ;; esac
  echo "Candidate B v2 gradient15 fresh fake run: ${method}"
  bash "$script_dir/run_candidate_B_spatial_composite_one_fake.sh" \
    --method "$method" --stage Gradient15Triad01 --ros-port "$ros_port" \
    --analysis-script "$analysis" --experiment-id "$identity" \
    --m1-upper-rate-hz "$m1_upper_rate_hz" \
    --output-root "$output_root/$folder"
done
python3 "$analysis" --aggregate "$output_root"
mkdir -p docs/test-protocols
report_target="docs/test-protocols/candidate-B-spatial-gradient15-rho0p85-av0p020-fake-results.md"
if [[ "$m1_upper_rate_hz" == 100 ]]; then
  report_target="docs/test-protocols/candidate-B-spatial-gradient15-m1-upper100hz-fake-results.md"
fi
cp "$output_root/candidate-B-spatial-gradient15-rho0p85-av0p020-fake-results.md" \
  "$report_target"
echo "CANDIDATE_B_SPATIAL_GRADIENT15_COMPARISON_COMPLETE=$output_root"
