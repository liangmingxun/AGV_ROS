#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$workspace/src/multi_agv_bringup/scripts/run_circle_0p10_comparison.sh" "$@"
