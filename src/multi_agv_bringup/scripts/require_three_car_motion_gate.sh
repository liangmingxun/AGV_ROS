#!/usr/bin/env bash
set -euo pipefail

passes="${1:-2}"
observe_seconds="${2:-10}"
if [[ ! "$passes" =~ ^[1-9][0-9]*$ ]]; then
  echo "usage: $0 [positive-pass-count] [positive-observe-seconds]" >&2
  exit 2
fi
if ! awk -v value="$observe_seconds" 'BEGIN {exit !(value > 0)}'; then
  echo "usage: $0 [positive-pass-count] [positive-observe-seconds]" >&2
  exit 2
fi

for ((pass = 1; pass <= passes; ++pass)); do
  echo "Three-car motion gate ${pass}/${passes}: ${observe_seconds}s continuous observation"
  rosrun multi_agv_bringup check_three_car_readonly_gate.py \
    --observe-seconds "$observe_seconds" \
    --require-valid-state
done

echo "THREE-CAR MOTION GATE: PASSED ${passes} CONSECUTIVE WINDOW(S)"
