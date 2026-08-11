#!/usr/bin/env bash
set -euo pipefail

passes="${1:-2}"
observe_seconds="${2:-10}"
authority="${3:-any}"
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
  extra_args=()
  if [[ "$authority" == "fused" ]]; then
    extra_args+=(--require-fused-cooperative-state)
  elif [[ "$authority" != "any" ]]; then
    echo "usage: $0 [positive-pass-count] [positive-observe-seconds] [any|fused]" >&2
    exit 2
  fi
  rosrun multi_agv_bringup check_three_car_readonly_gate.py \
    --observe-seconds "$observe_seconds" \
    --require-valid-state "${extra_args[@]}"
done

echo "THREE-CAR MOTION GATE: PASSED ${passes} CONSECUTIVE WINDOW(S)"
