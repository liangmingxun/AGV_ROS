#!/usr/bin/env bash
set -euo pipefail

# Explicit 30 cm entry point. Keep all physical gates and motion logic in the
# canonical runner so the two command names cannot drift apart.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="$script_dir/src/multi_agv_bringup/config/three_car_camera_formation_init.yaml"

if ! grep -Eq '^[[:space:]]+side_length:[[:space:]]+0\.30([[:space:]]|$)' \
     "$config"; then
  echo "ERROR: formation initialization config is not the required 0.30 m geometry" >&2
  exit 4
fi

exec bash "$script_dir/run_three_car_camera_formation_init.sh" "$@"
