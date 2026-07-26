#!/usr/bin/env bash
# Source this file and call check_local_clock_sync, or execute it directly.

check_local_clock_sync() {
  local maximum_offset="${AGV_MAX_LOCAL_CLOCK_OFFSET_SECONDS:-0.020}"
  local tracking=""
  local leap_status=""
  local offset=""
  local absolute_offset=""

  CLOCK_SYNC_SOURCE=""
  CLOCK_SYSTEM_OFFSET_SECONDS=""

  if command -v chronyc >/dev/null 2>&1; then
    tracking="$(chronyc tracking 2>/dev/null || true)"
    leap_status="$(
      awk -F: '/^Leap status/ {
        sub(/^[[:space:]]+/, "", $2); sub(/[[:space:]]+$/, "", $2);
        print $2; exit
      }' <<<"$tracking"
    )"
    offset="$(
      awk '/^System time/ {
        value=$4;
        if ($6 == "slow") value=-value;
        printf "%.9f", value;
        exit
      }' <<<"$tracking"
    )"
    if [[ "$leap_status" == "Normal" && -n "$offset" ]]; then
      absolute_offset="$(awk -v value="$offset" \
        'BEGIN {if (value < 0) value=-value; printf "%.9f", value}')"
      if awk -v value="$absolute_offset" -v limit="$maximum_offset" \
        'BEGIN {exit !(value <= limit)}'; then
        CLOCK_SYNC_SOURCE="chrony"
        CLOCK_SYSTEM_OFFSET_SECONDS="$offset"
      else
        echo "ERROR: chrony system offset ${absolute_offset} s exceeds ${maximum_offset} s" >&2
        echo "Wait for chrony to settle and run this check again; do not step the clock during motion." >&2
        return 12
      fi
    fi
  fi

  if [[ -z "$CLOCK_SYNC_SOURCE" ]] &&
     command -v timedatectl >/dev/null 2>&1 &&
     [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]]; then
    CLOCK_SYNC_SOURCE="timedatectl"
    CLOCK_SYSTEM_OFFSET_SECONDS="unknown"
  fi

  if [[ -z "$CLOCK_SYNC_SOURCE" ]]; then
    echo "ERROR: system clock is not synchronised by chrony or timedatectl" >&2
    return 12
  fi

  echo "Clock synchronisation accepted via ${CLOCK_SYNC_SOURCE}" \
       "(system_offset=${CLOCK_SYSTEM_OFFSET_SECONDS}s)"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  set -euo pipefail
  check_local_clock_sync
fi
