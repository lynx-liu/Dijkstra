#!/usr/bin/env bash
# Run a command with hard timeout; log start/end to /tmp/ch_autofix.log
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG="/tmp/ch_autofix.log"
LABEL="${1:?label}"
shift
MAX_SEC="${1:?max_seconds}"
shift

log() { echo "[$(date '+%F %T')] [$LABEL] $*" | tee -a "$LOG"; }

log "START (max=${MAX_SEC}s) $*"
t0=$(date +%s)
if timeout --preserve-status "$MAX_SEC" "$@"; then
  rc=0
else
  rc=$?
fi
elapsed=$(( $(date +%s) - t0 ))
if [[ $rc -eq 124 ]]; then
  log "TIMEOUT after ${elapsed}s — killed"
elif [[ $rc -ne 0 ]]; then
  log "FAIL rc=$rc elapsed=${elapsed}s"
else
  log "OK elapsed=${elapsed}s"
fi
exit $rc
