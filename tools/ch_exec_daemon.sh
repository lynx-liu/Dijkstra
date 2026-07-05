#!/usr/bin/env bash
# Daemon: run Path C executor every 30 min; heartbeat progress every 5 min.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG="/tmp/ch_autofix.log"
HEARTBEAT=300
COOLDOWN=1800

echo "[$(date '+%F %T')] ch_exec_daemon started pid=$$" >>"$LOG"

while true; do
  # Heartbeat during long inner steps
  (
    while pgrep -f 'ch_exec_loop.sh' >/dev/null 2>&1; do
      bash "$ROOT/tools/ch_progress.sh" >/dev/null 2>&1 || true
      sleep "$HEARTBEAT"
    done
  ) &
  hb_pid=$!

  bash "$ROOT/tools/ch_exec_loop.sh" >>"$LOG" 2>&1
  rc=$?
  kill "$hb_pid" 2>/dev/null || true

  if [[ $rc -eq 0 ]]; then
    echo "[$(date '+%F %T')] GOAL MET — daemon stopping" >>"$LOG"
    exit 0
  fi

  echo "[$(date '+%F %T')] cycle failed rc=$rc — cooldown ${COOLDOWN}s" >>"$LOG"
  sleep "$COOLDOWN"
done
