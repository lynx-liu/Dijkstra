#!/usr/bin/env bash
# 10-minute autofix loop. Start: nohup bash tools/ch_autofix_loop.sh &
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "[$(date '+%F %T')] ch_autofix_loop started pid=$$" >>/tmp/ch_autofix.log
while true; do
  bash "$ROOT/tools/ch_autofix.sh" || true
  sleep 600
done
