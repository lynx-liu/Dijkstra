#!/usr/bin/env bash
# Check → fix → verify. Target: bench <=3s reachable=37
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STATE="${ROOT}/.ch_autofix_state"
LOG="/tmp/ch_autofix.log"
LOCK="/tmp/ch_autofix.lock"
TARGET_SEC=3.0
TARGET_REACH=37

log() { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

if [[ -f "$LOCK" ]]; then
  pid="$(cat "$LOCK" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    log "skip: already running pid=$pid"
    bash "$ROOT/tools/ch_progress.sh" >/dev/null
    exit 0
  fi
fi
echo $$ >"$LOCK"
trap 'rm -f "$LOCK"' EXIT

[[ -f "$STATE" ]] || echo "attempt=0 phase=init" >"$STATE"
# shellcheck disable=SC1090
source "$STATE" 2>/dev/null || { attempt=0; phase=init; }

log "========== tick attempt=$attempt phase=$phase =========="
bash "$ROOT/tools/ch_progress.sh" >/dev/null

health_ok=0
curl -sf --max-time 5 http://127.0.0.1:8080/health >/dev/null && health_ok=1

# Quick CH check (30s cap) — skip if building
ch_ok=0
if [[ "$health_ok" == "1" ]] && ! pgrep -f './build/mmlp_build_hwy' >/dev/null 2>&1; then
  ch_out="$(timeout 45 env MMLP_DEBUG_CH=0 "${ROOT}/build/ch_query_test" \
    "${ROOT}/data/graph/china_prd.mmlp.bin" \
    "${ROOT}/data/graph/china.mmlp.bin" 2>&1)" || true
  log "ch_test: $(echo "$ch_out" | tail -2 | tr '\n' ' ')"
  echo "$ch_out" | grep -q 'hit travel=' && ch_ok=1
fi

bench_sec=""
bench_reach=""
if [[ "$health_ok" == "1" ]] && ! pgrep -f './build/mmlp_build_hwy' >/dev/null 2>&1; then
  bench_out="$(timeout 200 python3 "${ROOT}/tools/bench_dest_arrive.py" --label autofix --timeout 180 --runs 1 2>&1)" || true
  log "bench: $bench_out"
  echo "$bench_out" > /tmp/ch_bench_latest.txt
  bench_sec="$(echo "$bench_out" | sed -n 's/.*autofix: \([0-9.]*\)s.*/\1/p')"
  bench_reach="$(echo "$bench_out" | sed -n 's/.*reachable=\([0-9]*\).*/\1/p')"
fi

goal_ok=0
if [[ -n "$bench_sec" && -n "$bench_reach" ]]; then
  awk -v b="$bench_sec" -v t="$TARGET_SEC" 'BEGIN{exit !(b<=t)}' && [[ "$bench_reach" == "$TARGET_REACH" ]] && goal_ok=1
fi

if [[ "$goal_ok" == "1" ]]; then
  log "GOAL MET: ${bench_sec}s reachable=${bench_reach}"
  echo "attempt=$attempt phase=done last_bench=${bench_sec} last_reach=${bench_reach} ch_ok=$ch_ok" >"$STATE"
  bash "$ROOT/tools/ch_progress.sh"
  echo 'AGENT_LOOP_TICK_ch_autofix GOAL_MET'
  exit 0
fi

# --- one recovery action per tick ---
if pgrep -f './build/mmlp_build_hwy' >/dev/null 2>&1; then
  log "action: wait (build running)"
  echo "attempt=$attempt phase=building last_bench=${bench_sec:-na} ch_ok=$ch_ok" >"$STATE"
elif [[ "$health_ok" != "1" ]]; then
  log "action: restart http"
  pkill -f mmlp_http_server 2>/dev/null || true
  pkill -f mmlp_service 2>/dev/null || true
  sleep 2
  cd "$ROOT"
  nohup env MMLP_WORKERS=12 python3 tools/mmlp_http_server.py \
    --host 127.0.0.1 --port 8080 --graph data/graph/china.mmlp.bin \
    --binary build/mmlp_service --load-mode index >>/tmp/srv.log 2>&1 &
  phase=restart
elif [[ "$ch_ok" != "1" ]]; then
  attempt=$((attempt + 1))
  log "action: rebuild PRD overlay only (attempt=$attempt, skip national)"
  cd "$ROOT"
  nohup bash -c "
    ./build/mmlp_build_hwy_csr data/graph/china_prd.mmlp.bin 2>&1 | tee /tmp/ch_autofix_build.log
    ./build/mmlp_build_hwy_ch data/graph/china_prd.mmlp.bin 2>&1 | tee -a /tmp/ch_autofix_build.log
    echo BUILD_DONE \$(date '+%F %T') >> /tmp/ch_autofix_build.log
  " >>/tmp/ch_autofix_build.log 2>&1 &
  phase=building_prd
else
  log "action: restart service (ch ok, bench slow)"
  pkill -f mmlp_http_server 2>/dev/null || true
  pkill -f mmlp_service 2>/dev/null || true
  sleep 2
  cd "$ROOT"
  nohup env MMLP_WORKERS=12 python3 tools/mmlp_http_server.py \
    --host 127.0.0.1 --port 8080 --graph data/graph/china.mmlp.bin \
    --binary build/mmlp_service --load-mode index >>/tmp/srv.log 2>&1 &
  phase=verify
fi

echo "attempt=$attempt phase=$phase last_bench=${bench_sec:-na} last_reach=${bench_reach:-na} ch_ok=$ch_ok" >"$STATE"
log "NOT AT GOAL: bench=${bench_sec:-?}s reach=${bench_reach:-?} ch_ok=$ch_ok phase=$phase"
bash "$ROOT/tools/ch_progress.sh"
echo 'AGENT_LOOP_TICK_ch_autofix tick'
