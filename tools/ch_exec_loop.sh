#!/usr/bin/env bash
# Path C: CH connectivity → bench <=3s. Progress every 5 min; watchdog on all steps.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG="/tmp/ch_autofix.log"
PROGRESS="/tmp/ch_progress_latest.txt"
STATE="${ROOT}/.ch_autofix_state"
TARGET_SEC=3.0
TARGET_REACH=37
INTERVAL_SEC=300  # 5 min progress heartbeat

log() { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

write_progress() {
  local phase="$1" bench="$2" reach="$3" ch_ok="$4" eta="$5" detail="$6"
  {
    echo "=== Path C 进度 @ $(date '+%Y-%m-%d %H:%M:%S') ==="
    echo "目标: <=${TARGET_SEC}s  reachable=${TARGET_REACH}"
    echo "阶段: $phase"
    echo "CH: $([ "$ch_ok" = 1 ] && echo hit || echo miss)"
    echo "最近 benchmark: ${bench:-未测}  reachable=${reach:-?}"
    echo "预计剩余: $eta"
    echo "详情: $detail"
    echo ""
    if pgrep -f './build/mmlp_build_hwy' >/dev/null 2>&1; then
      echo "运行中: $(pgrep -af './build/mmlp_build_hwy' | head -1)"
      tail -2 /tmp/ch_autofix_build.log 2>/dev/null | sed 's/^/  /'
    fi
    echo "日志: tail -f /tmp/ch_autofix.log"
    echo "=== end ==="
  } | tee "$PROGRESS"
  echo "AGENT_PROGRESS_C $(date '+%H:%M') phase=$phase bench=${bench:-na} ch=$ch_ok eta=$eta"
}

run_phase() {
  local phase="$1"
  echo "phase=$phase attempt=${attempt:-0}" >"$STATE"
  write_progress "$phase" "${last_bench:-}" "${last_reach:-}" "${ch_ok:-0}" "$2" "$3"
}

attempt=0
ch_ok=0
last_bench=""
last_reach=""

log "======== Path C executor started pid=$$ ========"
run_phase "init" "见下方阶段表" "启动"

# --- Phase 1: build binaries (max 3 min) ---
run_phase "build_bin" "2 min" "编译 mmlp_service"
bash "$ROOT/tools/ch_watchdog.sh" build_bin 180 \
  bash -c "cd '$ROOT' && cmake --build build --target mmlp_service mmlp_build_hwy_csr mmlp_build_hwy_ch ch_query_test -j12" \
  || true

# --- Phase 2: PRD overlay only (csr max 15 min, ch max 20 min) ---
run_phase "prd_overlay" "35 min" "PRD hwy.csr + hwy.ch"
bash "$ROOT/tools/ch_watchdog.sh" prd_csr 900 \
  "$ROOT/build/mmlp_build_hwy_csr" "$ROOT/data/graph/china_prd.mmlp.bin" \
  2>&1 | tee /tmp/ch_autofix_build.log || true
bash "$ROOT/tools/ch_watchdog.sh" prd_ch 1200 \
  "$ROOT/build/mmlp_build_hwy_ch" "$ROOT/data/graph/china_prd.mmlp.bin" \
  2>&1 | tee -a /tmp/ch_autofix_build.log || true

# --- Phase 3: CH smoke test (max 90s) ---
run_phase "ch_test" "2 min" "ch_query_test"
ch_out=$(bash "$ROOT/tools/ch_watchdog.sh" ch_test 90 \
  env MMLP_DEBUG_CH=0 "$ROOT/build/ch_query_test" \
  "$ROOT/data/graph/china_prd.mmlp.bin" "$ROOT/data/graph/china.mmlp.bin" 2>&1) || true
log "ch_out: $(echo "$ch_out" | tail -3 | tr '\n' ' ')"
echo "$ch_out" | grep -q 'hit travel=' && ch_ok=1 || ch_ok=0

# --- Phase 4: restart service (max 2 min) ---
run_phase "restart" "3 min" "加载新 overlay"
pkill -f mmlp_http_server 2>/dev/null || true
pkill -f mmlp_service 2>/dev/null || true
sleep 2
bash "$ROOT/tools/ch_watchdog.sh" restart 120 bash -c "
  cd '$ROOT' && nohup env MMLP_WORKERS=12 python3 tools/mmlp_http_server.py \
    --host 127.0.0.1 --port 8080 --graph data/graph/china.mmlp.bin \
    --binary build/mmlp_service --load-mode index >>/tmp/srv.log 2>&1 &
  for i in \$(seq 1 30); do curl -sf http://127.0.0.1:8080/health >/dev/null && exit 0; sleep 2; done
  exit 1
" || true

# --- Phase 5: benchmark (max 3 min) ---
run_phase "benchmark" "即时" "98车 arrive API"
bench_out=$(bash "$ROOT/tools/ch_watchdog.sh" bench 200 \
  python3 "$ROOT/tools/bench_dest_arrive.py" --label path_c --timeout 180 --runs 1 2>&1) || true
log "bench: $bench_out"
echo "$bench_out" >/tmp/ch_bench_latest.txt
last_bench=$(echo "$bench_out" | sed -n 's/.*path_c: \([0-9.]*\)s.*/\1/p')
last_reach=$(echo "$bench_out" | sed -n 's/.*reachable=\([0-9]*\).*/\1/p')

goal=0
if [[ -n "$last_bench" && -n "$last_reach" ]]; then
  awk -v b="$last_bench" -v t="$TARGET_SEC" 'BEGIN{exit !(b<=t)}' && [[ "$last_reach" == "$TARGET_REACH" ]] && goal=1
fi

if [[ "$goal" == "1" ]]; then
  run_phase "DONE" "0" "GOAL MET ${last_bench}s reachable=${last_reach}"
  log "GOAL MET: ${last_bench}s reachable=${last_reach}"
  exit 0
fi

run_phase "retry_needed" "需改代码/overlay" "bench=${last_bench:-?}s ch_ok=$ch_ok — 下一轮改进"
exit 1
