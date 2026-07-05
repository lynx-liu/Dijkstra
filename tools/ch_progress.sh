#!/usr/bin/env bash
# Fast progress snapshot (no slow CH test). Writes /tmp/ch_progress_latest.txt
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="/tmp/ch_progress_latest.txt"
STAMP="$(date '+%Y-%m-%d %H:%M:%S')"

{
  echo "=== CH/dest-arrive 进度 @ ${STAMP} ==="
  echo "目标: <=3s, reachable=37"
  echo ""

  if [[ -f "${ROOT}/.ch_autofix_state" ]]; then
    echo "自动修复状态: $(cat "${ROOT}/.ch_autofix_state")"
  else
    echo "自动修复状态: 未启动"
  fi
  echo ""

  echo "Overlay 文件:"
  for f in china.mmlp.hwy.csr china.mmlp.hwy.ch china_prd.mmlp.hwy.csr china_prd.mmlp.hwy.ch; do
    p="${ROOT}/data/graph/${f}"
    [[ -f "$p" ]] && ls -lh "$p" | awk '{print "  "$6" "$7" "$8"  "$5"  "$9}'
  done
  echo ""

  if pgrep -f './build/mmlp_build_hwy_csr' >/dev/null 2>&1; then
    echo "构建: 进行中 (mmlp_build_hwy_csr)"
    tail -2 /tmp/ch_autofix_build.log 2>/dev/null | sed 's/^/  /'
  elif pgrep -f './build/mmlp_build_hwy_ch' >/dev/null 2>&1; then
    echo "构建: 进行中 (mmlp_build_hwy_ch)"
    tail -2 /tmp/ch_autofix_build.log 2>/dev/null | sed 's/^/  /'
  else
    echo "构建: 空闲"
    grep -E 'overlay_components|post_bridges|wrote.*hwy' /tmp/ch_autofix_build.log 2>/dev/null | tail -2 | sed 's/^/  /' || \
    grep -E 'overlay_components|wrote.*hwy' /tmp/national_hwy_build.log 2>/dev/null | tail -2 | sed 's/^/  /' || true
  fi
  echo ""

  if curl -sf --max-time 3 http://127.0.0.1:8080/health >/dev/null 2>&1; then
    echo "HTTP 服务: 正常 (:8080)"
  else
    echo "HTTP 服务: 未响应"
  fi

  if [[ -f /tmp/ch_autofix.log ]]; then
    echo ""
    echo "最近自动修复日志:"
    tail -8 /tmp/ch_autofix.log | sed 's/^/  /'
  fi

  if [[ -f /tmp/ch_bench_latest.txt ]]; then
    echo ""
    echo "最近 benchmark:"
    cat /tmp/ch_bench_latest.txt | sed 's/^/  /'
  fi

  echo ""
  echo "=== end ==="
} | tee "$OUT"
