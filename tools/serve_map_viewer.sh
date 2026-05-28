#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${1:-8765}"
BIND="${BIND:-0.0.0.0}"

if [[ ! -f "${ROOT}/web/data/urumqi.geojson" && -f "${ROOT}/data/graph/preview/urumqi.geojson" ]]; then
  mkdir -p "${ROOT}/web/data"
  cp "${ROOT}/data/graph/preview/urumqi.geojson" "${ROOT}/web/data/urumqi.geojson"
fi

if [[ ! -f "${ROOT}/web/data/urumqi.geojson" ]]; then
  echo "WARN: 缺少 web/data/urumqi.geojson"
  echo "  运行: bash tools/export_preview_regions.sh"
  echo "  或: python3 tools/export_graph_geojson.py -i data/graph/china.mmlp.bin \\"
  echo "        -o web/data/urumqi.geojson --bbox 87.45,43.75,87.75,43.95"
fi

cd "${ROOT}"
echo "=========================================="
echo "  路网预览（必须保持本终端运行）"
echo "  浏览器打开:"
echo "    http://127.0.0.1:${PORT}/web/index.html"
if [[ "${BIND}" == "0.0.0.0" ]]; then
  echo "  远程/容器请用机器 IP 替换 127.0.0.1，并确认端口 ${PORT} 已转发"
fi
echo "  不要用 file:// 直接打开 html"
echo "  按 Ctrl+C 停止"
echo "=========================================="
exec python3 -m http.server "${PORT}" --bind "${BIND}"
