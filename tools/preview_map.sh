#!/usr/bin/env bash
# Export sample regions from china.mmlp.bin and start a local map preview server.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GRAPH="${ROOT}/data/graph/china.mmlp.bin"
OUT_DIR="${ROOT}/web/data"
PORT="${PORT:-8765}"

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found: ${GRAPH}" >&2
  echo "Run: bash tools/deploy_graph_nationwide.sh" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

export_geo() {
  local name="$1"
  local bbox="$2"
  echo "[preview] ${name} bbox=${bbox}"
  python3 "${ROOT}/tools/graph_to_geojson.py" \
    -i "${GRAPH}" \
    -o "${OUT_DIR}/${name}.json" \
    --bbox "${bbox}" \
    --max-edges 15000
}

export_geo urumqi   "87.45,43.75,87.75,43.95"
export_geo beijing  "116.2,39.7,116.6,40.0"
export_geo shanghai "121.3,31.1,121.6,31.4"

echo ""
echo "Preview ready. Open in browser:"
echo "  http://127.0.0.1:${PORT}/"
echo ""
cd "${ROOT}/web"
exec python3 -m http.server "${PORT}"
