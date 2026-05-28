#!/usr/bin/env bash
# Export several preview GeoJSON regions from china.mmlp.bin for web viewer.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GRAPH="${ROOT}/data/graph/china.mmlp.bin"
OUT="${ROOT}/data/graph/preview"
mkdir -p "${OUT}"

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: missing ${GRAPH}" >&2
  exit 1
fi

export_one() {
  local name="$1" bbox="$2"
  echo "=== ${name} bbox=${bbox} ==="
  python3 "${ROOT}/tools/export_graph_geojson.py" \
    -i "${GRAPH}" -o "${OUT}/${name}.geojson" --bbox "${bbox}" --max-features 100000
}

export_one urumqi      "87.45,43.75,87.75,43.95"
export_one beijing     "116.2,39.7,116.6,40.0"
export_one shanghai    "121.3,31.1,121.7,31.4"
export_one xinjiang_west "75.0,38.0,96.0,49.0"

echo "Done. Open web/index.html via: bash tools/serve_map_viewer.sh"
