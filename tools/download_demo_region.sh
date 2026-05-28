#!/usr/bin/env bash
# Download real OSM XML for Urumqi area (demo, ~few MB) and build mmlp graph.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OSM_DIR="${ROOT}/data/osm"
GRAPH_DIR="${ROOT}/data/graph"
mkdir -p "${OSM_DIR}" "${GRAPH_DIR}"

# Urumqi vicinity: minLon,minLat,maxLon,maxLat
BBOX="87.45,43.75,87.75,43.95"
OUT_OSM="${OSM_DIR}/urumqi_demo.osm"
OUT_BIN="${GRAPH_DIR}/urumqi_demo.mmlp.bin"

if [[ -f "${OUT_OSM}" && -s "${OUT_OSM}" ]]; then
  echo "[download_demo] using existing ${OUT_OSM}"
else
  echo "[download_demo] fetching OSM from Overpass API (bbox=${BBOX}) ..."
  # BBOX env: minLon,minLat,maxLon,maxLat  -> Overpass: south,west,north,east
  IFS=',' read -r MINLON MINLAT MAXLON MAXLAT <<< "${BBOX}"
  OP_BBOX="${MINLAT},${MINLON},${MAXLAT},${MAXLON}"
  QUERY="[out:xml][timeout:180];(
    way[\"highway\"~\"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|service)\$\"](${OP_BBOX});
    way[\"railway\"=\"rail\"](${OP_BBOX});
  );out body;>;out skel qt;"
  echo "[download_demo] Overpass bbox (s,w,n,e)=${OP_BBOX}"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "${OUT_OSM}" --data-urlencode "data=${QUERY}" \
      "https://overpass-api.de/api/interpreter"
  else
    wget -q -O "${OUT_OSM}" --post-data="data=${QUERY}" \
      "https://overpass-api.de/api/interpreter"
  fi
  echo "[download_demo] saved $(ls -lh "${OUT_OSM}" | awk '{print $5}') -> ${OUT_OSM}"
fi

python3 "${ROOT}/tools/osm_to_graph.py" --input "${OUT_OSM}" --output "${OUT_BIN}"
echo "[download_demo] graph -> ${OUT_BIN}"
ls -lh "${OUT_BIN}"
