#!/usr/bin/env bash
# Download MapLibre GL + Leaflet bridge for offline vector mbtiles rendering.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENDOR="${ROOT}/web/vendor/map"
ML_VER="3.6.2"
LEAFLET_BRIDGE_VER="0.0.20"

mkdir -p "${VENDOR}"

fetch() {
  local out="$1"
  shift
  for url in "$@"; do
    if curl -fsSL --connect-timeout 15 --max-time 300 -o "${out}" "${url}"; then
      return 0
    fi
    rm -f "${out}"
  done
  return 1
}

if [[ -f "${VENDOR}/maplibre-gl.js" && -f "${VENDOR}/maplibre-gl.css" && -f "${VENDOR}/leaflet-maplibre-gl.js" ]]; then
  echo "MapLibre vendor already present: ${VENDOR}"
  exit 0
fi

echo "Fetching MapLibre ${ML_VER} -> ${VENDOR}"

fetch "${VENDOR}/maplibre-gl.css" \
  "https://unpkg.com/maplibre-gl@${ML_VER}/dist/maplibre-gl.css"

fetch "${VENDOR}/maplibre-gl.js" \
  "https://unpkg.com/maplibre-gl@${ML_VER}/dist/maplibre-gl.js"

fetch "${VENDOR}/leaflet-maplibre-gl.js" \
  "https://unpkg.com/@maplibre/maplibre-gl-leaflet@${LEAFLET_BRIDGE_VER}/leaflet-maplibre-gl.js"

echo "MapLibre vendor ready."
