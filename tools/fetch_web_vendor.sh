#!/usr/bin/env bash
# Download Leaflet for offline / restricted-network map pages.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENDOR="${ROOT}/web/vendor/leaflet"
VER="1.9.4"

mkdir -p "${VENDOR}"

fetch() {
  local out="$1"
  shift
  for url in "$@"; do
    if curl -fsSL --connect-timeout 15 --max-time 120 -o "${out}" "${url}"; then
      return 0
    fi
  done
  return 1
}

echo "Fetching Leaflet ${VER} -> ${VENDOR}"

fetch "${VENDOR}/leaflet.css" \
  "https://cdn.bootcdn.net/ajax/libs/leaflet/${VER}/leaflet.css" \
  "https://unpkg.com/leaflet@${VER}/dist/leaflet.css"

fetch "${VENDOR}/leaflet.js" \
  "https://cdn.bootcdn.net/ajax/libs/leaflet/${VER}/leaflet.js" \
  "https://unpkg.com/leaflet@${VER}/dist/leaflet.js"

echo "Leaflet vendor ready."
