#!/usr/bin/env bash
# Download Geofabrik China OSM extract.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/osm.defaults.env"

mkdir -p "${OSM_DIR}"

if [[ -f "${PBF_PATH}" ]]; then
  echo "[download_osm] already exists: ${PBF_PATH}"
  ls -lh "${PBF_PATH}"
  exit 0
fi

echo "[download_osm] downloading ${GEOFABRIK_URL}"
echo "[download_osm] target ${PBF_PATH} (~1.4 GiB, may take a while)"

if command -v wget >/dev/null 2>&1; then
  wget -c -O "${PBF_PATH}" "${GEOFABRIK_URL}"
elif command -v curl >/dev/null 2>&1; then
  curl -L -C - -o "${PBF_PATH}" "${GEOFABRIK_URL}"
else
  echo "ERROR: need wget or curl" >&2
  exit 1
fi

echo "[download_osm] complete"
ls -lh "${PBF_PATH}"
