#!/usr/bin/env bash
# Download Geofabrik China OSM extract (+ Central Asia / Russia when enabled).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/osm.defaults.env"

mkdir -p "${OSM_DIR}"
cd "${ROOT}"

download_one() {
  local url="$1"
  local path="$2"
  local label="$3"
  if [[ -f "${path}" ]]; then
    echo "[download_osm] already exists: ${path}"
    ls -lh "${path}"
    return 0
  fi
  echo "[download_osm] downloading ${label}"
  echo "[download_osm]   url=${url}"
  echo "[download_osm]   -> ${path}"
  if command -v wget >/dev/null 2>&1; then
    wget -c -O "${path}" "${url}"
  elif command -v curl >/dev/null 2>&1; then
    curl -L -C - -o "${path}" "${url}"
  else
    echo "ERROR: need wget or curl" >&2
    exit 1
  fi
  ls -lh "${path}"
}

download_one "${GEOFABRIK_URL}" "${PBF_PATH}" "China"

if [[ "${INCLUDE_CENTRAL_ASIA}" == "1" ]]; then
  echo "[download_osm] INCLUDE_CENTRAL_ASIA=1 — fetching 中亚五国"
  download_one "${CA_KZ_URL}" "${CA_KZ_PBF}" "Kazakhstan 哈萨克斯坦"
  download_one "${CA_KG_URL}" "${CA_KG_PBF}" "Kyrgyzstan 吉尔吉斯斯坦"
  download_one "${CA_TJ_URL}" "${CA_TJ_PBF}" "Tajikistan 塔吉克斯坦"
  download_one "${CA_TM_URL}" "${CA_TM_PBF}" "Turkmenistan 土库曼斯坦"
  download_one "${CA_UZ_URL}" "${CA_UZ_PBF}" "Uzbekistan 乌兹别克斯坦"
else
  echo "[download_osm] INCLUDE_CENTRAL_ASIA=0 — skip Central Asia"
fi

if [[ "${INCLUDE_RUSSIA}" == "1" ]]; then
  echo "[download_osm] INCLUDE_RUSSIA=1 — fetching Russian Federation (~4 GB)"
  download_one "${RU_URL}" "${RU_PBF}" "Russia 俄罗斯"
else
  echo "[download_osm] INCLUDE_RUSSIA=0 — skip Russia"
fi

echo "[download_osm] complete"
