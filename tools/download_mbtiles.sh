#!/usr/bin/env bash
# Download China Shortbread vector mbtiles for offline street map (roads + place names).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/map.defaults.env"

DEST="${ROOT}/${MBTILES_PATH}"
ZIP="${ROOT}/${MAP_DIR}/${MBTILES_ZIP_NAME}"
KEEP_ZIP="${KEEP_MBTILES_ZIP:-0}"

if [[ -f "${DEST}" ]]; then
  echo "mbtiles already installed: ${DEST}"
  ls -lh "${DEST}"
  exit 0
fi

mkdir -p "${ROOT}/${MAP_DIR}"

echo "Downloading China Shortbread mbtiles (~2 GB)..."
echo "URL: ${MBTILES_DOWNLOAD_URL}"
curl -fL --connect-timeout 30 --retry 3 --retry-delay 5 -C - -o "${ZIP}" "${MBTILES_DOWNLOAD_URL}"

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT
echo "Extracting ${MBTILES_ZIP_MEMBER} ..."
unzip -q -o "${ZIP}" "${MBTILES_ZIP_MEMBER}" -d "${tmpdir}"
mv -f "${tmpdir}/${MBTILES_ZIP_MEMBER}" "${DEST}"

if [[ "${KEEP_ZIP}" != "1" ]]; then
  rm -f "${ZIP}"
fi

echo "Installed offline map: ${DEST}"
ls -lh "${DEST}"
