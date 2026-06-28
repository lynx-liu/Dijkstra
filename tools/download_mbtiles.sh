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

if [[ ! -f "${ZIP}" ]]; then
  echo "Downloading China Shortbread mbtiles (~2 GB)..."
  echo "URL: ${MBTILES_DOWNLOAD_URL}"
  curl -fL --connect-timeout 30 --retry 3 --retry-delay 5 -C - -o "${ZIP}" "${MBTILES_DOWNLOAD_URL}"
else
  echo "Using existing zip: ${ZIP}"
  ls -lh "${ZIP}"
fi

resolve_zip_member() {
  local zip="$1"
  local member="${MBTILES_ZIP_MEMBER:-}"
  if [[ -n "${member}" ]] && unzip -l "${zip}" | awk 'NR>3 {print $4}' | grep -Fxq "${member}"; then
    echo "${member}"
    return 0
  fi
  local found
  found="$(unzip -l "${zip}" | awk 'NR>3 && $4 ~ /\.mbtiles$/ {print $4; exit}')"
  if [[ -n "${found}" ]]; then
    echo "${found}"
    return 0
  fi
  echo "ERROR: no .mbtiles found inside ${zip}" >&2
  unzip -l "${zip}" | tail -20 >&2
  return 1
}

ZIP_MEMBER="$(resolve_zip_member "${ZIP}")"
echo "Extracting ${ZIP_MEMBER} ..."

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT
unzip -q -o "${ZIP}" "${ZIP_MEMBER}" -d "${tmpdir}"
mv -f "${tmpdir}/${ZIP_MEMBER}" "${DEST}"

if [[ "${KEEP_ZIP}" != "1" ]]; then
  rm -f "${ZIP}"
fi

echo "Installed offline map: ${DEST}"
ls -lh "${DEST}"
