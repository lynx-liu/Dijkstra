#!/usr/bin/env bash
# Download China (+ optional Central Asia) Shortbread vector mbtiles for offline map.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/map.defaults.env"
# Align with graph coverage flag when set in osm.defaults.env
if [[ -f "${ROOT}/config/osm.defaults.env" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT}/config/osm.defaults.env"
fi
INCLUDE_CENTRAL_ASIA_MAP="${INCLUDE_CENTRAL_ASIA_MAP:-${INCLUDE_CENTRAL_ASIA:-1}}"

DEST="${ROOT}/${MBTILES_PATH}"
ZIP="${ROOT}/${MAP_DIR}/${MBTILES_ZIP_NAME}"
KEEP_ZIP="${KEEP_MBTILES_ZIP:-0}"
CA_DIR="${ROOT}/${MBTILES_CA_DIR}"

download_zip_extract() {
  local url="$1"
  local zip="$2"
  local dest="$3"
  local label="$4"

  if [[ -f "${dest}" ]]; then
    echo "[mbtiles] already exists: ${dest}"
    ls -lh "${dest}"
    return 0
  fi

  mkdir -p "$(dirname "${zip}")" "$(dirname "${dest}")"
  if [[ ! -f "${zip}" ]]; then
    echo "[mbtiles] downloading ${label}..."
    echo "  URL: ${url}"
    curl -fL --connect-timeout 30 --retry 3 --retry-delay 5 -C - -o "${zip}" "${url}"
  else
    echo "[mbtiles] using existing zip: ${zip}"
    ls -lh "${zip}"
  fi

  local member found
  member=""
  if unzip -l "${zip}" | awk 'NR>3 {print $4}' | grep -Eq '\.mbtiles$'; then
    found="$(unzip -l "${zip}" | awk 'NR>3 && $4 ~ /\.mbtiles$/ {print $4; exit}')"
    member="${found}"
  fi
  if [[ -z "${member}" ]]; then
    echo "ERROR: no .mbtiles in ${zip}" >&2
    unzip -l "${zip}" | tail -20 >&2
    return 1
  fi

  echo "[mbtiles] extracting ${member} -> ${dest}"
  local tmpdir
  tmpdir="$(mktemp -d)"
  if ! unzip -q -o "${zip}" "${member}" -d "${tmpdir}"; then
    rm -rf "${tmpdir}"
    return 1
  fi
  mv -f "${tmpdir}/${member}" "${dest}"
  rm -rf "${tmpdir}"
  if [[ "${KEEP_ZIP}" != "1" ]]; then
    rm -f "${zip}"
  fi
  ls -lh "${dest}"
}

# --- China base ---
if [[ -f "${DEST}" ]]; then
  echo "[mbtiles] China base present: ${DEST}"
  ls -lh "${DEST}"
else
  download_zip_extract "${MBTILES_DOWNLOAD_URL}" "${ZIP}" "${DEST}" "China Shortbread (~2 GB)"
fi

# --- Central Asia overlays ---
if [[ "${INCLUDE_CENTRAL_ASIA_MAP}" == "1" ]]; then
  echo "[mbtiles] INCLUDE_CENTRAL_ASIA_MAP=1 — fetching 中亚五国 Shortbread"
  mkdir -p "${CA_DIR}"
  download_zip_extract "${CA_KZ_MBTILES_URL}" "${CA_DIR}/kazakhstan.mbtiles-shortbread.zip" \
    "${ROOT}/${CA_KZ_MBTILES}" "Kazakhstan"
  download_zip_extract "${CA_KG_MBTILES_URL}" "${CA_DIR}/kyrgyzstan.mbtiles-shortbread.zip" \
    "${ROOT}/${CA_KG_MBTILES}" "Kyrgyzstan"
  download_zip_extract "${CA_TJ_MBTILES_URL}" "${CA_DIR}/tajikistan.mbtiles-shortbread.zip" \
    "${ROOT}/${CA_TJ_MBTILES}" "Tajikistan"
  download_zip_extract "${CA_TM_MBTILES_URL}" "${CA_DIR}/turkmenistan.mbtiles-shortbread.zip" \
    "${ROOT}/${CA_TM_MBTILES}" "Turkmenistan"
  download_zip_extract "${CA_UZ_MBTILES_URL}" "${CA_DIR}/uzbekistan.mbtiles-shortbread.zip" \
    "${ROOT}/${CA_UZ_MBTILES}" "Uzbekistan"
  echo "[mbtiles] Central Asia overlays ready under ${CA_DIR}"
  ls -lh "${CA_DIR}"/*.mbtiles 2>/dev/null || true
else
  echo "[mbtiles] INCLUDE_CENTRAL_ASIA_MAP=0 — China base only"
fi

echo "[mbtiles] done"
echo "Restart HTTP service to pick up overlays: bash tools/start_http_server.sh"
