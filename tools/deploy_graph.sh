#!/usr/bin/env bash
# Download OSM (if needed) and build mmlp binary graph.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/osm.defaults.env"

LOG_FILE="${DATA_DIR}/deploy.log"
mkdir -p "${OSM_DIR}" "${GRAPH_DIR}"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "=== deploy_graph started $(date -Is) ==="

if [[ ! -f "${PBF_PATH}" ]]; then
  bash "${ROOT}/tools/download_osm.sh"
fi

INPUT_PBF="${PBF_PATH}"
BBOX_ARGS=()
if [[ -n "${BBOX:-}" ]]; then
  if [[ "${USE_OSMIUM_EXTRACT:-0}" == "1" ]]; then
    if ! command -v osmium >/dev/null 2>&1; then
      echo "ERROR: osmium-tool required (apt install osmium-tool)" >&2
      exit 1
    fi
    CLIP="${OSM_DIR}/china-bbox.osm.pbf"
    echo "[deploy_graph] osmium extract bbox=${BBOX}"
    osmium extract -b "${BBOX}" "${PBF_PATH}" -o "${CLIP}" --overwrite
    INPUT_PBF="${CLIP}"
  else
    BBOX_ARGS=(--bbox "${BBOX}")
    echo "[deploy_graph] Python bbox filter ${BBOX}"
  fi
else
  echo "[deploy_graph] building nationwide graph"
fi

# Skip download if PBF already complete
if [[ -f "${GRAPH_PATH}" ]]; then
  echo "[deploy_graph] graph already exists: ${GRAPH_PATH}"
  ls -lh "${GRAPH_PATH}"
  BASE="${GRAPH_PATH%.mmlp.bin}"
  if [[ ! -f "${BASE}.sidx" ]]; then
    echo "[deploy_graph] building auxiliary index for fast service startup ..."
    "${ROOT}/build/mmlp_build_aux" "${GRAPH_PATH}" --skip-nidx 2>/dev/null ||
      python3 "${ROOT}/tools/build_graph_auxiliary.py" "${GRAPH_PATH}"
  fi
  exit 0
fi

echo "[deploy_graph] building graph (SQLite streaming, low RAM) ..."
python3 "${ROOT}/tools/osm_to_graph.py" --input "${INPUT_PBF}" --output "${GRAPH_PATH}" "${BBOX_ARGS[@]}"

echo "[deploy_graph] graph ready: ${GRAPH_PATH}"
ls -lh "${GRAPH_PATH}"
echo "[deploy_graph] building auxiliary index for fast service startup ..."
cmake --build "${ROOT}/build" --target mmlp_build_aux 2>/dev/null || true
if [[ -x "${ROOT}/build/mmlp_build_aux" ]]; then
  "${ROOT}/build/mmlp_build_aux" "${GRAPH_PATH}"
else
  python3 "${ROOT}/tools/build_graph_auxiliary.py" "${GRAPH_PATH}"
fi
