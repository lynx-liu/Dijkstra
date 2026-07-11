#!/usr/bin/env bash
# Download OSM (if needed) and build mmlp binary graph.
# With INCLUDE_CENTRAL_ASIA=1 (default), merges China + 中亚五国 into one graph.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/osm.defaults.env"

LOG_FILE="${DATA_DIR}/deploy.log"
mkdir -p "${OSM_DIR}" "${GRAPH_DIR}"
cd "${ROOT}"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "=== deploy_graph started $(date -Is) ==="

bash "${ROOT}/tools/download_osm.sh"

coverage_required() {
  if [[ "${INCLUDE_CENTRAL_ASIA}" == "1" ]]; then
    echo "${GRAPH_COVERAGE_REQUIRED_CA}"
  else
    echo "${GRAPH_COVERAGE_REQUIRED_CHINA}"
  fi
}

coverage_file() {
  echo "${GRAPH_PATH}.coverage"
}

coverage_ok() {
  local want have
  want="$(coverage_required)"
  [[ -f "$(coverage_file)" ]] || return 1
  have="$(tr -s '[:space:]' ' ' <"$(coverage_file)" | sed 's/^ //;s/ $//')"
  [[ "${have}" == "${want}" ]]
}

list_input_pbfs() {
  local inputs=("${PBF_PATH}")
  if [[ "${INCLUDE_CENTRAL_ASIA}" == "1" ]]; then
    inputs+=("${CA_KZ_PBF}" "${CA_KG_PBF}" "${CA_TJ_PBF}" "${CA_TM_PBF}" "${CA_UZ_PBF}")
  fi
  local p
  for p in "${inputs[@]}"; do
    if [[ ! -f "${p}" ]]; then
      echo "ERROR: missing OSM extract: ${p}" >&2
      exit 1
    fi
    echo "${p}"
  done
}

INPUT_PBF="${PBF_PATH}"
BBOX_ARGS=()
MULTI_INPUTS=()
while IFS= read -r p; do
  MULTI_INPUTS+=("${p}")
done < <(list_input_pbfs)

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
    MULTI_INPUTS=("${CLIP}")
  else
    BBOX_ARGS=(--bbox "${BBOX}")
    echo "[deploy_graph] Python bbox filter ${BBOX}"
  fi
else
  if [[ "${INCLUDE_CENTRAL_ASIA}" == "1" ]]; then
    echo "[deploy_graph] building China + Central Asia (kz/kg/tj/tm/uz) graph"
  else
    echo "[deploy_graph] building China-only graph"
  fi
fi

FORCE_REBUILD="${FORCE_REBUILD_GRAPH:-0}"
if [[ -f "${GRAPH_PATH}" ]] && coverage_ok && [[ "${FORCE_REBUILD}" != "1" ]]; then
  echo "[deploy_graph] graph already exists with coverage: $(cat "$(coverage_file)")"
  ls -lh "${GRAPH_PATH}"
  BASE="${GRAPH_PATH%.mmlp.bin}"
  if [[ "${GRAPH_PATH}" == *.mmlp.bin ]]; then
    BASE="${GRAPH_PATH%.bin}"
  fi
  if [[ ! -f "${BASE}.sidx" ]]; then
    echo "[deploy_graph] building auxiliary index for fast service startup ..."
    "${ROOT}/build/mmlp_build_aux" "${GRAPH_PATH}" --skip-nidx 2>/dev/null ||
      python3 "${ROOT}/tools/build_graph_auxiliary.py" "${GRAPH_PATH}"
  fi
  exit 0
fi

if [[ -f "${GRAPH_PATH}" ]] && ! coverage_ok; then
  echo "[deploy_graph] coverage mismatch — rebuilding graph to include Central Asia"
  echo "  have: $(cat "$(coverage_file)" 2>/dev/null || echo '(none)')"
  echo "  want: $(coverage_required)"
fi

# Optional: single merged PBF via osmium (faster one-file apply).
BUILD_INPUTS=("${MULTI_INPUTS[@]}")
if [[ "${#MULTI_INPUTS[@]}" -gt 1 ]] && command -v osmium >/dev/null 2>&1 && [[ -z "${BBOX:-}" ]]; then
  echo "[deploy_graph] osmium cat -> ${MERGED_PBF_PATH}"
  osmium cat "${MULTI_INPUTS[@]}" -o "${MERGED_PBF_PATH}" --overwrite
  BUILD_INPUTS=("${MERGED_PBF_PATH}")
fi

echo "[deploy_graph] building graph (SQLite streaming, low RAM) ..."
INPUT_ARGS=()
for p in "${BUILD_INPUTS[@]}"; do
  INPUT_ARGS+=(--input "${p}")
done
if ((${#BBOX_ARGS[@]})); then
  python3 "${ROOT}/tools/osm_to_graph.py" "${INPUT_ARGS[@]}" --output "${GRAPH_PATH}" "${BBOX_ARGS[@]}"
else
  python3 "${ROOT}/tools/osm_to_graph.py" "${INPUT_ARGS[@]}" --output "${GRAPH_PATH}"
fi

printf '%s\n' "$(coverage_required)" >"$(coverage_file)"
echo "[deploy_graph] wrote coverage: $(cat "$(coverage_file)")"

# Graph changed → invalidate national Full CH / indexes so bootstrap rebuilds them.
BASE="${GRAPH_PATH%.bin}"
rm -f "${BASE}.full.ch" "${BASE}.csr" "${BASE}.sidx" "${BASE}.nidx" "${BASE}.eidx" "${BASE}.egeo" \
  "${BASE}.hwy.ch" "${BASE}.hwy.csr" "${BASE}.rtidx"

echo "[deploy_graph] graph ready: ${GRAPH_PATH}"
ls -lh "${GRAPH_PATH}"
echo "[deploy_graph] building auxiliary index for fast service startup ..."
# shellcheck source=/dev/null
source "${ROOT}/tools/env_runtime.sh" 2>/dev/null || true
cmake --build "${ROOT}/build" --target mmlp_build_aux 2>/dev/null || true
if [[ -x "${ROOT}/build/mmlp_build_aux" ]]; then
  "${ROOT}/build/mmlp_build_aux" "${GRAPH_PATH}"
else
  python3 "${ROOT}/tools/build_graph_auxiliary.py" "${GRAPH_PATH}"
fi

echo "[deploy_graph] done $(date -Is)"
echo "Next: bash tools/bootstrap_service.sh   # rebuilds national Full CH from new graph"
