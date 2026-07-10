#!/usr/bin/env bash
# Build nationwide dense Full CH: china.mmlp.csr + china.mmlp.full.ch
# Do NOT run ensure_graph_index.sh between steps (it deletes national .csr).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/data/graph/china.mmlp.bin"
CSR="${ROOT}/data/graph/china.mmlp.csr"
OUT="${ROOT}/data/graph/china.mmlp.full.ch"
LOG_DIR="${ROOT}/logs"
BUILD_AUX="${ROOT}/build/mmlp_build_aux"
BUILD_CH="${ROOT}/build/mmlp_build_full_ch"
QUERY_TEST="${ROOT}/build/full_ch_query_test"

mkdir -p "${LOG_DIR}"
if [[ ! -f "${BIN}" ]]; then
  echo "ERROR: missing ${BIN}" >&2
  exit 1
fi
if [[ ! -x "${BUILD_AUX}" || ! -x "${BUILD_CH}" ]]; then
  echo "ERROR: build tools missing; run: cmake --build build --target mmlp_build_aux mmlp_build_full_ch -j" >&2
  exit 1
fi

if [[ ! -f "${CSR}" || "${FORCE_CSR:-0}" == "1" ]]; then
  echo "[national] building CSR -> ${CSR}"
  "${BUILD_AUX}" "${BIN}" --csr-only 2>&1 | tee "${LOG_DIR}/build_national_csr.log"
else
  echo "[national] CSR exists ($(du -h "${CSR}" | cut -f1)); skip (FORCE_CSR=1 to rebuild)"
fi

if [[ ! -f "${OUT}" || "${FORCE_CH:-0}" == "1" ]]; then
  echo "[national] building Full CH -> ${OUT}"
  "${BUILD_CH}" "${BIN}" "${CSR}" "${OUT}" 2>&1 | tee "${LOG_DIR}/build_national_full_ch.log"
else
  echo "[national] full.ch exists ($(du -h "${OUT}" | cut -f1)); skip (FORCE_CH=1 to rebuild)"
fi

ls -lh "${CSR}" "${OUT}"
if [[ -x "${QUERY_TEST}" ]]; then
  echo "[national] smoke query..."
  "${QUERY_TEST}" "${BIN}" 5
fi
echo "[national] done"
