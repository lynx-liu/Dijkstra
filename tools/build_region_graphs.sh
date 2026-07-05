#!/usr/bin/env bash
# Build regional graph subsets for fast destination routing near dense areas.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GRAPH="${1:-${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}}"
PY="${ROOT}/tools/build_region_graph.py"
BUILD_AUX="${ROOT}/build/mmlp_build_aux"
BUILD_HWY_CSR="${ROOT}/build/mmlp_build_hwy_csr"
BUILD_HWY_CH="${ROOT}/build/mmlp_build_hwy_ch"
SKIP_EXISTING="${SKIP_EXISTING:-1}"
# prd,sc cover Guangzhou/PRD destination arrive; add nx,xj only when needed.
REGIONS="${REGIONS:-prd,sc}"

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found: ${GRAPH}" >&2
  exit 1
fi

graph_dir="$(dirname "${GRAPH}")"
graph_base="$(basename "${GRAPH}" .bin)"
if [[ "${graph_base}" == *.mmlp ]]; then
  graph_base="${graph_base%.mmlp}"
fi

region_bin() {
  echo "${graph_dir}/${graph_base}_${1}.mmlp.bin"
}

region_index_ready() {
  local bin
  bin="$(region_bin "$1")"
  [[ -f "${bin}" && -f "${bin%.bin}.sidx" && -f "${bin%.bin}.nidx" && -f "${bin%.bin}.eidx" ]]
}

region_prd_hwy_ready() {
  local bin
  bin="$(region_bin prd)"
  [[ -f "${bin%.bin}.hwy.csr" && -f "${bin%.bin}.hwy.ch" ]]
}

want_region() {
  local suffix="$1"
  [[ ",${REGIONS}," == *",${suffix},"* ]]
}

build_prd_hwy() {
  local prd_bin
  prd_bin="$(region_bin prd)"
  if [[ ! -f "${BUILD_HWY_CSR}" || ! -f "${prd_bin}" ]]; then
    return 0
  fi
  echo "=== PRD highway CSR + CH ==="
  "${BUILD_HWY_CSR}" "${prd_bin}"
  if [[ -f "${BUILD_HWY_CH}" ]]; then
    "${BUILD_HWY_CH}" "${prd_bin}"
  fi
}

build_region() {
  local suffix="$1"
  local bbox="$2"
  local margin="${3:-0.15}"
  local bin
  bin="$(region_bin "${suffix}")"

  if [[ "${SKIP_EXISTING}" == "1" ]] && region_index_ready "${suffix}"; then
    echo "[region] ${suffix}: index OK, skip extract (${bin})"
    return 0
  fi

  if [[ "${suffix}" == "prd" ]]; then
    python3 "${PY}" "${GRAPH}" prd --bbox "${bbox}" --margin-deg "${margin}"
  else
    python3 "${PY}" "${GRAPH}" "${suffix}" --bbox "${bbox}"
  fi

  if [[ -f "${BUILD_AUX}" && -f "${bin}" ]]; then
    "${BUILD_AUX}" "${bin}" --csr-only
  fi
}

echo "=== Build regional graphs from ${GRAPH} (regions=${REGIONS}) ==="

if want_region prd; then
  if [[ "${SKIP_EXISTING}" == "1" ]] && region_index_ready prd && region_prd_hwy_ready; then
    echo "[region] prd: graph + hwy overlay OK, skip"
  else
    build_region prd "112.0,22.0,114.8,24.2" "0.08"
    if [[ "${SKIP_EXISTING}" != "1" ]] || ! region_prd_hwy_ready; then
      build_prd_hwy
    else
      echo "[region] prd: hwy overlay OK, skip"
    fi
  fi
fi

if want_region sc; then
  build_region sc "108.0,20.0,118.5,26.5"
fi

if want_region nx; then
  build_region nx "104.0,35.0,108.5,40.5"
fi

if want_region xj; then
  build_region xj "73.0,34.0,96.5,49.5"
fi

echo "Done. Regional files:"
ls -lh "${graph_dir}/${graph_base}"_*.mmlp.bin 2>/dev/null || true
