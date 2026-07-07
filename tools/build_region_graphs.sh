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
# Nationwide regional tiles (areas outside these fall back to national graph):
#   prd — Guangzhou/PRD metro (+ hwy overlay)
#   gd  — Guangdong province
#   sc  — South China (Hainan, Guangxi, coastal south)
#   nx  — Ningxia / Inner Mongolia west
#   xj  — Xinjiang
ALL_REGIONS="prd,gd,sc,nx,xj"
REGIONS="${REGIONS:-all}"
if [[ "${REGIONS}" == "all" ]]; then
  REGIONS="${ALL_REGIONS}"
fi

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found: ${GRAPH}" >&2
  exit 1
fi

graph_dir="$(dirname "${GRAPH}")"
graph_base="$(basename "${GRAPH}" .bin)"
if [[ "${graph_base}" == *.mmlp ]]; then
  graph_base="${graph_base%.mmlp}"
fi

STEP=0
TOTAL_STEPS=0
SCRIPT_START=$SECONDS

region_bin() {
  echo "${graph_dir}/${graph_base}_${1}.mmlp.bin"
}

region_index_ready() {
  local bin
  bin="$(region_bin "$1")"
  [[ -f "${bin}" && -f "${bin%.bin}.sidx" && -f "${bin%.bin}.nidx" && -f "${bin%.bin}.eidx" ]]
}

region_csr_ready() {
  local bin
  bin="$(region_bin "$1")"
  [[ -f "${bin%.bin}.csr" ]]
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

count_planned_steps() {
  TOTAL_STEPS=0
  for suffix in prd gd sc nx xj; do
    if ! want_region "${suffix}"; then
      continue
    fi
    if [[ "${SKIP_EXISTING}" != "1" ]] || ! region_index_ready "${suffix}" || ! region_csr_ready "${suffix}"; then
      TOTAL_STEPS=$((TOTAL_STEPS + 2))
    fi
  done
  if want_region prd && [[ "${SKIP_EXISTING}" != "1" ]] || ! region_prd_hwy_ready; then
    if want_region prd; then
      TOTAL_STEPS=$((TOTAL_STEPS + 2))
    fi
  fi
  if [[ "${TOTAL_STEPS}" -eq 0 ]]; then
    TOTAL_STEPS=1
  fi
}

progress_step() {
  local label="$1"
  STEP=$((STEP + 1))
  local elapsed=$((SECONDS - SCRIPT_START))
  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  printf "  [%d/%d] +%ds  %s\n" "${STEP}" "${TOTAL_STEPS}" "${elapsed}" "${label}"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

progress_skip() {
  local label="$1"
  echo "[skip] ${label}"
}

progress_done() {
  local label="$1"
  local dt="$2"
  echo "[done] ${label} (${dt}s)"
}

build_prd_hwy() {
  local prd_bin
  prd_bin="$(region_bin prd)"
  if [[ ! -f "${BUILD_HWY_CSR}" || ! -f "${prd_bin}" ]]; then
    return 0
  fi
  local t0=$SECONDS
  progress_step "PRD highway CSR"
  "${BUILD_HWY_CSR}" "${prd_bin}"
  progress_done "PRD highway CSR" "$((SECONDS - t0))"

  if [[ -f "${BUILD_HWY_CH}" ]]; then
    t0=$SECONDS
    progress_step "PRD highway CH (contraction)"
    "${BUILD_HWY_CH}" "${prd_bin}"
    progress_done "PRD highway CH" "$((SECONDS - t0))"
  fi
}

build_region() {
  local suffix="$1"
  local bbox="$2"
  local margin="${3:-0.15}"
  local bin
  bin="$(region_bin "${suffix}")"

  if [[ "${SKIP_EXISTING}" == "1" ]] && region_index_ready "${suffix}" && region_csr_ready "${suffix}"; then
    progress_skip "${suffix}: index + CSR already present (${bin})"
    return 0
  fi

  if [[ -f "${bin}" ]] && region_index_ready "${suffix}" && ! region_csr_ready "${suffix}"; then
    progress_skip "${suffix}: bin + index present, CSR missing — building CSR only"
    if [[ -f "${BUILD_AUX}" ]]; then
      local t0=$SECONDS
      progress_step "${suffix}: build CSR (mmlp_build_aux --csr-only)"
      "${BUILD_AUX}" "${bin}" --csr-only
      progress_done "${suffix} CSR" "$((SECONDS - t0))"
    fi
    return 0
  fi

  local t0=$SECONDS
  progress_step "${suffix}: extract subgraph bbox=${bbox}"
  if [[ "${suffix}" == "prd" ]]; then
    python3 "${PY}" "${GRAPH}" prd --bbox "${bbox}" --margin-deg "${margin}"
  else
    python3 "${PY}" "${GRAPH}" "${suffix}" --bbox "${bbox}"
  fi
  progress_done "${suffix} extract" "$((SECONDS - t0))"

  if [[ -f "${BUILD_AUX}" && -f "${bin}" ]]; then
    t0=$SECONDS
    progress_step "${suffix}: build spatial index + CSR (mmlp_build_aux)"
    "${BUILD_AUX}" "${bin}" --csr-only
    progress_done "${suffix} index+CSR" "$((SECONDS - t0))"
  fi
}

echo "=== Build regional graphs ==="
echo "Graph:   ${GRAPH}"
echo "Regions: ${REGIONS} (set REGIONS=all for ${ALL_REGIONS})"
echo "Skip:    SKIP_EXISTING=${SKIP_EXISTING}"
count_planned_steps
echo "Planned build steps: ${TOTAL_STEPS} (skipped regions count as instant)"
echo ""

if want_region prd; then
  if [[ "${SKIP_EXISTING}" == "1" ]] && region_index_ready prd && region_csr_ready prd && region_prd_hwy_ready; then
    progress_skip "prd: graph + CSR + hwy overlay OK"
  else
    build_region prd "112.0,22.0,114.8,24.2" "0.08"
    if [[ "${SKIP_EXISTING}" != "1" ]] || ! region_prd_hwy_ready; then
      build_prd_hwy
    else
      progress_skip "prd: hwy overlay OK"
    fi
  fi
fi

if want_region gd; then
  build_region gd "109.65,20.15,117.25,25.55" "0.08"
fi

if want_region sc; then
  build_region sc "108.0,20.0,118.5,26.5"
fi

if want_region nx; then
  build_region nx "106.0,38.0,112.0,42.0"
fi

if want_region xj; then
  build_region xj "73.0,34.0,96.5,49.5"
fi

echo ""
echo "=== Regional graphs summary (+$((SECONDS - SCRIPT_START))s total) ==="
for suffix in prd gd sc nx xj; do
  if ! want_region "${suffix}"; then
    continue
  fi
  bin="$(region_bin "${suffix}")"
  if [[ -f "${bin}" ]]; then
    size="$(du -h "${bin}" | awk '{print $1}')"
    flags=""
    region_index_ready "${suffix}" && flags+=" index"
    region_csr_ready "${suffix}" && flags+=" csr"
    [[ "${suffix}" == "prd" ]] && region_prd_hwy_ready && flags+=" hwy"
    echo "  ${suffix}: ${bin} (${size})${flags}"
  else
    echo "  ${suffix}: MISSING"
  fi
done
