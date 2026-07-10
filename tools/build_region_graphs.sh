#!/usr/bin/env bash
# Build provincial/regional graph subsets + optional full-CH sidecars.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GRAPH="${1:-${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}}"
PY="${ROOT}/tools/build_region_graph.py"
GEN="${ROOT}/tools/gen_china_regions.py"
REGIONS_JSON="${ROOT}/config/china_regions.json"
BUILD_AUX="${ROOT}/build/mmlp_build_aux"
BUILD_HWY_CSR="${ROOT}/build/mmlp_build_hwy_csr"
BUILD_HWY_CH="${ROOT}/build/mmlp_build_hwy_ch}"
BUILD_FULL_CH_BIN="${ROOT}/build/mmlp_build_full_ch"
SKIP_EXISTING="${SKIP_EXISTING:-1}"
BUILD_FULL_CH="${BUILD_FULL_CH:-0}"
FORCE_REGIONS="${FORCE_REGIONS:-}"
REGIONS="${REGIONS:-all}"

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found: ${GRAPH}" >&2
  exit 1
fi
if [[ ! -f "${REGIONS_JSON}" ]]; then
  echo "ERROR: ${REGIONS_JSON} not found" >&2
  exit 1
fi

python3 "${GEN}" >/dev/null

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

region_csr_ready() {
  local bin
  bin="$(region_bin "$1")"
  [[ -f "${bin%.bin}.csr" ]]
}

region_full_ch_ready() {
  local bin
  bin="$(region_bin "$1")"
  [[ -f "${bin%.bin}.full.ch" ]]
}

region_hwy_ready() {
  local bin
  bin="$(region_bin "$1")"
  [[ -f "${bin%.bin}.hwy.csr" && -f "${bin%.bin}.hwy.ch" ]]
}

force_region() {
  local suffix="$1"
  [[ -n "${FORCE_REGIONS}" && ",${FORCE_REGIONS}," == *",${suffix},"* ]]
}

want_region() {
  local suffix="$1"
  if [[ "${REGIONS}" == "all" ]]; then
    return 0
  fi
  [[ ",${REGIONS}," == *",${suffix},"* ]]
}

list_regions() {
  python3 - "${REGIONS_JSON}" <<'PY'
import json, sys
with open(sys.argv[1], encoding="utf-8") as f:
    data = json.load(f)
for r in sorted(data["regions"], key=lambda x: (x.get("priority", 999), x["suffix"])):
    bbox = r["bbox"]
    margin = r.get("margin", 0.12)
    hwy = "1" if r.get("hwy_overlay") else "0"
    print(f'{r["suffix"]}\t{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}\t{margin}\t{hwy}\t{r["name"]}')
PY
}

STEP=0
TOTAL_STEPS=0
SCRIPT_START=$SECONDS

count_planned_steps() {
  TOTAL_STEPS=0
  while IFS=$'\t' read -r suffix bbox margin hwy name; do
    if ! want_region "${suffix}"; then
      continue
    fi
    if force_region "${suffix}" || [[ "${SKIP_EXISTING}" != "1" ]] || ! region_index_ready "${suffix}" || ! region_csr_ready "${suffix}"; then
      TOTAL_STEPS=$((TOTAL_STEPS + 2))
    fi
    if [[ "${hwy}" == "1" ]] && { force_region "${suffix}" || [[ "${SKIP_EXISTING}" != "1" ]] || ! region_hwy_ready "${suffix}"; }; then
      TOTAL_STEPS=$((TOTAL_STEPS + 2))
    fi
    if [[ "${BUILD_FULL_CH}" == "1" ]] && { force_region "${suffix}" || [[ "${SKIP_EXISTING}" != "1" ]] || ! region_full_ch_ready "${suffix}"; }; then
      TOTAL_STEPS=$((TOTAL_STEPS + 1))
    fi
  done < <(list_regions)
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
  echo "[skip] $*"
}

progress_done() {
  echo "[done] $1 (${2}s)"
}

build_hwy_overlay() {
  local suffix="$1"
  local bin
  bin="$(region_bin "${suffix}")"
  if [[ ! -x "${BUILD_HWY_CSR}" || ! -f "${bin}" ]]; then
    return 0
  fi
  if [[ "${SKIP_EXISTING}" == "1" ]] && ! force_region "${suffix}" && region_hwy_ready "${suffix}"; then
    progress_skip "${suffix}: hwy overlay OK"
    return 0
  fi
  local t0=$SECONDS
  progress_step "${suffix}: highway CSR"
  "${BUILD_HWY_CSR}" "${bin}"
  progress_done "${suffix} highway CSR" "$((SECONDS - t0))"

  if [[ -x "${BUILD_HWY_CH}" ]]; then
    t0=$SECONDS
    progress_step "${suffix}: highway CH"
    "${BUILD_HWY_CH}" "${bin}"
    progress_done "${suffix} highway CH" "$((SECONDS - t0))"
  fi
}

build_full_ch() {
  local suffix="$1"
  local bin
  bin="$(region_bin "${suffix}")"
  if [[ "${BUILD_FULL_CH}" != "1" ]]; then
    return 0
  fi
  if [[ ! -x "${BUILD_FULL_CH_BIN}" ]]; then
    echo "WARN: mmlp_build_full_ch missing; run cmake --build build --target mmlp_build_full_ch" >&2
    return 0
  fi
  if [[ "${SKIP_EXISTING}" == "1" ]] && ! force_region "${suffix}" && region_full_ch_ready "${suffix}"; then
    progress_skip "${suffix}: full.ch OK"
    return 0
  fi
  if ! region_csr_ready "${suffix}"; then
    echo "WARN: ${suffix} CSR missing, skip full.ch" >&2
    return 0
  fi
  local t0=$SECONDS
  progress_step "${suffix}: full CH (contraction, may take hours on large provinces)"
  "${BUILD_FULL_CH_BIN}" "${bin}"
  progress_done "${suffix} full.ch" "$((SECONDS - t0))"
}

build_region() {
  local suffix="$1"
  local bbox="$2"
  local margin="$3"
  local hwy="$4"
  local name="$5"
  local bin
  bin="$(region_bin "${suffix}")"

  if [[ "${SKIP_EXISTING}" == "1" ]] && ! force_region "${suffix}" && region_index_ready "${suffix}" && region_csr_ready "${suffix}"; then
    progress_skip "${suffix}(${name}): index + CSR present"
    if [[ "${hwy}" == "1" ]]; then
      build_hwy_overlay "${suffix}"
    fi
    build_full_ch "${suffix}"
    return 0
  fi

  if [[ -f "${bin}" ]] && region_index_ready "${suffix}" && ! region_csr_ready "${suffix}"; then
    progress_skip "${suffix}: bin + index present, CSR missing — building CSR only"
    if [[ -x "${BUILD_AUX}" ]]; then
      local t0=$SECONDS
      progress_step "${suffix}: build CSR (mmlp_build_aux --csr-only)"
      "${BUILD_AUX}" "${bin}" --csr-only
      progress_done "${suffix} CSR" "$((SECONDS - t0))"
    fi
    if [[ "${hwy}" == "1" ]]; then
      build_hwy_overlay "${suffix}"
    fi
    build_full_ch "${suffix}"
    return 0
  fi

  local t0=$SECONDS
  progress_step "${suffix}(${name}): extract subgraph bbox=${bbox}"
  python3 "${PY}" "${GRAPH}" "${suffix}" --bbox "${bbox}" --margin-deg "${margin}"
  progress_done "${suffix} extract" "$((SECONDS - t0))"

  if [[ -x "${BUILD_AUX}" && -f "${bin}" ]]; then
    t0=$SECONDS
    progress_step "${suffix}: spatial index + CSR"
    "${BUILD_AUX}" "${bin}" --csr-only
    progress_done "${suffix} index+CSR" "$((SECONDS - t0))"
  fi

  if [[ "${hwy}" == "1" ]]; then
    build_hwy_overlay "${suffix}"
  fi
  build_full_ch "${suffix}"
}

ALL_SUFFIXES="$(list_regions | cut -f1 | paste -sd, -)"

echo "=== Build regional graphs (provincial tiles) ==="
echo "Graph:       ${GRAPH}"
echo "Regions:     ${REGIONS} (all = ${ALL_SUFFIXES})"
echo "Skip:        SKIP_EXISTING=${SKIP_EXISTING} FORCE_REGIONS=${FORCE_REGIONS:-<none>}"
echo "Full CH:     BUILD_FULL_CH=${BUILD_FULL_CH}"
count_planned_steps
echo "Planned build steps: ${TOTAL_STEPS}"
echo ""

while IFS=$'\t' read -r suffix bbox margin hwy name; do
  if ! want_region "${suffix}"; then
    continue
  fi
  build_region "${suffix}" "${bbox}" "${margin}" "${hwy}" "${name}"
done < <(list_regions)

echo ""
echo "=== Regional graphs summary (+$((SECONDS - SCRIPT_START))s total) ==="
while IFS=$'\t' read -r suffix bbox margin hwy name; do
  if ! want_region "${suffix}"; then
    continue
  fi
  bin="$(region_bin "${suffix}")"
  if [[ -f "${bin}" ]]; then
    size="$(du -h "${bin}" | awk '{print $1}')"
    flags=""
    region_index_ready "${suffix}" && flags+=" index"
    region_csr_ready "${suffix}" && flags+=" csr"
    [[ "${hwy}" == "1" ]] && region_hwy_ready && flags+=" hwy"
    [[ "${BUILD_FULL_CH}" == "1" ]] && region_full_ch_ready "${suffix}" && flags+=" full.ch"
    echo "  ${suffix}(${name}): ${bin} (${size})${flags}"
  else
    echo "  ${suffix}(${name}): MISSING"
  fi
done < <(list_regions)
