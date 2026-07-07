#!/usr/bin/env bash
# One-shot bootstrap for HTTP service prerequisites.
# - install python deps
# - build binaries (service + graph/overlay builders)
# - ensure nationwide graph + index sidecars
# - ensure regional graphs (PRD hwy overlay for destination arrive)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GRAPH="${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}"
AUTO_DEPLOY_GRAPH="${AUTO_DEPLOY_GRAPH:-1}"   # 1: build graph when missing
BUILD_JOBS="${MMLP_BUILD_JOBS:-12}"

echo "=== MMLP bootstrap ==="
echo "Root:  ${ROOT}"
echo "Graph: ${GRAPH}"
echo ""

cd "${ROOT}"

if [[ $(id -u) -eq 0 ]] && [[ -f /etc/redhat-release ]]; then
  bash tools/fix_centos7_yum.sh
fi

bash tools/check_python.sh
echo ""

echo "[1/7] Install Python dependencies..."
bash tools/install_deps.sh

echo "[2/7] Build binaries (incremental)..."
bash tools/install_build_deps.sh
# shellcheck source=/dev/null
source tools/env_build.sh
if [[ ! -d build ]]; then
  "${CMAKE}" -S . -B build
fi
"${CMAKE}" --build build --target mmlp_service mmlp_build_aux mmlp_build_hwy_csr mmlp_build_hwy_ch \
  -j"${BUILD_JOBS}"

if [[ ! -f "${GRAPH}" ]]; then
  if [[ "${AUTO_DEPLOY_GRAPH}" == "1" ]]; then
    echo "[3/7] Graph file missing, deploying nationwide graph..."
    bash tools/deploy_graph_nationwide.sh
  else
    echo "ERROR: graph not found: ${GRAPH}" >&2
    echo "Set AUTO_DEPLOY_GRAPH=1 or run: bash tools/deploy_graph_nationwide.sh" >&2
    exit 1
  fi
else
  echo "[3/7] Graph file exists, skip deploy."
fi

echo "[4/7] Ensure nationwide index sidecars..."
bash tools/ensure_graph_index.sh "${GRAPH}"

echo "[5/7] Ensure regional graphs (prd + gd + nx + xj, skip if already built)..."
REGIONS="${REGIONS:-prd,gd,nx,xj}" bash tools/build_region_graphs.sh "${GRAPH}"

echo "[6/7] Fetch map page vendor (Leaflet + MapLibre + glyphs, offline)..."
bash tools/fetch_web_vendor.sh
bash tools/fetch_map_vendor.sh
bash tools/fetch_map_glyphs.sh

echo "[7/7] Offline map tiles (optional, ~2 GB)..."
if [[ "${AUTO_DOWNLOAD_MBTILES:-0}" == "1" ]]; then
  bash tools/download_mbtiles.sh
else
  if [[ -f "${ROOT}/data/map/china.mbtiles" ]]; then
    echo "[7/7] china.mbtiles present."
  else
    echo "[7/7] Skip mbtiles (set AUTO_DOWNLOAD_MBTILES=1 or run: bash tools/download_mbtiles.sh)"
  fi
fi

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found after bootstrap: ${GRAPH}" >&2
  exit 1
fi

graph_dir="$(dirname "${GRAPH}")"
graph_base="$(basename "${GRAPH}" .bin)"
if [[ "${graph_base}" == *.mmlp ]]; then
  graph_base="${graph_base%.mmlp}"
fi
PRD_BIN="${graph_dir}/${graph_base}_prd.mmlp.bin"
GD_BIN="${graph_dir}/${graph_base}_gd.mmlp.bin"
NX_BIN="${graph_dir}/${graph_base}_nx.mmlp.bin"
XJ_BIN="${graph_dir}/${graph_base}_xj.mmlp.bin"
if [[ ! -f "${PRD_BIN}" || ! -f "${PRD_BIN%.bin}.hwy.ch" ]]; then
  echo "ERROR: PRD regional graph or hwy overlay missing." >&2
  echo "  expected: ${PRD_BIN}" >&2
  echo "  expected: ${PRD_BIN%.bin}.hwy.ch" >&2
  echo "Re-run with: SKIP_EXISTING=0 REGIONS=prd,gd,nx,xj bash tools/build_region_graphs.sh" >&2
  exit 1
fi
if [[ ! -f "${GD_BIN}" || ! -f "${GD_BIN%.bin}.csr" ]]; then
  echo "ERROR: GD regional graph missing (needed for Guangdong province)." >&2
  echo "  expected: ${GD_BIN}" >&2
  echo "Re-run with: SKIP_EXISTING=0 REGIONS=gd bash tools/build_region_graphs.sh" >&2
  exit 1
fi
if [[ ! -f "${NX_BIN}" || ! -f "${NX_BIN%.bin}.csr" ]]; then
  echo "ERROR: NX regional graph missing (needed for Ningxia/Inner Mongolia)." >&2
  echo "  expected: ${NX_BIN}" >&2
  echo "Re-run with: SKIP_EXISTING=0 REGIONS=nx bash tools/build_region_graphs.sh" >&2
  exit 1
fi
if [[ ! -f "${XJ_BIN}" || ! -f "${XJ_BIN%.bin}.sidx" ]]; then
  echo "ERROR: XJ regional graph missing (needed for Xinjiang)." >&2
  echo "  expected: ${XJ_BIN}" >&2
  echo "Re-run with: SKIP_EXISTING=0 REGIONS=xj bash tools/build_region_graphs.sh" >&2
  exit 1
fi

echo ""
echo "Bootstrap complete."
echo "  binaries: build/mmlp_service build/mmlp_build_aux build/mmlp_build_hwy_csr build/mmlp_build_hwy_ch"
echo "  graph:    ${GRAPH}"
echo "  regional: ${PRD_BIN} (+ hwy), ${GD_BIN}, ${NX_BIN}, ${XJ_BIN}"
echo ""
echo "Start HTTP service:"
echo "  bash tools/start_http_server.sh"
echo ""
echo "Benchmark destination arrive (98-car Guangzhou case):"
echo "  python3 tools/bench_dest_arrive.py --label verify --runs 3"
echo ""

if [[ "${START_AFTER_BOOTSTRAP:-0}" == "1" ]]; then
  exec bash tools/start_http_server.sh
fi
