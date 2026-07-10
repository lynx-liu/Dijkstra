#!/usr/bin/env bash
# One-shot bootstrap for HTTP service prerequisites (same outcome as a warm local host):
# - install python deps + C++ toolchain
# - build binaries
# - ensure nationwide graph + index sidecars
# - build nationwide Full CH (china.mmlp.full.ch) if missing — no manual copy
# - provincial Full CH only if FORCE_PROVINCIAL_CH=1 (not required when national exists)
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
  mmlp_build_full_ch full_ch_query_test -j"${BUILD_JOBS}"

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

# shellcheck source=/dev/null
source tools/china_regions_util.sh
graph_dir="$(dirname "${GRAPH}")"
graph_base="$(basename "${GRAPH}" .bin)"
if [[ "${graph_base}" == *.mmlp ]]; then
  graph_base="${graph_base%.mmlp}"
fi
# china.mmlp.bin -> china.mmlp.full.ch
NATIONAL_FULL_CH="${GRAPH%.bin}.full.ch"

echo "[5/7] Nationwide Full CH (required for same experience as local)..."
# Build on this host if missing — same as a first-time local machine, no copy step.
bash tools/build_national_full_ch.sh "${GRAPH}"
if [[ ! -f "${NATIONAL_FULL_CH}" ]]; then
  echo "ERROR: national Full CH missing after build: ${NATIONAL_FULL_CH}" >&2
  exit 1
fi
echo "[5/7] National Full CH ready: $(du -h "${NATIONAL_FULL_CH}" | cut -f1)"

if [[ "${FORCE_PROVINCIAL_CH:-0}" == "1" ]]; then
  echo "[5b/7] FORCE_PROVINCIAL_CH=1 — also building all provincial full.ch..."
  ensure_all_china_regions "${GRAPH}"
else
  echo "[5b/7] Skip provincial Full CH (national path is enough; set FORCE_PROVINCIAL_CH=1 to force)."
fi

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
if [[ ! -f "${NATIONAL_FULL_CH}" ]]; then
  echo "ERROR: national Full CH required: ${NATIONAL_FULL_CH}" >&2
  exit 1
fi

echo ""
echo "Bootstrap complete."
echo "  binaries: build/mmlp_service (+ aux / full_ch builders)"
echo "  graph:    ${GRAPH}"
echo "  national: ${NATIONAL_FULL_CH}"
echo ""
echo "Start HTTP service:"
echo "  MMLP_PRELOAD_REGIONS=off bash tools/start_http_server.sh"
echo ""
echo "Benchmark destination arrive (98-car Guangzhou case):"
echo "  python3 tools/bench_dest_arrive.py --label verify --runs 3"
echo ""

if [[ "${START_AFTER_BOOTSTRAP:-0}" == "1" ]]; then
  exec env MMLP_PRELOAD_REGIONS=off bash tools/start_http_server.sh
fi
