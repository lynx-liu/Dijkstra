#!/usr/bin/env bash
# One-shot bootstrap for HTTP service prerequisites.
# - install python deps
# - build binaries
# - ensure nationwide graph exists
# - ensure index files exist for fast startup
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GRAPH="${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}"
AUTO_DEPLOY_GRAPH="${AUTO_DEPLOY_GRAPH:-1}"   # 1: build graph when missing
START_AFTER_BOOTSTRAP="${START_AFTER_BOOTSTRAP:-0}"  # 1: start server immediately

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

echo "[1/5] Install Python dependencies..."
bash tools/install_deps.sh

echo "[2/5] Build binaries..."
if [[ -x build/mmlp_service ]] && [[ -x build/mmlp_build_aux ]] && [[ "${FORCE_REBUILD:-0}" != "1" ]]; then
  echo "[2/5] Binaries exist, skip rebuild."
else
  bash tools/install_build_deps.sh
  # shellcheck source=/dev/null
  source tools/env_build.sh
  rm -rf build
  "${CMAKE}" -S . -B build
  "${CMAKE}" --build build --target mmlp_service mmlp_build_aux
fi

if [[ ! -f "${GRAPH}" ]]; then
  if [[ "${AUTO_DEPLOY_GRAPH}" == "1" ]]; then
    echo "[3/5] Graph file missing, deploying nationwide graph..."
    bash tools/deploy_graph_nationwide.sh
  else
    echo "ERROR: graph not found: ${GRAPH}" >&2
    echo "Set AUTO_DEPLOY_GRAPH=1 or run: bash tools/deploy_graph_nationwide.sh" >&2
    exit 1
  fi
else
  echo "[3/5] Graph file exists, skip deploy."
fi

echo "[4/5] Ensure fast index files..."
bash tools/ensure_graph_index.sh "${GRAPH}"

echo "[5/6] Fetch map page vendor (Leaflet + MapLibre + glyphs, offline)..."
bash tools/fetch_web_vendor.sh
bash tools/fetch_map_vendor.sh
bash tools/fetch_map_glyphs.sh

echo "[6/6] Offline map tiles (optional, ~2 GB)..."
if [[ "${AUTO_DOWNLOAD_MBTILES:-0}" == "1" ]]; then
  bash tools/download_mbtiles.sh
else
  if [[ -f "${ROOT}/data/map/china.mbtiles" ]]; then
    echo "[6/6] china.mbtiles present."
  else
    echo "[6/6] Skip mbtiles (set AUTO_DOWNLOAD_MBTILES=1 or run: bash tools/download_mbtiles.sh)"
  fi
fi

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found. Run: bash tools/bootstrap_service.sh" >&2
  exit 1
fi

echo ""
echo "Bootstrap complete."
echo "Start: bash tools/start_http_server.sh"
echo ""

if [[ "${START_AFTER_BOOTSTRAP}" == "1" ]]; then
  exec bash tools/start_http_server.sh
fi
