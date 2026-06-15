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

bash tools/check_python.sh
echo ""

echo "[1/5] Install Python dependencies..."
bash tools/install_deps.sh

echo "[2/5] Build binaries..."
cmake -S . -B build
cmake --build build --target mmlp_service mmlp_build_aux

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

echo "[5/5] Fetch map page vendor (Leaflet, for offline/restricted network)..."
bash tools/fetch_web_vendor.sh

echo ""
echo "Bootstrap complete."
echo "Start service with:"
echo "  bash tools/start_http_server.sh"
echo ""

if [[ "${START_AFTER_BOOTSTRAP}" == "1" ]]; then
  exec bash tools/start_http_server.sh
fi
