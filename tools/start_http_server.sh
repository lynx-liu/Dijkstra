#!/usr/bin/env bash
# Start MMLP HTTP meeting service. Blocks until graph/index is ready.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${PORT:-8080}"
HOST="${HOST:-0.0.0.0}"
GRAPH="${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}"
BINARY="${ROOT}/build/mmlp_service"
LOAD_MODE="${MMLP_LOAD_MODE:-index}"
# Set to 0 to fail instead of auto-building missing index files.
AUTO_BUILD_INDEX="${AUTO_BUILD_INDEX:-1}"

if [[ ! -x "${BINARY}" ]]; then
  echo "Build first: cmake -S ${ROOT} -B ${ROOT}/build && cmake --build ${ROOT}/build --target mmlp_service mmlp_build_aux"
  exit 1
fi
if [[ ! -f "${GRAPH}" ]]; then
  echo "Missing graph: ${GRAPH}"
  echo "Run: bash ${ROOT}/tools/deploy_graph_nationwide.sh"
  exit 1
fi

BASE="${GRAPH%.bin}"
index_ready() {
  [[ -f "${BASE}.sidx" && -f "${BASE}.nidx" && -f "${BASE}.eidx" ]]
}

if [[ "${LOAD_MODE}" == "index" ]] && ! index_ready; then
  if [[ "${AUTO_BUILD_INDEX}" == "1" ]]; then
    bash "${ROOT}/tools/ensure_graph_index.sh" "${GRAPH}"
  else
    echo "Index files missing for: ${GRAPH}"
    echo "Run once:"
    echo "  bash ${ROOT}/tools/ensure_graph_index.sh"
    echo "Or:"
    echo "  MMLP_LOAD_MODE=full bash $0"
    exit 1
  fi
fi

if [[ "${LOAD_MODE}" == "index" ]] && ! index_ready; then
  echo "ERROR: index build did not produce .sidx/.nidx/.eidx" >&2
  exit 1
fi

echo "=== MMLP HTTP service ==="
echo "Graph: ${GRAPH}"
echo "Load mode: ${LOAD_MODE}"
if [[ "${LOAD_MODE}" == "full" ]]; then
  echo "Startup: full graph in RAM (~5 minutes)"
else
  echo "Startup: spatial index only (usually under 1 minute)"
fi
echo "Listen: http://${HOST}:${PORT}  (GET /health, POST /api/vehicle, POST /api/meetings/lead)"
echo ""
export MMLP_LOAD_MODE="${LOAD_MODE}"
exec python3 "${ROOT}/tools/mmlp_http_server.py" --host "${HOST}" --port "${PORT}" --graph "${GRAPH}" --binary "${BINARY}" --load-mode "${LOAD_MODE}"
