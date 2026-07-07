#!/usr/bin/env bash
# Start MMLP HTTP meeting service. Blocks until graph/index is ready.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/tools/env_runtime.sh"
PORT="${PORT:-8080}"
HOST="${HOST:-0.0.0.0}"
GRAPH="${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}"
BINARY="${ROOT}/build/mmlp_service"
LOAD_MODE="${MMLP_LOAD_MODE:-index}"
AUTO_BUILD_INDEX="${AUTO_BUILD_INDEX:-1}"
RESTART="${RESTART:-1}"
MMLP_WORKERS="${MMLP_WORKERS:-$(( $(nproc) > 2 ? $(nproc) - 2 : $(nproc) ))}"

if [[ ! -x "${BINARY}" ]]; then
  echo "ERROR: binaries not built. Run: bash tools/bootstrap_service.sh" >&2
  exit 1
fi
if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found. Run: bash tools/bootstrap_service.sh" >&2
  exit 1
fi

BASE="${GRAPH%.bin}"
index_ready() {
  [[ -f "${BASE}.sidx" && -f "${BASE}.nidx" && -f "${BASE}.eidx" ]]
}

regional_ready() {
  local graph_dir graph_base prd gd nx xj
  graph_dir="$(dirname "${GRAPH}")"
  graph_base="$(basename "${GRAPH}" .bin)"
  if [[ "${graph_base}" == *.mmlp ]]; then
    graph_base="${graph_base%.mmlp}"
  fi
  prd="${graph_dir}/${graph_base}_prd.mmlp.bin"
  gd="${graph_dir}/${graph_base}_gd.mmlp.bin"
  nx="${graph_dir}/${graph_base}_nx.mmlp.bin"
  xj="${graph_dir}/${graph_base}_xj.mmlp.bin"
  [[ -f "${prd}" && -f "${prd%.bin}.sidx" && -f "${prd%.bin}.hwy.csr" && -f "${prd%.bin}.hwy.ch" ]] &&
    [[ -f "${gd}" && -f "${gd%.bin}.sidx" && -f "${gd%.bin}.csr" ]] &&
    [[ -f "${nx}" && -f "${nx%.bin}.sidx" && -f "${nx%.bin}.csr" ]] &&
    [[ -f "${xj}" && -f "${xj%.bin}.sidx" && -f "${xj%.bin}.csr" ]]
}

if [[ "${LOAD_MODE}" == "index" ]] && ! index_ready; then
  if [[ "${AUTO_BUILD_INDEX}" == "1" ]]; then
    bash "${ROOT}/tools/ensure_graph_index.sh" "${GRAPH}"
  else
    echo "Index files missing. Run: bash tools/bootstrap_service.sh" >&2
    exit 1
  fi
fi

if [[ "${LOAD_MODE}" == "index" ]] && ! index_ready; then
  echo "ERROR: index build did not produce .sidx/.nidx/.eidx" >&2
  exit 1
fi

if [[ "${LOAD_MODE}" == "index" ]] && ! regional_ready; then
  echo "Regional graphs missing (prd/gd/nx/xj); building (first time may take 10-60 minutes)..."
  REGIONS="${REGIONS:-prd,gd,nx,xj}" bash "${ROOT}/tools/build_region_graphs.sh" "${GRAPH}"
fi

if [[ "${LOAD_MODE}" == "index" ]] && ! regional_ready; then
  echo "ERROR: regional graphs still missing after build (need prd+hwy, gd, nx, xj)." >&2
  echo "Run: bash tools/bootstrap_service.sh" >&2
  exit 1
fi

echo "=== MMLP HTTP service ==="
echo "Graph: ${GRAPH}"
MBTILES="${MMLP_MBTILES:-${ROOT}/data/map/china.mbtiles}"
if [[ -f "${MBTILES}" ]]; then
  echo "Map: ${MBTILES} (offline mbtiles)"
else
  echo "Map: graph render — install: bash tools/download_mbtiles.sh"
fi
echo "Load mode: ${LOAD_MODE}"
echo "Workers: ${MMLP_WORKERS}"
if [[ "${LOAD_MODE}" == "full" ]]; then
  echo "Startup: full graph in RAM (~5 minutes)"
else
  echo "Startup: spatial index only (usually under 1 minute)"
fi
echo "Listen: http://${HOST}:${PORT}"
echo "  GET  /health /map"
echo "  POST /api/vehicle /api/meetings/lead /api/destinations/arrive"
echo ""

port_in_use() {
  if command -v ss >/dev/null 2>&1; then
    ss -tln 2>/dev/null | grep -q ":${PORT} " && return 0
    return 1
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -i ":${PORT}" -sTCP:LISTEN >/dev/null 2>&1 && return 0
    return 1
  fi
  return 1
}

if port_in_use; then
  if [[ "${RESTART}" == "1" ]]; then
    echo "Restarting: stopping existing service on port ${PORT} ..."
    pkill -f mmlp_http_server.py 2>/dev/null || true
    pkill -f mmlp_service 2>/dev/null || true
    sleep 1
  elif pgrep -f "mmlp_http_server.py" >/dev/null 2>&1; then
    echo "ERROR: port ${PORT} already in use (mmlp HTTP probably already running)." >&2
    echo "  curl http://127.0.0.1:${PORT}/health" >&2
    echo "  To replace: RESTART=1 bash $0" >&2
    exit 1
  else
    echo "ERROR: port ${PORT} is in use by another process. Try PORT=8081 bash $0" >&2
    exit 1
  fi
fi

export MMLP_LOAD_MODE="${LOAD_MODE}"
export MMLP_WORKERS="${MMLP_WORKERS}"
exec python3 "${ROOT}/tools/mmlp_http_server.py" --host "${HOST}" --port "${PORT}" --graph "${GRAPH}" \
  --binary "${BINARY}" --load-mode "${LOAD_MODE}"
