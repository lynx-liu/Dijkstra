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
  # shellcheck source=/dev/null
  source "${ROOT}/tools/china_regions_util.sh"
  china_regions_all_ready "${GRAPH}"
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

# National dense Full CH is required for the same interactive experience as local.
# If missing, build it here (no manual copy). Provincial sidecars are optional fallback only.
if [[ "${LOAD_MODE}" == "index" ]] && [[ ! -f "${BASE}.full.ch" ]]; then
  echo "National Full CH missing; building china.mmlp.full.ch (often ~10–20 min)..."
  bash "${ROOT}/tools/build_national_full_ch.sh" "${GRAPH}"
fi

if [[ "${LOAD_MODE}" == "index" ]] && [[ ! -f "${BASE}.full.ch" ]] && ! regional_ready; then
  echo "ERROR: national Full CH build failed and provincial sidecars are incomplete." >&2
  # shellcheck source=/dev/null
  source "${ROOT}/tools/china_regions_util.sh"
  china_regions_print_missing "${GRAPH}" >&2
  echo "Re-run: bash tools/bootstrap_service.sh" >&2
  exit 1
fi
if [[ -f "${BASE}.full.ch" ]]; then
  echo "National Full CH: ${BASE}.full.ch ($(du -h "${BASE}.full.ch" | cut -f1))"
fi

echo "=== MMLP HTTP service ==="
echo "Graph: ${GRAPH}"
# shellcheck source=/dev/null
source "${ROOT}/config/map.defaults.env" 2>/dev/null || true
if [[ -f "${ROOT}/data/map/china_central_asia.mbtiles" ]]; then
  echo "Map: data/map/china_central_asia.mbtiles (China + Central Asia)"
elif [[ -d "${ROOT}/data/map/central_asia" ]] && compgen -G "${ROOT}/data/map/central_asia/*.mbtiles" >/dev/null; then
  echo "Map: china.mbtiles + central_asia/*.mbtiles overlays"
elif [[ -f "${ROOT}/data/map/china.mbtiles" ]]; then
  echo "Map: data/map/china.mbtiles (China only — run: bash tools/download_mbtiles.sh for 中亚)"
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
echo "Provincial CH: all regions preloaded at startup (set MMLP_PRELOAD_REGIONS=off to lazy-load)"
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
