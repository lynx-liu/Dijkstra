#!/usr/bin/env bash
# Ensure china.mmlp.{sidx,nidx,eidx} exist next to the graph bin (one-time build).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/tools/env_runtime.sh"
GRAPH="${1:-${MMLP_GRAPH_PATH:-${ROOT}/data/graph/china.mmlp.bin}}"

if [[ ! -f "${GRAPH}" ]]; then
  echo "ERROR: graph not found: ${GRAPH}" >&2
  exit 1
fi

BASE="${GRAPH%.bin}"
if [[ -f "${BASE}.sidx" && -f "${BASE}.nidx" && -f "${BASE}.eidx" ]]; then
  if [[ ! -f "${BASE}.egeo" ]]; then
    echo "=== Building .egeo (speeds corridor filter, ~1 minute) ==="
    if [[ ! -x "${ROOT}/build/mmlp_build_aux" ]]; then
      cmake -S "${ROOT}" -B "${ROOT}/build"
      cmake --build "${ROOT}/build" --target mmlp_build_aux
    fi
    "${ROOT}/build/mmlp_build_aux" "${GRAPH}" --egeo-only
  fi
  if [[ ! -f "${BASE}.hwy.csr" ]]; then
    echo "=== Building .hwy.csr (arterial overlay, ~2 minutes) ==="
    cmake --build "${ROOT}/build" --target mmlp_build_hwy_csr 2>/dev/null || {
      cmake -S "${ROOT}" -B "${ROOT}/build"
      cmake --build "${ROOT}/build" --target mmlp_build_hwy_csr
    }
    "${ROOT}/build/mmlp_build_hwy_csr" "${GRAPH}"
  fi
  if [[ ! -f "${BASE}.hwy.ch" ]] || [[ "$(stat -c%s "${BASE}.hwy.ch" 2>/dev/null || echo 0)" -lt 1000000 ]]; then
    echo "=== Building .hwy.ch (CH preprocess, ~10-30 minutes) ==="
    cmake --build "${ROOT}/build" --target mmlp_build_hwy_ch 2>/dev/null || {
      cmake -S "${ROOT}" -B "${ROOT}/build"
      cmake --build "${ROOT}/build" --target mmlp_build_hwy_ch
    }
    "${ROOT}/build/mmlp_build_hwy_ch" "${GRAPH}"
  fi
  if [[ ! -f "${BASE}.rtidx" ]]; then
    echo "=== Building .rtidx (partition tiles, ~3 minutes) ==="
    cmake --build "${ROOT}/build" --target mmlp_build_rtiles 2>/dev/null || {
      cmake -S "${ROOT}" -B "${ROOT}/build"
      cmake --build "${ROOT}/build" --target mmlp_build_rtiles
    }
    "${ROOT}/build/mmlp_build_rtiles" "${GRAPH}"
  fi
  echo "Index files OK:"
  ls -lh "${BASE}.sidx" "${BASE}.nidx" "${BASE}.eidx" "${BASE}.egeo" "${BASE}.hwy.csr" \
    "${BASE}.hwy.ch" "${BASE}.rtidx" 2>/dev/null || ls -lh "${BASE}.sidx" "${BASE}.nidx" "${BASE}.eidx"
  exit 0
fi

echo "=== Building graph index (one-time, about 5-8 minutes) ==="
echo "Graph: ${GRAPH}"
echo ""

if [[ ! -x "${ROOT}/build/mmlp_build_aux" ]]; then
  echo "Building mmlp_build_aux ..."
  cmake -S "${ROOT}" -B "${ROOT}/build"
  cmake --build "${ROOT}/build" --target mmlp_build_aux
fi

if [[ -x "${ROOT}/build/mmlp_build_aux" ]]; then
  if [[ -f "${BASE}.nidx" ]]; then
    "${ROOT}/build/mmlp_build_aux" "${GRAPH}" --skip-nidx
  else
    "${ROOT}/build/mmlp_build_aux" "${GRAPH}"
  fi
else
  python3 "${ROOT}/tools/build_graph_auxiliary.py" "${GRAPH}"
fi

echo ""
echo "Done. Index files:"
ls -lh "${BASE}.sidx" "${BASE}.nidx" "${BASE}.eidx"
