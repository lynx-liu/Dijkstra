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
  # Do NOT delete .csr here: bootstrap runs build_national_full_ch next, and
  # wiping CSR forces a multi-minute mmlp_build_aux --csr-only every time.
  # Legacy partition cache is still safe to drop.
  rm -f "${BASE}.rtidx"
  echo "Index files OK:"
  ls -lh "${BASE}.sidx" "${BASE}.nidx" "${BASE}.eidx" "${BASE}.egeo" 2>/dev/null \
    || ls -lh "${BASE}.sidx" "${BASE}.nidx" "${BASE}.eidx"
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
