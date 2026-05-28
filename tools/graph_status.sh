#!/usr/bin/env bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/config/osm.defaults.env"

echo "=== MMLP map data status ==="
PBF="${ROOT}/${PBF_PATH}"
BIN="${ROOT}/${GRAPH_PATH}"

if [[ -f "${PBF}" ]]; then
  echo "OSM PBF:  $(ls -lh "${PBF}" | awk '{print $5, $9}')"
else
  echo "OSM PBF:  (missing) ${PBF}"
fi

if [[ -f "${BIN}" ]]; then
  echo "Graph:    $(ls -lh "${BIN}" | awk '{print $5, $9}')"
  python3 -c "
import struct, sys
p=sys.argv[1]
with open(p,'rb') as f:
    m=f.read(8); v=struct.unpack('<I', f.read(4))[0]
    n,e=struct.unpack('<QQ', f.read(16))
    print(f'  version={v} nodes={n:,} edges={e:,}')
" "${BIN}" 2>/dev/null || true
else
  echo "Graph:    (missing) ${BIN}"
fi

if [[ -f "${ROOT}/${DATA_DIR}/deploy.log" ]]; then
  echo "--- last log lines ---"
  tail -3 "${ROOT}/${DATA_DIR}/deploy.log"
fi
