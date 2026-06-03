#!/usr/bin/env bash
# Smoke test: meetings/lead — simulated Xinjiang plates, near to ~500km.
set -euo pipefail

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8080}"
BASE="${BASE_URL:-http://${HOST}:${PORT}}"

VEHICLE_TIME=$(date -u -d '5 minutes ago' +%Y-%m-%dT%H:%M:%SZ)
echo "vehicle time: ${VEHICLE_TIME}" >&2
echo "focal: 新A5D107 (基准车，城区约15km)" >&2
echo >&2

python3 - "${VEHICLE_TIME}" <<'PY' | curl -s -X POST "${BASE}/api/meetings/lead" \
  -H 'Content-Type: application/json' \
  -d @- | python3 -m json.tool
import json, sys
vt = sys.argv[1]
rows = [
    ("新A5D107", 43.9055, 87.4561, 72),
    ("新A8K231", 43.918,  87.475,  65),
    ("新A3M892", 43.915,  87.528,  68),
    ("新A2H668", 43.65,   87.50,   70),
    ("新A1L335", 43.92,   86.25,   75),
    ("新A6C778", 43.92,   88.75,   72),
    ("新A4N902", 42.95,   89.18,   78),
    ("新A7T281", 42.35,   89.85,   80),
    ("新A5X640", 41.76,   86.17,   82),
]
print(json.dumps({
    "vehicles": [
        {"id": vid, "lat": lat, "lon": lon, "speed": spd, "time": vt}
        for vid, lat, lon, spd in rows
    ],
}, separators=(",", ":"), ensure_ascii=False))
PY
