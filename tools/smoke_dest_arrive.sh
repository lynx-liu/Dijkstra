#!/usr/bin/env bash
# Smoke test: destination arrival — simulated plates, near to ~500km.
set -euo pipefail

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8080}"
BASE="${BASE_URL:-http://${HOST}:${PORT}}"

VEHICLE_TIME=$(date -u -d '5 minutes ago' +%Y-%m-%dT%H:%M:%SZ)
# 远途（数百公里）需更长窗口；+24h
ARRIVE_BY=$(date -u -d "${VEHICLE_TIME} + 24 hours" +%Y-%m-%dT%H:%M:%SZ)

echo "vehicle time : ${VEHICLE_TIME}" >&2
echo "arrive by    : ${ARRIVE_BY} (+24h)" >&2
echo "destination  : 43.92, 87.50 (乌鲁木齐)" >&2
echo >&2

START=$(date +%s.%N)
python3 - "${VEHICLE_TIME}" "${ARRIVE_BY}" <<'PY' | curl -s -X POST "${BASE}/api/destinations/arrive" \
  -H 'Content-Type: application/json' \
  -d @- | python3 -m json.tool
import json, sys
vt, ab = sys.argv[1], sys.argv[2]
# (id, 直线距目的地约, lat, lon, speed_kmh)
vehicles = [
    ("新A8K231",  "3km",    43.918,  87.475,  65),
    ("新A3M892",  "5km",    43.915,  87.528,  68),
    ("新A5D107",  "15km",   43.9055, 87.4561, 72),
    ("新A2H668",  "30km",   43.65,   87.50,   70),
    ("新A9R004",  "50km",   43.47,   87.498,  68),
    ("新A1L335",  "100km",  43.92,   86.25,   75),
    ("新A6C778",  "100km",  43.92,   88.75,   72),
    ("新A0B519",  "100km",  43.02,   87.50,   70),
    ("新A4N902",  "180km",  42.95,   89.18,   78),
    ("新A7T281",  "280km",  42.35,   89.85,   80),
    ("新A5X640",  "380km",  41.76,   86.17,   82),
    ("新A3Q118",  "480km",  41.15,   85.55,   85),
]
print(json.dumps({
    "lat": 43.92, "lon": 87.50,
    "arriveBy": ab, "sortBy": "distance",
    "vehicles": [
        {"id": vid, "lat": lat, "lon": lon, "speed": spd, "time": vt}
        for vid, _, lat, lon, spd in vehicles
    ],
}, separators=(",", ":"), ensure_ascii=False))
PY
END=$(date +%s.%N)
python3 -c "import sys; s,e=sys.argv[1:3]; print(f'elapsed={float(e)-float(s):.2f}s (HTTP+curl+json.tool)', file=sys.stderr)" "$START" "$END"
