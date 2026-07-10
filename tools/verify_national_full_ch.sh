#!/usr/bin/env bash
# Verify national Full CH destination routing:
# same-province, cross-province, multi-vehicle, first click + change destination.
# Requires warm mmlp_service on BASE (default http://127.0.0.1:8080).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BASE="${MMLP_BASE:-http://127.0.0.1:8080}"
OUT_DIR="${ROOT}/logs/verify_national_full_ch"
mkdir -p "${OUT_DIR}"

VEHICLE_TIME=$(date -u -d '5 minutes ago' +%Y-%m-%dT%H:%M:%SZ)
# Cross-country needs a long horizon (weeks).
ARRIVE_BY=$(date -u -d "${VEHICLE_TIME} + 30 days" +%Y-%m-%dT%H:%M:%SZ)

# Multi-vehicle fleet: GD local + NX/SD/XJ/SC remotes (and a few more for batch).
FLEET_JSON="${OUT_DIR}/fleet.json"
python3 - "${VEHICLE_TIME}" <<'PY' > "${FLEET_JSON}"
import json, sys
vt = sys.argv[1]
vehicles = [
    # Guangdong (same-province when dest=GZ)
    {"id": "粤A10001", "lat": 23.13, "lon": 113.26, "speed": 65, "time": vt},
    {"id": "粤B20002", "lat": 22.55, "lon": 114.05, "speed": 68, "time": vt},
    # Ningxia / Shandong / Xinjiang / Sichuan / Heilongjiang (cross-province)
    {"id": "宁AS6269", "lat": 38.47, "lon": 106.27, "speed": 70, "time": vt},
    {"id": "鲁C30003", "lat": 36.67, "lon": 117.00, "speed": 72, "time": vt},
    {"id": "新A40004", "lat": 43.83, "lon": 87.62, "speed": 75, "time": vt},
    {"id": "川A50005", "lat": 30.67, "lon": 104.06, "speed": 70, "time": vt},
    {"id": "黑A60006", "lat": 45.75, "lon": 126.65, "speed": 68, "time": vt},
    {"id": "滇A70007", "lat": 25.04, "lon": 102.71, "speed": 70, "time": vt},
]
print(json.dumps(vehicles, ensure_ascii=False))
PY

analyze() {
  local path="$1" ms="$2" label="$3"
  python3 - "${path}" "${ms}" "${label}" <<'PY'
import json, math, sys
path, ms, label = sys.argv[1], int(sys.argv[2]), sys.argv[3]
d = json.load(open(path))
if "error" in d and not d.get("vehicles"):
    print(f"  ERROR {label}: {d.get('error')}")
    sys.exit(3)
rows = d.get("vehicles") or []
ok = bad = 0
def hav(a, b):
    R = 6371000.0
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dphi = p2 - p1
    dl = math.radians(b[1] - a[1])
    x = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(x)))
def ll(p):
    if isinstance(p, dict):
        return (float(p["lat"]), float(p["lon"]))
    return (float(p[0]), float(p[1]))
for r in rows:
    route = r.get("route")
    pts = []
    if isinstance(route, list):
        pts = route
    elif isinstance(route, dict):
        pts = route.get("points") or []
    n = len(pts)
    max_seg = 0.0
    for i in range(1, n):
        max_seg = max(max_seg, hav(ll(pts[i - 1]), ll(pts[i])))
    drawable = n >= 8 and max_seg <= 50000.0
    vid = r.get("vehicleId") or r.get("id")
    dist = r.get("routeDistanceM")
    if drawable:
        ok += 1
        print(f"  OK  {vid} pts={n} maxSeg_km={max_seg/1000:.1f} dist_km={(dist or 0)/1000:.0f}")
    else:
        bad += 1
        print(f"  BAD {vid} pts={n} maxSeg_km={max_seg/1000:.1f} dist_km={(dist or 0)/1000:.0f}")
print(f"  summary reachable={len(rows)} drawable_ok={ok} bad={bad} latency_ms={ms}")
if ms > 1000:
    print("  WARN: latency > 1000ms (warm target <1s)")
if bad or len(rows) == 0:
    sys.exit(2)
PY
}

post_dest() {
  local name="$1" lat="$2" lon="$3" label="$4"
  local payload
  payload=$(python3 - "${lat}" "${lon}" "${ARRIVE_BY}" "${FLEET_JSON}" <<'PY'
import json, sys
lat, lon, ab, fleet_path = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], sys.argv[4]
vehicles = json.load(open(fleet_path))
print(json.dumps({
    "lat": lat, "lon": lon, "arriveBy": ab, "sortBy": "duration",
    "vehicles": vehicles,
}, ensure_ascii=False, separators=(",", ":")))
PY
)
  local t0 t1 ms code
  t0=$(date +%s%3N)
  code=$(curl -sS -o "${OUT_DIR}/${name}.json" -w "%{http_code}" \
    -H 'Content-Type: application/json' \
    -X POST "${BASE}/api/destinations/arrive" \
    --data "${payload}" || echo "000")
  t1=$(date +%s%3N)
  ms=$((t1 - t0))
  echo "[${label}] http=${code} ms=${ms} -> ${OUT_DIR}/${name}.json"
  analyze "${OUT_DIR}/${name}.json" "${ms}" "${label}"
}

echo "vehicle_time=${VEHICLE_TIME} arrive_by=${ARRIVE_BY}"
echo "=== health ==="
curl -sS "${BASE}/healthz" 2>/dev/null || curl -sS "${BASE}/api/map/state" 2>/dev/null | head -c 200 || true
echo
echo

echo "=== 1) FIRST dest: Guangzhou (same-province for GD trucks) ==="
post_dest "01_first_gz" 23.1291 113.2644 "first_gz"

echo "=== 2) CHANGE dest: Beijing (cross-province for all) ==="
post_dest "02_change_bj" 39.9042 116.4074 "change_bj"

echo "=== 3) CHANGE dest: Hefei ==="
post_dest "03_change_hf" 31.8206 117.2272 "change_hf"

echo "=== 4) CHANGE dest: Urumqi (long haul) ==="
post_dest "04_change_xj" 43.8256 87.6168 "change_xj"

echo "=== 5) REPEAT Beijing (warm second hit) ==="
post_dest "05_repeat_bj" 39.9042 116.4074 "repeat_bj"

echo "=== done; artifacts in ${OUT_DIR} ==="
