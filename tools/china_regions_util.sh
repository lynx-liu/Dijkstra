#!/usr/bin/env bash
# Shared helpers for provincial graph / full.ch readiness (safe to source).

china_regions_json() {
  local root="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
  echo "${root}/config/china_regions.json"
}

# Print: ready missing_bin missing_ch total
china_regions_counts() {
  local graph="${1:?graph path required}"
  local json
  json="$(china_regions_json)"
  python3 - "${graph}" "${json}" <<'PY'
import json, os, sys
graph, cfg = sys.argv[1], sys.argv[2]
root = os.path.dirname(graph)
base = os.path.basename(graph)
if base.endswith(".bin"):
    base = base[:-4]
if base.endswith(".mmlp"):
    base = base[:-5]
with open(cfg, encoding="utf-8") as f:
    regions = [r["suffix"] for r in json.load(f)["regions"]]
ready = missing_bin = missing_ch = 0
for s in regions:
    binf = f"{root}/{base}_{s}.mmlp.bin"
    ch = f"{root}/{base}_{s}.mmlp.full.ch"
    if os.path.isfile(binf) and os.path.isfile(ch):
        ready += 1
    elif os.path.isfile(binf):
        missing_ch += 1
    else:
        missing_bin += 1
print(ready, missing_bin, missing_ch, len(regions))
PY
}

china_regions_all_ready() {
  local graph="$1"
  local counts ready missing_bin missing_ch total
  read -r ready missing_bin missing_ch total <<<"$(china_regions_counts "${graph}")"
  [[ "${ready}" -eq "${total}" ]]
}

china_regions_print_missing() {
  local graph="${1:?graph path required}"
  local json
  json="$(china_regions_json)"
  python3 - "${graph}" "${json}" <<'PY'
import json, os, sys
graph, cfg = sys.argv[1], sys.argv[2]
root = os.path.dirname(graph)
base = os.path.basename(graph)
if base.endswith(".bin"):
    base = base[:-4]
if base.endswith(".mmlp"):
    base = base[:-5]
with open(cfg, encoding="utf-8") as f:
    regions = json.load(f)["regions"]
for r in sorted(regions, key=lambda x: x["suffix"]):
    s = r["suffix"]
    binf = f"{root}/{base}_{s}.mmlp.bin"
    ch = f"{root}/{base}_{s}.mmlp.full.ch"
    if os.path.isfile(binf) and os.path.isfile(ch):
        continue
    if not os.path.isfile(binf):
        print(f"  {s}({r['name']}): missing graph")
    else:
        print(f"  {s}({r['name']}): missing full.ch")
PY
}

ensure_all_china_regions() {
  local graph="${1:?graph path required}"
  local root
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if china_regions_all_ready "${graph}"; then
    local counts ready _b _c total
    read -r ready _b _c total <<<"$(china_regions_counts "${graph}")"
    echo "[regions] all ${total} provincial graphs + full.ch ready"
    return 0
  fi
  echo "[regions] building missing provincial graphs + full.ch (REGIONS=all BUILD_FULL_CH=1)..."
  china_regions_print_missing "${graph}" >&2 || true
  REGIONS="${REGIONS:-all}" BUILD_FULL_CH="${BUILD_FULL_CH:-1}" SKIP_EXISTING="${SKIP_EXISTING:-1}" \
    FORCE_REGIONS="${FORCE_REGIONS:-}" \
    bash "${root}/tools/build_region_graphs.sh" "${graph}"
  if ! china_regions_all_ready "${graph}"; then
    echo "ERROR: not all provincial full.ch sidecars are ready:" >&2
    china_regions_print_missing "${graph}" >&2
    return 1
  fi
}
