#!/usr/bin/env bash
# Download Noto Sans glyph PBFs for offline Chinese place labels on vector mbtiles.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GLYPH_DIR="${ROOT}/web/vendor/map/glyphs/Noto Sans Regular"
GLYPH_BASE="https://demotiles.maplibre.org/font/Noto%20Sans%20Regular"

mkdir -p "${GLYPH_DIR}"

if [[ -f "${GLYPH_DIR}/0-255.pbf" && -f "${GLYPH_DIR}/8192-8447.pbf" ]]; then
  echo "Map glyphs already present: ${GLYPH_DIR}"
  exit 0
fi

echo "Fetching Noto Sans glyph ranges -> ${GLYPH_DIR}"

ranges=(
  "0-255" "256-511" "512-767" "768-1023" "1024-1279" "1280-1535"
  "1536-1791" "1792-2047" "2048-2303" "2304-2559" "2560-2815" "2816-3071"
  "3072-3327" "3328-3583" "3584-3839" "3840-4095" "4096-4351" "4352-4607"
  "4608-4863" "4864-5119" "5120-5375" "5376-5631" "5632-5887" "5888-6143"
  "6144-6399" "6400-6655" "6656-6911" "6912-7167" "7168-7423" "7424-7679"
  "7680-7935" "7936-8191" "8192-8447"
)

for r in "${ranges[@]}"; do
  out="${GLYPH_DIR}/${r}.pbf"
  if [[ -f "${out}" ]]; then
    continue
  fi
  curl -fsSL --connect-timeout 15 --max-time 60 -o "${out}" "${GLYPH_BASE}/${r}.pbf"
done

echo "Glyphs ready ($(ls -1 "${GLYPH_DIR}" | wc -l) ranges)."
