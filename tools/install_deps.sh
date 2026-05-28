#!/usr/bin/env bash
set -euo pipefail
pip3 install -r "$(cd "$(dirname "$0")" && pwd)/requirements.txt"
if ! command -v osmium >/dev/null 2>&1; then
  echo "[install_deps] optional: apt install osmium-tool  (for faster bbox clip)"
fi
