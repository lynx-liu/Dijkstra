#!/usr/bin/env bash
# Nationwide only: clear regional overrides and run full China deploy.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export BBOX=
export USE_OSMIUM_EXTRACT=0
exec bash "${ROOT}/tools/deploy_graph.sh" "$@"
