#!/usr/bin/env bash
# Print python3 version and exit non-zero if older than 3.6.
set -euo pipefail
ver=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")')
major=$(python3 -c 'import sys; print(sys.version_info.major)')
minor=$(python3 -c 'import sys; print(sys.version_info.minor)')
echo "python3: ${ver}"
if [[ "${major}" -lt 3 ]] || [[ "${major}" -eq 3 && "${minor}" -lt 6 ]]; then
  echo "ERROR: Python 3.6+ required." >&2
  exit 1
fi
