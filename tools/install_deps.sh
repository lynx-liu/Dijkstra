#!/usr/bin/env bash
# Python deps for tools (Python 3.6+).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

py_major=$(python3 -c 'import sys; print(sys.version_info.major)')
py_minor=$(python3 -c 'import sys; print(sys.version_info.minor)')
echo "[install_deps] python3: $(python3 --version 2>&1)"
if [[ "${py_major}" -lt 3 ]] || [[ "${py_major}" -eq 3 && "${py_minor}" -lt 6 ]]; then
  echo "ERROR: Python 3.6+ required." >&2
  exit 1
fi

python3 -m pip install --upgrade 'pip>=20' 'setuptools>=40' 'wheel' 'packaging'

echo "[install_deps] pip install -r requirements.txt"
python3 -m pip install -r "${ROOT}/requirements.txt"
