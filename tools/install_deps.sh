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

pip3 install --upgrade 'pip>=20' 'setuptools>=40' 'wheel' 'packaging'

install_rhel_build_deps() {
  # pyosmium ships manylinux wheels; skip yum gcc/cmake here (see install_build_deps.sh).
  local pkg=yum
  command -v dnf >/dev/null 2>&1 && pkg=dnf
  if [[ $(id -u) -ne 0 ]]; then
    return 0
  fi
  ${pkg} install -y python3-devel 2>/dev/null || true
}

install_debian_build_deps() {
  if command -v apt-get >/dev/null 2>&1 && [[ $(id -u) -eq 0 ]]; then
    apt-get install -y python3-dev g++ cmake libboost-dev libexpat1-dev zlib1g-dev libbz2-dev \
      2>/dev/null || true
  fi
}

if [[ -f /etc/redhat-release ]]; then
  install_rhel_build_deps
elif [[ -f /etc/debian_version ]]; then
  install_debian_build_deps
fi

echo "[install_deps] pip install -r requirements.txt"
pip3 install -r "${ROOT}/requirements.txt"
