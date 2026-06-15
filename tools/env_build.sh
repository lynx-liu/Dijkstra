#!/usr/bin/env bash
# Source before cmake / running mmlp binaries on RHEL7 (GCC 4.8 needs devtoolset for C++17).
# Usage: source tools/env_build.sh

_need_gcc_major=7

_gcc_ver() {
  if command -v g++ >/dev/null 2>&1; then
    g++ -dumpversion 2>/dev/null | cut -d. -f1
  else
    echo 0
  fi
}

_enable_devtoolset() {
  local v
  for v in 11 10 9 8 7; do
    if [[ -f "/opt/rh/devtoolset-${v}/enable" ]]; then
      # shellcheck disable=SC1090
      source "/opt/rh/devtoolset-${v}/enable"
      return 0
    fi
  done
  for v in 13 12 11 10 9; do
    if [[ -f "/opt/rh/gcc-toolset-${v}/enable" ]]; then
      # shellcheck disable=SC1090
      source "/opt/rh/gcc-toolset-${v}/enable"
      return 0
    fi
  done
  return 1
}

_enable_devtoolset || true

if [[ "$(_gcc_ver)" -lt "${_need_gcc_major}" ]]; then
  echo "ERROR: g++ $(_gcc_ver) is too old; C++17 needs g++ 7+." >&2
  echo "Run as root: bash tools/install_build_deps.sh" >&2
  return 1 2>/dev/null || exit 1
fi

if command -v cmake3 >/dev/null 2>&1; then
  export CMAKE=cmake3
elif command -v cmake >/dev/null 2>&1; then
  export CMAKE=cmake
else
  echo "ERROR: cmake not found. Run: bash tools/install_build_deps.sh" >&2
  return 1 2>/dev/null || exit 1
fi

export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"
