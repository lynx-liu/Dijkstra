#!/usr/bin/env bash
# Runtime library path for mmlp binaries on CentOS 7 (devtoolset). No gcc/cmake check.

_load_devtoolset() {
  local v f
  for v in 11 10 9 8 7; do
    f="/opt/rh/devtoolset-${v}/enable"
    if [[ -f "${f}" ]]; then
      set +eu
      # shellcheck disable=SC1090
      source "${f}"
      set -eu
      return 0
    fi
  done
  for v in 13 12 11 10 9; do
    f="/opt/rh/gcc-toolset-${v}/enable"
    if [[ -f "${f}" ]]; then
      set +eu
      # shellcheck disable=SC1090
      source "${f}"
      set -eu
      return 0
    fi
  done
  return 0
}

_load_devtoolset
