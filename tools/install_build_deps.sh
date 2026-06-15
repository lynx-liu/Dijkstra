#!/usr/bin/env bash
# C++ build toolchain for MMLP (C++17). On CentOS 7 installs devtoolset-8 from vault mirror.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"

need_gcc_major=7

gcc_major() {
  if command -v g++ >/dev/null 2>&1; then
    g++ -dumpversion 2>/dev/null | cut -d. -f1
  else
    echo 0
  fi
}

try_enable_devtoolset() {
  # shellcheck source=/dev/null
  source "${ROOT}/env_build.sh" 2>/dev/null || true
}

install_centos7_devtoolset() {
  local pkg=yum
  command -v dnf >/dev/null 2>&1 && pkg=dnf

  echo "[install_build_deps] CentOS 7 detected; installing devtoolset-8 (C++17)..."
  ${pkg} install -y epel-release make 2>/dev/null || ${pkg} install -y make

  if ! command -v cmake3 >/dev/null 2>&1 && ! command -v cmake >/dev/null 2>&1; then
    ${pkg} install -y cmake3 || ${pkg} install -y cmake
  fi
  if command -v cmake3 >/dev/null 2>&1 && ! command -v cmake >/dev/null 2>&1; then
    alternatives --set cmake /usr/bin/cmake3 2>/dev/null || ln -sf "$(command -v cmake3)" /usr/local/bin/cmake
  fi

  local arch
  arch=$(uname -m)
  local repo=/etc/yum.repos.d/mmlp-centos-sclo-vault.repo
  cat >"${repo}" <<EOF
[mmlp-sclo-rh]
name=CentOS-7 SCLo rh (vault)
baseurl=https://mirrors.aliyun.com/centos-vault/7.9.2009/sclo/${arch}/rh/
        https://vault.centos.org/7.9.2009/sclo/${arch}/rh/
enabled=1
gpgcheck=0

[mmlp-sclo-sclo]
name=CentOS-7 SCLo sclo (vault)
baseurl=https://mirrors.aliyun.com/centos-vault/7.9.2009/sclo/${arch}/sclo/
        https://vault.centos.org/7.9.2009/sclo/${arch}/sclo/
enabled=1
gpgcheck=0
EOF

  ${pkg} clean all || true
  ${pkg} install -y --disablerepo='*' --enablerepo='mmlp-sclo-rh,mmlp-sclo-sclo' \
    devtoolset-8-gcc devtoolset-8-gcc-c++ devtoolset-8-binutils devtoolset-8-runtime
}

echo "[install_build_deps] checking g++..."
try_enable_devtoolset || true
maj=$(gcc_major)
if [[ "${maj}" -ge "${need_gcc_major}" ]]; then
  echo "[install_build_deps] g++ OK: $(g++ --version | head -1)"
  exit 0
fi

if [[ ! -f /etc/redhat-release ]]; then
  echo "ERROR: g++ ${maj} too old; need ${need_gcc_major}+. Install a newer compiler for your distro." >&2
  exit 1
fi

if [[ $(id -u) -ne 0 ]]; then
  echo "ERROR: g++ ${maj} too old; need ${need_gcc_major}+ (C++17)." >&2
  echo "Run as root: sudo bash tools/install_build_deps.sh" >&2
  exit 1
fi

if grep -qiE 'centos.*7|aliyun|alinux' /etc/redhat-release 2>/dev/null; then
  install_centos7_devtoolset
else
  local_pkg=yum
  command -v dnf >/dev/null 2>&1 && local_pkg=dnf
  ${local_pkg} install -y gcc-c++ cmake make || true
fi

try_enable_devtoolset || true
maj=$(gcc_major)
if [[ "${maj}" -lt "${need_gcc_major}" ]]; then
  echo "ERROR: still on g++ ${maj} after install. Check yum output above." >&2
  exit 1
fi

echo "[install_build_deps] g++ ready: $(g++ --version | head -1)"
echo "[install_build_deps] cmake: $(cmake3 --version 2>/dev/null | head -1 || cmake --version | head -1)"
