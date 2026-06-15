#!/usr/bin/env bash
# Disable broken CentOS 7 SCL mirrorlist repos (EOL). Safe to run multiple times.
set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  exit 0
fi
if [[ ! -f /etc/redhat-release ]]; then
  exit 0
fi

_fixed=0
for f in /etc/yum.repos.d/*.repo; do
  [[ -f "${f}" ]] || continue
  if grep -qiE 'sclo-rh|sclo/sclo|centos-sclo' "${f}" 2>/dev/null; then
    if grep -qE '^enabled\s*=\s*1' "${f}" 2>/dev/null || grep -qE '^mirrorlist=' "${f}" 2>/dev/null; then
      sed -i \
        -e 's/^enabled\s*=\s*1/enabled=0/' \
        -e 's/^mirrorlist=/#mirrorlist=/' \
        "${f}"
      _fixed=1
      echo "[fix_yum] disabled broken repo: $(basename "${f}")"
    fi
  fi
done

if command -v yum-config-manager >/dev/null 2>&1; then
  yum-config-manager --disable centos-sclo-rh centos-sclo-sclo 2>/dev/null || true
fi

if [[ "${_fixed}" -eq 1 ]]; then
  yum clean all >/dev/null 2>&1 || true
fi

# Export for other scripts (bash only).
YUM_OPTS="${YUM_OPTS:-} --disablerepo=centos-sclo-rh --disablerepo=centos-sclo-sclo"
