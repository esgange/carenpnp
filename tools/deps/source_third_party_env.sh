#!/usr/bin/env bash
# Source this file from the workspace root to use the frozen offline Python env.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source_setup() {
  local had_errexit=0
  local had_nounset=0
  case $- in
    *e*) had_errexit=1; set +e ;;
  esac
  case $- in
    *u*) had_nounset=1; set +u ;;
  esac
  # shellcheck disable=SC1090
  source "$1"
  local status=$?
  if [ "${had_nounset}" -eq 1 ]; then
    set -u
  fi
  if [ "${had_errexit}" -eq 1 ]; then
    set -e
  fi
  return "${status}"
}

if [ -f /opt/ros/humble/setup.bash ]; then
  source_setup /opt/ros/humble/setup.bash
fi
if [ -f "${ROOT}/install/setup.bash" ]; then
  source_setup "${ROOT}/install/setup.bash"
fi
if [ -f "${ROOT}/third_party/.venv/bin/activate" ]; then
  source_setup "${ROOT}/third_party/.venv/bin/activate"
fi
