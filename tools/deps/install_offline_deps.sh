#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEBS_DIR="${ROOT}/third_party/debs"
WHEELS_DIR="${ROOT}/third_party/wheels"
VENV_DIR="${ROOT}/third_party/.venv"
SKIP_SYSTEM=0
SKIP_PYTHON=0

usage() {
  cat <<'USAGE'
Usage: tools/deps/install_offline_deps.sh [--system-only] [--python-only] [--venv PATH]

Installs frozen dependencies from third_party/debs and third_party/wheels.
No network access is used for Python installation. System package installation
uses local .deb files and apt/dpkg only.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --system-only) SKIP_PYTHON=1 ;;
    --python-only) SKIP_SYSTEM=1 ;;
    --venv)
      shift
      VENV_DIR="${1:?--venv requires a path}"
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

run_sudo() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

verify_checksums() {
  local dir="$1"
  if [ -f "${dir}/SHA256SUMS" ]; then
    (cd "${dir}" && sha256sum -c SHA256SUMS)
  fi
}

if [ "${SKIP_SYSTEM}" -eq 0 ]; then
  if ! compgen -G "${DEBS_DIR}/*.deb" >/dev/null; then
    echo "No .deb files found in ${DEBS_DIR}. Run fetch_offline_deps.sh first." >&2
    exit 1
  fi
  verify_checksums "${DEBS_DIR}"
  echo "Installing local apt/ROS packages from ${DEBS_DIR}..."
  # dpkg may need a second pass after dependencies are unpacked, so follow with
  # apt-get --no-download fixup. It will fail if any required .deb is missing.
  run_sudo dpkg -i "${DEBS_DIR}"/*.deb || true
  run_sudo apt-get install -f -y --no-download
fi

if [ "${SKIP_PYTHON}" -eq 0 ]; then
  if ! compgen -G "${WHEELS_DIR}/*" >/dev/null; then
    echo "No wheels found in ${WHEELS_DIR}. Run fetch_offline_deps.sh first." >&2
    exit 1
  fi
  verify_checksums "${WHEELS_DIR}"
  echo "Creating Python environment at ${VENV_DIR}..."
  if [ -d "${VENV_DIR}" ] && [ ! -f "${VENV_DIR}/bin/activate" ]; then
    rm -rf "${VENV_DIR}"
  fi
  if ! python3 -m venv "${VENV_DIR}"; then
    echo "python3 venv could not run ensurepip; retrying with offline pip bootstrap..."
    rm -rf "${VENV_DIR}"
    python3 -m venv --without-pip "${VENV_DIR}"
    python3 -m pip \
      --python "${VENV_DIR}/bin/python" \
      install \
      --no-index \
      --find-links "${WHEELS_DIR}" \
      pip
  fi
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
  python -m pip install --upgrade --no-index --find-links "${WHEELS_DIR}" setuptools wheel
  python -m pip install --no-index --find-links "${WHEELS_DIR}" --requirement "${ROOT}/requirements.lock.txt"
  if [ -d "${ROOT}/third_party/sam2" ]; then
    python -m pip install \
      --no-index \
      --find-links "${WHEELS_DIR}" \
      --no-build-isolation \
      -e "${ROOT}/third_party/sam2"
  fi
  if [ -d "${ROOT}/cell_external_bridge" ]; then
    python -m pip install \
      --no-index \
      --find-links "${WHEELS_DIR}" \
      --no-build-isolation \
      -e "${ROOT}/cell_external_bridge"
  fi
fi

echo "Offline dependency install complete."
