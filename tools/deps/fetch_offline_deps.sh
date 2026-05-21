#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
APT_LIST="${ROOT}/tools/deps/apt-packages.freeze.txt"
DEBS_DIR="${ROOT}/third_party/debs"
WHEELS_DIR="${ROOT}/third_party/wheels"
APT_STATE="${ROOT}/third_party/.apt-state"
APT_CACHE="${ROOT}/third_party/.apt-cache"
PYTORCH_CPU_INDEX_URL="${PYTORCH_CPU_INDEX_URL:-https://download.pytorch.org/whl/cpu}"
SKIP_APT=0
SKIP_PIP=0
HOST_STATUS=0

usage() {
  cat <<'USAGE'
Usage: tools/deps/fetch_offline_deps.sh [--skip-apt] [--skip-pip] [--host-status]

Downloads the frozen offline dependency bundle into:
  third_party/debs
  third_party/wheels

Default apt mode simulates an empty target machine so the dependency closure is
downloaded. Use --host-status to download only what apt thinks this machine
needs or reinstalls.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --skip-apt) SKIP_APT=1 ;;
    --skip-pip) SKIP_PIP=1 ;;
    --host-status) HOST_STATUS=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

mkdir -p "${DEBS_DIR}" "${WHEELS_DIR}"

read_apt_specs() {
  grep -Ev '^\s*(#|$)' "${APT_LIST}"
}

if [ "${SKIP_APT}" -eq 0 ]; then
  mapfile -t APT_SPECS < <(read_apt_specs)
  if [ "${#APT_SPECS[@]}" -eq 0 ]; then
    echo "No apt specs found in ${APT_LIST}" >&2
    exit 1
  fi

  mkdir -p \
    "${APT_STATE}/lists/partial" \
    "${APT_CACHE}/archives/partial" \
    "${DEBS_DIR}/partial"
  APT_BASE=(
    -o "Dir::State=${APT_STATE}"
    -o "Dir::State::lists=${APT_STATE}/lists"
    -o "Dir::Cache=${APT_CACHE}"
    -o "Dir::Cache::archives=${DEBS_DIR}"
    -o "Debug::NoLocking=true"
  )

  echo "Updating apt metadata in workspace-local cache..."
  apt-get "${APT_BASE[@]}" update

  echo "Downloading apt/ROS package closure to ${DEBS_DIR}..."
  APT_COMMON=(
    -y
    --download-only
    --reinstall
  )
  if [ "${HOST_STATUS}" -eq 1 ]; then
    if ! apt-get "${APT_BASE[@]}" install "${APT_COMMON[@]}" "${APT_SPECS[@]}"; then
      echo "Exact apt versions were not all available; retrying with current repository candidates." >&2
      APT_NAMES=("${APT_SPECS[@]%%=*}")
      apt-get "${APT_BASE[@]}" install "${APT_COMMON[@]}" "${APT_NAMES[@]}"
    fi
  else
    EMPTY_STATUS="${APT_STATE}/empty-status"
    : > "${EMPTY_STATUS}"
    if ! apt-get \
      "${APT_BASE[@]}" \
      -o "Dir::State::status=${EMPTY_STATUS}" \
      install \
      "${APT_COMMON[@]}" \
      "${APT_SPECS[@]}"; then
      echo "Exact apt versions were not all available; retrying with current repository candidates." >&2
      APT_NAMES=("${APT_SPECS[@]%%=*}")
      apt-get \
        "${APT_BASE[@]}" \
        -o "Dir::State::status=${EMPTY_STATUS}" \
        install \
        "${APT_COMMON[@]}" \
        "${APT_NAMES[@]}"
    fi
  fi

  (
    cd "${DEBS_DIR}"
    find . -maxdepth 1 -type f -name '*.deb' -print0 \
      | sort -z \
      | xargs -0r sha256sum > SHA256SUMS
    for deb in ./*.deb; do
      [ -e "${deb}" ] || continue
      printf '%s=%s\n' "$(dpkg-deb -f "${deb}" Package)" "$(dpkg-deb -f "${deb}" Version)"
    done | sort > apt-downloaded.lock
  )
fi

if [ "${SKIP_PIP}" -eq 0 ]; then
  echo "Downloading Python wheels to ${WHEELS_DIR}..."
  python3 -m pip download \
    --dest "${WHEELS_DIR}" \
    --extra-index-url "${PYTORCH_CPU_INDEX_URL}" \
    --requirement "${ROOT}/requirements.lock.txt"
  mapfile -d '' SOURCE_DISTS < <(
    find "${WHEELS_DIR}" -maxdepth 1 -type f \
      \( -name '*.tar.gz' -o -name '*.zip' \) \
      -print0
  )
  if [ "${#SOURCE_DISTS[@]}" -gt 0 ]; then
    python3 -m pip wheel \
      --wheel-dir "${WHEELS_DIR}" \
      --no-deps \
      --no-build-isolation \
      "${SOURCE_DISTS[@]}"
  fi
  if [ -d "${ROOT}/third_party/sam2" ]; then
    python3 -m pip wheel \
      --wheel-dir "${WHEELS_DIR}" \
      --no-deps \
      --no-build-isolation \
      "${ROOT}/third_party/sam2" || true
  fi
  (
    cd "${WHEELS_DIR}"
    find . -maxdepth 1 -type f ! -name 'SHA256SUMS' ! -name '.gitkeep' -print0 \
      | sort -z \
      | xargs -0r sha256sum > SHA256SUMS
  )
fi

python3 "${ROOT}/tools/deps/audit_dependencies.py"
echo "Offline dependency fetch complete."
