#!/usr/bin/env bash
set -Eeuo pipefail

# Download official NoMachine DEBs into NoMachine/debs and update SHA256SUMS.
# Use this on a development machine when refreshing the bundled package set.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEB_DIR="${SCRIPT_DIR}/debs"
X86_DOWNLOAD_PAGE="https://download.nomachine.com/download/?id=1&platform=linux"
ARM_DOWNLOAD_PAGE="https://download.nomachine.com/download/?distro=arm&id=30&platform=linux"
ARCHES=("$@")

if [ "${#ARCHES[@]}" -eq 0 ]; then
  ARCHES=(amd64)
fi

fetch_page() {
  curl -fsSL "$1"
}

download_page_for_arch() {
  case "$1" in
    amd64|i386) printf '%s\n' "$X86_DOWNLOAD_PAGE" ;;
    arm64|armhf) printf '%s\n' "$ARM_DOWNLOAD_PAGE" ;;
    *) echo "Unsupported architecture: $1" >&2; exit 2 ;;
  esac
}

resolve_url() {
  local arch="$1"
  local page="$2"
  local matches
  matches="$(
    fetch_page "$page" \
      | grep -Eo 'https?://[^"]+/nomachine_[0-9][^"]+_'"$arch"'\.deb' \
      || true
  )"
  printf '%s\n' "$matches" | head -n 1
}

mkdir -p "$DEB_DIR"
for arch in "${ARCHES[@]}"; do
  page="$(download_page_for_arch "$arch")"
  url="$(resolve_url "$arch" "$page")"
  if [ -z "$url" ]; then
    echo "Could not resolve NoMachine DEB URL for $arch" >&2
    exit 1
  fi
  echo "Downloading $url"
  curl -fL "$url" -o "${DEB_DIR}/$(basename "$url")"
done

(
  cd "$DEB_DIR"
  sha256sum nomachine_*.deb > SHA256SUMS
)

echo "Cached NoMachine packages in ${DEB_DIR}"
