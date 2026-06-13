#!/usr/bin/env bash
set -Eeuo pipefail

# Install and start NoMachine on this station PC so another PC on the LAN can
# control the full desktop, terminals included. This script is intended to run
# on the remote robot/cell node, not on the viewing laptop.

X86_DOWNLOAD_PAGE="https://download.nomachine.com/download/?id=1&platform=linux"
ARM_DOWNLOAD_PAGE="https://download.nomachine.com/download/?distro=arm&id=30&platform=linux"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DEB_DIR="${SCRIPT_DIR}/debs"

INSTALL_XFCE=0
OPEN_UFW=0
FORCE_VIRTUAL_DISPLAY=0
AUTO_HEADLESS=0
OFFLINE_ONLY=0
NO_PROMPT=0
NOMACHINE_DEB_URL="${NOMACHINE_DEB_URL:-}"
TMP_DIR=""

usage() {
  cat <<'EOF'
Usage:
  ./NoMachine/install_nomachine_remote_access.sh [options]

Options:
  --install-xfce           Install a lightweight XFCE desktop if none is present.
  --open-ufw               If UFW is active, allow NoMachine TCP port 4000.
  --auto-headless          If no connected monitor is detected, stop the display
                           manager before restarting NoMachine so it can create
                           a virtual display.
  --force-virtual-display  Stop the Linux display manager and restart NoMachine
                           so NoMachine can create its own headless display.
                           This logs out any local graphical session.
  --deb-url URL            Install a specific NoMachine .deb instead of
                           auto-detecting the current official package.
  --offline-only           Use only bundled repo files. Do not download.
  -y, --yes                Do not prompt before installing packages.
  -h, --help               Show this help.

Environment:
  NOMACHINE_DEB_URL        Same as --deb-url.

Examples:
  ./NoMachine/install_nomachine_remote_access.sh
  ./NoMachine/install_nomachine_remote_access.sh --install-xfce --open-ufw
  ./NoMachine/install_nomachine_remote_access.sh --install-xfce --open-ufw --auto-headless --yes
EOF
}

log() {
  printf '[NoMachine install] %s\n' "$*" >&2
}

die() {
  printf '[NoMachine install] ERROR: %s\n' "$*" >&2
  exit 1
}

sudo_run() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

apt_update() {
  sudo_run env DEBIAN_FRONTEND=noninteractive apt-get update
}

apt_install() {
  sudo_run env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
}

confirm() {
  if [ "$NO_PROMPT" -eq 1 ]; then
    return 0
  fi
  printf '%s [y/N] ' "$1"
  read -r answer
  case "$answer" in
    y|Y|yes|YES) return 0 ;;
    *) return 1 ;;
  esac
}

cleanup() {
  if [ -n "$TMP_DIR" ] && [ -d "$TMP_DIR" ]; then
    rm -rf "$TMP_DIR"
  fi
}
trap cleanup EXIT

while [ "$#" -gt 0 ]; do
  case "$1" in
    --install-xfce)
      INSTALL_XFCE=1
      ;;
    --open-ufw)
      OPEN_UFW=1
      ;;
    --auto-headless)
      AUTO_HEADLESS=1
      ;;
    --force-virtual-display)
      FORCE_VIRTUAL_DISPLAY=1
      ;;
    --deb-url)
      shift
      [ "$#" -gt 0 ] || die "--deb-url needs a URL"
      NOMACHINE_DEB_URL="$1"
      ;;
    --offline-only)
      OFFLINE_ONLY=1
      ;;
    -y|--yes)
      NO_PROMPT=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
  shift
done

if ! command -v sudo >/dev/null 2>&1 && [ "$(id -u)" -ne 0 ]; then
  die "sudo is required unless this script is run as root"
fi

ensure_fetch_tool() {
  if [ "$OFFLINE_ONLY" -eq 1 ]; then
    printf 'offline'
    return
  fi
  if command -v curl >/dev/null 2>&1; then
    printf 'curl'
    return
  fi
  if command -v wget >/dev/null 2>&1; then
    printf 'wget'
    return
  fi

  log "Installing curl and CA certificates for HTTPS downloads."
  apt_update
  apt_install curl ca-certificates
  printf 'curl'
}

fetch_stdout() {
  local url="$1"
  if [ "$FETCH_TOOL" = "offline" ]; then
    die "Network fetch requested in --offline-only mode: $url"
  fi
  if [ "$FETCH_TOOL" = "curl" ]; then
    curl -fsSL "$url"
  else
    wget -qO- "$url"
  fi
}

fetch_file() {
  local url="$1"
  local output="$2"
  if [ "$FETCH_TOOL" = "offline" ]; then
    die "Network download requested in --offline-only mode: $url"
  fi
  if [ "$FETCH_TOOL" = "curl" ]; then
    curl -fL "$url" -o "$output"
  else
    wget -O "$output" "$url"
  fi
}

detect_deb_arch() {
  if command -v dpkg >/dev/null 2>&1; then
    dpkg --print-architecture
    return
  fi

  case "$(uname -m)" in
    x86_64) printf 'amd64\n' ;;
    i386|i686) printf 'i386\n' ;;
    aarch64|arm64) printf 'arm64\n' ;;
    armv7l|armv7*) printf 'armhf\n' ;;
    *) die "Unsupported architecture: $(uname -m)" ;;
  esac
}

download_page_for_arch() {
  case "$1" in
    amd64|i386) printf '%s\n' "$X86_DOWNLOAD_PAGE" ;;
    arm64|armhf) printf '%s\n' "$ARM_DOWNLOAD_PAGE" ;;
    *) die "Unsupported Debian architecture: $1" ;;
  esac
}

resolve_nomachine_deb_url() {
  local arch="$1"
  local page="$2"
  local html
  local url

  if [ -n "$NOMACHINE_DEB_URL" ]; then
    printf '%s\n' "$NOMACHINE_DEB_URL"
    return
  fi

  log "Resolving latest NoMachine DEB for architecture '$arch'."
  html="$(fetch_stdout "$page")"
  url="$(
    printf '%s\n' "$html" \
      | grep -Eo 'https?://[^"]+/nomachine_[0-9][^"]+_'"$arch"'\.deb' \
      | head -n 1
      || true
  )"

  if [ -z "$url" ]; then
    die "Could not find a NoMachine DEB URL for '$arch' on $page. Re-run with --deb-url."
  fi
  printf '%s\n' "$url"
}

find_local_nomachine_deb() {
  local arch="$1"
  local found=""
  if [ -d "$LOCAL_DEB_DIR" ]; then
    found="$(
      find "$LOCAL_DEB_DIR" -maxdepth 1 -type f -name "nomachine_*_${arch}.deb" \
        | sort -V \
        | tail -n 1 \
        || true
    )"
  fi
  if [ -n "$found" ]; then
    printf '%s\n' "$found"
  fi
}

verify_local_deb_checksum() {
  local deb_path="$1"
  local checksum_file="${LOCAL_DEB_DIR}/SHA256SUMS"
  if [ ! -f "$checksum_file" ]; then
    return 0
  fi
  (
    cd "$LOCAL_DEB_DIR"
    sha256sum -c SHA256SUMS --ignore-missing
  )
  local filename
  filename="$(basename "$deb_path")"
  if ! grep -Fq "  ${filename}" "$checksum_file"; then
    log "No checksum entry for ${filename}; continuing without checksum verification for this file."
  fi
}

download_nomachine_deb_to_cache() {
  local arch="$1"
  local page="$2"
  local url
  local output_dir
  local output_path

  url="$(resolve_nomachine_deb_url "$arch" "$page")"
  output_dir="$LOCAL_DEB_DIR"
  if ! mkdir -p "$output_dir" 2>/dev/null; then
    TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/nomachine-install.XXXXXX")"
    output_dir="$TMP_DIR"
  fi
  output_path="${output_dir}/$(basename "$url")"
  log "Downloading: $url"
  fetch_file "$url" "$output_path"
  printf '%s\n' "$output_path"
}

monitor_connected() {
  local status_file
  for status_file in /sys/class/drm/*/status; do
    [ -e "$status_file" ] || continue
    if grep -qx 'connected' "$status_file" 2>/dev/null; then
      return 0
    fi
  done
  return 1
}

desktop_detected() {
  command -v gnome-session >/dev/null 2>&1 && return 0
  command -v startxfce4 >/dev/null 2>&1 && return 0
  command -v startplasma-x11 >/dev/null 2>&1 && return 0
  command -v mate-session >/dev/null 2>&1 && return 0
  [ -d /usr/share/xsessions ] && find /usr/share/xsessions -name '*.desktop' -print -quit | grep -q .
}

print_lan_addresses() {
  local ips
  ips="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' || true)"
  if [ -z "$ips" ]; then
    log "Could not detect an IPv4 LAN address. Check with: hostname -I"
    return
  fi

  log "Connect from the other PC with one of:"
  while IFS= read -r ip; do
    [ -n "$ip" ] && printf '  nx://%s:4000\n' "$ip"
  done <<< "$ips"
}

FETCH_TOOL="$(ensure_fetch_tool)"

if ! command -v apt-get >/dev/null 2>&1; then
  die "This installer currently supports Debian/Ubuntu systems with apt-get"
fi

if [ "$INSTALL_XFCE" -eq 1 ]; then
  if desktop_detected; then
    log "Desktop environment already detected; skipping XFCE install."
  else
    confirm "Install XFCE desktop packages now?" || die "Cancelled before desktop install"
    apt_update
    apt_install xfce4 xfce4-terminal dbus-x11
  fi
elif ! desktop_detected; then
  log "No desktop environment was detected."
  log "NoMachine needs a desktop environment; re-run with --install-xfce if this PC is headless/minimal."
fi

if [ -x /etc/NX/nxserver ]; then
  log "NoMachine is already installed; skipping package download/install."
else
  ARCH="$(detect_deb_arch)"
  DEB_PATH="$(find_local_nomachine_deb "$ARCH")"
  if [ -n "$DEB_PATH" ]; then
    log "Using bundled NoMachine package: $DEB_PATH"
    verify_local_deb_checksum "$DEB_PATH"
  else
    if [ "$OFFLINE_ONLY" -eq 1 ]; then
      die "No bundled NoMachine package found for '$ARCH' in $LOCAL_DEB_DIR"
    fi
    DOWNLOAD_PAGE="$(download_page_for_arch "$ARCH")"
    DEB_PATH="$(download_nomachine_deb_to_cache "$ARCH" "$DOWNLOAD_PAGE")"
  fi

  confirm "Install NoMachine package with apt?" || die "Cancelled before NoMachine install"
  apt_install "$DEB_PATH"
fi

if [ "$OPEN_UFW" -eq 1 ]; then
  if command -v ufw >/dev/null 2>&1 && sudo_run ufw status | grep -qi '^Status: active'; then
    log "Opening TCP/4000 in UFW for NoMachine."
    sudo_run ufw allow 4000/tcp comment 'NoMachine NX remote desktop'
  else
    log "UFW is not active or not installed; no firewall rule changed."
  fi
fi

if [ "$AUTO_HEADLESS" -eq 1 ] && ! monitor_connected; then
  FORCE_VIRTUAL_DISPLAY=1
  log "No connected monitor detected; enabling NoMachine virtual-display mode."
fi

if [ "$FORCE_VIRTUAL_DISPLAY" -eq 1 ]; then
  confirm "Stop the display manager to force NoMachine virtual-display mode?" \
    || die "Cancelled before changing display mode"
  sudo_run systemctl stop display-manager || true
fi

if [ -x /etc/NX/nxserver ]; then
  sudo_run /etc/NX/nxserver --startup --start-mode manual
  sudo_run /etc/NX/nxserver --status || true
else
  die "NoMachine installed, but /etc/NX/nxserver was not found"
fi

print_lan_addresses
log "Use the remote PC's normal Linux username/password when NoMachine asks for login."
log "For no-monitor machines, an HDMI dummy plug is still the simplest way to avoid GPU/display-manager edge cases."
