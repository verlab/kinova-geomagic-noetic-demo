#!/usr/bin/env bash
# Install OpenHaptics 3.4 + Geomagic Touch driver from tarballs on THIS machine (host),
# same layout as inside the Docker image (uses docker/install_vendor_geomagic.sh).
#
# Requirements:
#   • Place in docker/vendor/ both tarballs (see docker/vendor/README.md).
#   • Ubuntu 20.04+ typical; requires sudo to write under /usr and /opt.
#
# Usage (from repo root):
#   ./docker/install-vendor-host.sh
#   ./docker/install-vendor-host.sh --dry-run
#
# After install: log out and back in (or: source /etc/profile.d/geomagic-touch-vendor.sh)
# so GTDD_HOME is set in new shells. Then run Geomagic_Touch_Setup or native ROS build.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENDOR_DIR="${REPO_ROOT}/docker/vendor"
INSTALL_INNER="${SCRIPT_DIR}/install_vendor_geomagic.sh"
PROFILE_D="/etc/profile.d/geomagic-touch-vendor.sh"
DRY_RUN=false

usage() {
  cat <<EOF
Install vendor OpenHaptics + Geomagic Touch from docker/vendor/*.tar.gz onto the host.

Options:
  --dry-run    show what would run
  -h, --help   this text
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

OH_TGZ="${VENDOR_DIR}/openhaptics_3.4-0-developer-edition-amd64.tar.gz"
GM_TGZ="${VENDOR_DIR}/geomagic_touch_device_driver_2016.1-1-amd64.tar.gz"

for f in "${OH_TGZ}" "${GM_TGZ}"; do
  if [[ ! -f "${f}" ]]; then
    echo "Missing vendor archive: ${f}" >&2
    echo "Copy both .tar.gz files into docker/vendor/ (see docker/vendor/README.md)." >&2
    exit 1
  fi
done

if [[ ! -f "${INSTALL_INNER}" ]]; then
  echo "Missing ${INSTALL_INNER}" >&2
  exit 1
fi

write_profile_d() {
  cat <<'PROFILE'
# geomagic-touch-vendor.sh — written by docker/install-vendor-host.sh
# Required by libPhantomIOLib / Geomagic Touch Setup (same as Docker ENV GTDD_HOME).
export GTDD_HOME=/opt/geomagic_touch_device_driver
export OPENHAPTICS_ROOT=/usr
PROFILE
}

install_profile_d() {
  if ${DRY_RUN}; then
    echo "[dry-run] would write ${PROFILE_D}"
    write_profile_d | sed 's/^/[dry-run profile] /'
    return 0
  fi
  if [[ ${EUID:-1} -eq 0 ]]; then
    write_profile_d > "${PROFILE_D}"
    chmod 644 "${PROFILE_D}"
  else
    write_profile_d | sudo tee "${PROFILE_D}" >/dev/null
    sudo chmod 644 "${PROFILE_D}"
  fi
  echo "Installed ${PROFILE_D} (GTDD_HOME + OPENHAPTICS_ROOT for login shells)."
}

run_vendor_install() {
  if [[ ${EUID:-1} -eq 0 ]]; then
    bash "${INSTALL_INNER}" "${VENDOR_DIR}"
  else
    sudo env DEBIAN_FRONTEND=noninteractive bash "${INSTALL_INNER}" "${VENDOR_DIR}"
  fi
}

if ${DRY_RUN}; then
  echo "[dry-run] sudo bash ${INSTALL_INNER} ${VENDOR_DIR}"
  install_profile_d
  echo "Done (dry-run)."
  exit 0
fi

if [[ ${EUID:-1} -ne 0 ]] && ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required when not root." >&2
  exit 1
fi

echo "Installing vendor payloads from ${VENDOR_DIR} into /usr and /opt ..."
echo "This overwrites OpenHaptics-related files under /usr (same as Docker build)."
run_vendor_install
install_profile_d

echo ""
echo "Vendor install finished."
echo "  • Re-open your session or run:  source ${PROFILE_D}"
echo "  • USB udev (if not yet):         ./docker/host-setup.sh"
echo "  • Native catkin build: install ROS Noetic deps, then catkin_make in catkin_ws"
echo ""
