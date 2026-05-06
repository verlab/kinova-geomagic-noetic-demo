#!/usr/bin/env bash
# Install host-side prerequisites for Geomagic Touch (USB) + Docker GUI workflow.
# Run from the repository root, e.g.:  ./docker/host-setup.sh
#
# Does: copy udev rules, reload udev, add your user to group plugdev (for MODE=0660, GROUP=plugdev).
# Does not: install Docker or vendor SDKs (those belong in the image / docs).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULES_SRC="${REPO_ROOT}/docker/udev/70-geomagic-touch.rules"
RULES_NAME="70-geomagic-touch.rules"
RULES_DST="/etc/udev/rules.d/${RULES_NAME}"
DRY_RUN=false

usage() {
  sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
  echo "Options:  --dry-run   print actions only"
  echo "          -h, --help  this text"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

if [[ ! -f "${RULES_SRC}" ]]; then
  echo "Cannot find ${RULES_SRC} (run this script from a clone of kinova-geomagic-noetic-demo)." >&2
  exit 1
fi

# User to put in plugdev — prefer the account that invoked sudo
TARGET_USER="${SUDO_USER:-}"
if [[ -z "${TARGET_USER}" && ${EUID:-1} -ne 0 ]]; then
  TARGET_USER="$(id -un)"
fi

have_sudo() {
  if ${DRY_RUN}; then return 0; fi
  if [[ ${EUID:-1} -eq 0 ]]; then return 0; fi
  command -v sudo >/dev/null 2>&1
}

if ! have_sudo; then
  echo "This script needs sudo to install rules under ${RULES_DST}." >&2
  exit 1
fi

sudo_wrap() {
  if ${DRY_RUN}; then
    printf '[dry-run] sudo'
    printf ' %q' "$@"
    echo
    return 0
  fi
  if [[ ${EUID:-1} -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

echo "Repository: ${REPO_ROOT}"
echo "Installing Geomagic Touch udev rules for USB (vendor 256f, products c652 / c631)."

sudo_wrap cp "${RULES_SRC}" "${RULES_DST}"
sudo_wrap chmod 644 "${RULES_DST}"

if ! getent group plugdev >/dev/null 2>&1; then
  echo "Creating group plugdev..."
  sudo_wrap groupadd plugdev
fi

if [[ -n "${TARGET_USER}" && "${TARGET_USER}" != "root" ]]; then
  if id -nG "${TARGET_USER}" 2>/dev/null | tr ' ' '\n' | grep -qx plugdev; then
    echo "User ${TARGET_USER} is already in group plugdev."
  else
    echo "Adding ${TARGET_USER} to group plugdev (needed for hidraw/usb access with these rules)."
    sudo_wrap usermod -aG plugdev "${TARGET_USER}"
    echo ""
    echo ">>> Log out and back in (or: newgrp plugdev) so the new group applies."
    echo ""
  fi
else
  echo "Could not detect a non-root user to add to plugdev; add your desktop user manually:"
  echo "  sudo usermod -aG plugdev YOUR_USERNAME"
fi

echo "Reloading udev..."
sudo_wrap udevadm control --reload-rules
sudo_wrap udevadm trigger --subsystem-match=usb || true
sudo_wrap udevadm trigger --subsystem-match=hidraw || true

echo ""
echo "Done. Next steps:"
echo "  • Replug the Geomagic Touch USB cable, or wait a second and try your container again."
echo "  • Verify the device:  lsusb  (look for 256f:c652 or 256f:c631; edit ${RULES_SRC} if yours differs)."
echo "  • Docker GUI (RViz / Geomagic setup): on X11 often  xhost +local:root  once per session."
echo "  • Kinova Gen3: connect the arm’s Ethernet to your LAN and use robot_ip in launch (default 192.168.1.10)."
echo "  • Build/run stack:  docker compose build   &&   docker compose up"
echo ""
