#!/usr/bin/env bash
set -e
unset vblank_mode 2>/dev/null || true
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"

# ── Geomagic Touch: ensure config directory exists ────────────────────────────
# The named volume 'geomagic-touch-config' is mounted at $GTDD_HOME.
# libHD (OpenHaptics) requires $GTDD_HOME/config/ to exist AND contain
# "Default Device.config" (created by Geomagic_Touch_Setup).
# Without it: HD_COMM_ERROR / HD_COMM_CONFIG_ERROR on all demos.
mkdir -p "${GTDD_HOME}/config"

# Pre-flight: warn if no config file (Setup not yet run in this volume).
if [ ! -f "${GTDD_HOME}/config/Default Device.config" ]; then
  echo "============================================================" >&2
  echo "[entrypoint] WARNING: ${GTDD_HOME}/config/Default Device.config not found." >&2
  echo "[entrypoint] OpenHaptics HD API will fail (HD_COMM_ERROR)." >&2
  echo "[entrypoint] Run Setup first:" >&2
  echo "[entrypoint]   docker compose --profile setup run --rm geomagic-setup" >&2
  echo "============================================================" >&2
fi

# ── USB: disable autosuspend for Geomagic Touch (vendor 256f) if present ──────
for vid in /sys/bus/usb/devices/*/idVendor; do
  [ "$(cat "$vid" 2>/dev/null)" = "256f" ] && {
    devdir="$(dirname "$vid")"
    echo on > "${devdir}/power/control" 2>/dev/null || true
  }
done

# ── ROS setup ─────────────────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash 2>/dev/null || true
exec "$@"
