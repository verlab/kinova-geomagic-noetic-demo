#!/usr/bin/env bash
set -e
unset vblank_mode 2>/dev/null || true
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"

# ── Avahi / mDNS: required for LAN-connected Geomagic Touch (.local hostnames) ─
# libHD → libPhantomIOLib42 resolves the Touch via its mDNS name (e.g. ep16029002631.local).
# Without avahi-daemon + libnss-mdns, .local resolution fails → HD_COMM_ERROR.
if command -v avahi-daemon >/dev/null 2>&1; then
  mkdir -p /run/dbus
  dbus-daemon --system --nofork --nopidfile 2>/dev/null &
  sleep 0.3
  avahi-daemon --daemonize --no-chroot 2>/dev/null || true
fi

# ── Geomagic Touch: ensure config directory exists ────────────────────────────
mkdir -p "${GTDD_HOME}/config"

# Pre-flight: warn if no config file (Setup not yet run in this volume).
if [ ! -f "${GTDD_HOME}/config/Default Device.config" ]; then
  echo "============================================================" >&2
  echo "[entrypoint] WARNING: ${GTDD_HOME}/config/Default Device.config not found." >&2
  echo "[entrypoint] OpenHaptics HD API will fail (HD_COMM_ERROR)." >&2
  echo "[entrypoint] Run Setup first:" >&2
  echo "[entrypoint]   docker compose --profile setup run --rm geomagic-setup" >&2
  echo "============================================================" >&2
else
  # Show device connection type for quick sanity check.
  CARD=$(grep -i '^CardType=' "${GTDD_HOME}/config/Default Device.config" 2>/dev/null | cut -d= -f2)
  HOST=$(grep -i '^HostName=' "${GTDD_HOME}/config/Default Device.config" 2>/dev/null | cut -d= -f2)
  if [ "$CARD" = "LAN" ] && [ -n "$HOST" ]; then
    echo "[entrypoint] Geomagic Touch LAN device: ${HOST}" >&2
    # Quick mDNS resolution test
    if getent hosts "$HOST" >/dev/null 2>&1; then
      echo "[entrypoint] mDNS OK: $(getent hosts "$HOST")" >&2
    else
      echo "============================================================" >&2
      echo "[entrypoint] WARNING: Cannot resolve ${HOST}" >&2
      echo "[entrypoint] mDNS (.local) resolution failed → HD_COMM_ERROR." >&2
      echo "[entrypoint] Ensure avahi-daemon is running and device is powered on." >&2
      echo "============================================================" >&2
    fi
  fi
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
