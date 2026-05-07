#!/usr/bin/env bash
set -e
unset vblank_mode 2>/dev/null || true
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"

# ── Avahi / mDNS: required for LAN-connected Geomagic Touch (.local hostnames) ─
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
  CARD=$(grep -i '^CardType=' "${GTDD_HOME}/config/Default Device.config" 2>/dev/null | cut -d= -f2)
  DEVHOST=$(grep -i '^HostName=' "${GTDD_HOME}/config/Default Device.config" 2>/dev/null | cut -d= -f2)

  if [ "$CARD" = "LAN" ] && [ -n "$DEVHOST" ]; then
    echo "[entrypoint] Geomagic Touch LAN device: ${DEVHOST}" >&2

    # mDNS resolution
    RESOLVED_IP=$(getent hosts "$DEVHOST" 2>/dev/null | awk '{print $1}')
    if [ -n "$RESOLVED_IP" ]; then
      echo "[entrypoint] mDNS OK: ${RESOLVED_IP}  ${DEVHOST}" >&2
    else
      echo "============================================================" >&2
      echo "[entrypoint] FAIL: Cannot resolve ${DEVHOST}" >&2
      echo "[entrypoint] mDNS (.local) resolution failed → HD_COMM_ERROR." >&2
      echo "[entrypoint] Ensure avahi-daemon is running and device is powered on." >&2
      echo "============================================================" >&2
      RESOLVED_IP=""
    fi

    # Ping test
    if [ -n "$RESOLVED_IP" ]; then
      if ping -c 1 -W 1 "$RESOLVED_IP" >/dev/null 2>&1; then
        echo "[entrypoint] Ping OK: ${RESOLVED_IP} (latency < 1ms required for servo)" >&2
      else
        echo "[entrypoint] WARNING: Ping FAILED to ${RESOLVED_IP}" >&2
        echo "[entrypoint]   → Check cable, interface must be Link-Local Only (169.254.x.x)" >&2
      fi
    fi

    # Network interface and route
    if [ -n "$RESOLVED_IP" ] && command -v ip >/dev/null 2>&1; then
      ROUTE_INFO=$(ip route get "$RESOLVED_IP" 2>/dev/null || true)
      IFACE=$(echo "$ROUTE_INFO" | grep -oP 'dev \K\S+' || true)
      LOCAL_IP=$(echo "$ROUTE_INFO" | grep -oP 'src \K\S+' || true)
      if [ -n "$IFACE" ]; then
        echo "[entrypoint] Route: ${LOCAL_IP} → ${RESOLVED_IP} via ${IFACE}" >&2
        case "$LOCAL_IP" in
          169.254.*)
            echo "[entrypoint] Interface ${IFACE} Link-Local OK (${LOCAL_IP})" >&2
            ;;
          *)
            echo "============================================================" >&2
            echo "[entrypoint] WARNING: ${IFACE} local IP is ${LOCAL_IP} (NOT link-local!)" >&2
            echo "[entrypoint] The Geomagic Touch Ethernet requires the host interface" >&2
            echo "[entrypoint] to be configured as 'Link-Local Only' (169.254.x.x)." >&2
            echo "[entrypoint] On the HOST run:" >&2
            echo "[entrypoint]   sudo nmcli connection modify \"Wired connection 1\" ipv4.method link-local" >&2
            echo "[entrypoint]   sudo nmcli connection down \"Wired connection 1\" && sudo nmcli connection up \"Wired connection 1\"" >&2
            echo "============================================================" >&2
            ;;
        esac
      fi
    fi

    # Real-time scheduling check
    RT_MAX=$(ulimit -r 2>/dev/null || echo "0")
    if [ "$RT_MAX" -ge 99 ] 2>/dev/null; then
      echo "[entrypoint] RT scheduling OK (rtprio=${RT_MAX})" >&2
    else
      echo "[entrypoint] WARNING: rtprio=${RT_MAX} (need 99 for 1kHz servo)" >&2
      echo "[entrypoint]   → Add ulimits.rtprio: 99 in docker-compose.yml" >&2
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
