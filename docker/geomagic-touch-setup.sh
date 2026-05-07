#!/usr/bin/env bash
# Launch Geomagic Touch configuration GUI (USB/LAN pairing). Requires X11 + --network host.
# GTDD_HOME = config root (vendor); binaries live under /opt/geomagic_touch_device_driver.
GEOM_DRIVER="/opt/geomagic_touch_device_driver"
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
mkdir -p "${GTDD_HOME}/config"
export LD_LIBRARY_PATH="${GEOM_DRIVER}/lib:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="${GEOM_DRIVER}/lib/plugins"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"
# In Docker, DRI/nouveau often fails — Mesa llvmpipe is enough for this small Qt UI.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
echo "[geomagic-touch-setup] After pairing, verify that the file"
echo "  ${GTDD_HOME}/config/Default Device.config"
echo "  exists. All OpenHaptics demos need it."
exec "${GEOM_DRIVER}/Geomagic_Touch_Setup" "$@"
