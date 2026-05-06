#!/usr/bin/env bash
# Launch Geomagic Touch configuration GUI (USB/LAN pairing). Requires X11 + --network host.
export GTDD_HOME="${GTDD_HOME:-/opt/geomagic_touch_device_driver}"
export LD_LIBRARY_PATH="/opt/geomagic_touch_device_driver/lib:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="/opt/geomagic_touch_device_driver/lib/plugins"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"
# In Docker, DRI/nouveau often fails — Mesa llvmpipe is enough for this small Qt UI.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
exec /opt/geomagic_touch_device_driver/Geomagic_Touch_Setup "$@"
