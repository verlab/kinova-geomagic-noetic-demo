#!/usr/bin/env bash
# Launch Geomagic Touch configuration GUI (USB/LAN pairing). Requires X11 + --network host.
export LD_LIBRARY_PATH="/opt/geomagic_touch_device_driver/lib:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="/opt/geomagic_touch_device_driver/lib/plugins"
exec /opt/geomagic_touch_device_driver/Geomagic_Touch_Setup "$@"
