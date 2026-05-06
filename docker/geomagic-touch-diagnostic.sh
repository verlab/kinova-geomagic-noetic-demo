#!/usr/bin/env bash
# Geomagic Touch Diagnostic / calibration helper (vendor binary under /opt/geomagic_touch_device_driver).
# Same environment as geomagic-touch-setup. Requires X11 + DISPLAY (see docker-compose).
set -euo pipefail

GEOM_DRIVER="/opt/geomagic_touch_device_driver"

export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export LD_LIBRARY_PATH="${GEOM_DRIVER}/lib:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="${GEOM_DRIVER}/lib/plugins"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

try_exec() {
  local p="$1"
  shift
  if [[ -x "$p" ]]; then
    exec "$p" "$@"
  fi
}

try_exec "${GEOM_DRIVER}/Geomagic_Touch_Diagnostic" "$@"
try_exec "${GEOM_DRIVER}/geomagic_touch_diagnostic" "$@"
try_exec "${GEOM_DRIVER}/Geomagic_Touch_Calibration" "$@"
try_exec "${GEOM_DRIVER}/geomagic_touch_calibration" "$@"

# Some vendor trees ship odd names — pick the first *Diagnostic* executable in the install root.
shopt -s nullglob
for p in "${GEOM_DRIVER}"/*[Dd]iagnostic* "${GEOM_DRIVER}"/bin/*[Dd]iagnostic*; do
  if [[ -f "$p" && -x "$p" ]]; then
    exec "$p" "$@"
  fi
done
shopt -u nullglob

echo "No diagnostic/calibration binary found under ${GEOM_DRIVER}." >&2
echo "Rebuild the image with geomagic_touch_device_driver in docker/vendor/." >&2
exit 1
