#!/usr/bin/env bash
# Geomagic Touch Diagnostic / calibration helper (vendor binary under /opt/geomagic_touch_device_driver).
# Same environment as geomagic-touch-setup. Requires X11 + DISPLAY (see docker-compose).
set -euo pipefail

GM="${GTDD_HOME:-/opt/geomagic_touch_device_driver}"

export GTDD_HOME="${GTDD_HOME:-/opt/geomagic_touch_device_driver}"
export LD_LIBRARY_PATH="${GTDD_HOME}/lib:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="${GTDD_HOME}/lib/plugins"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

try_exec() {
  local p="$1"
  shift
  if [[ -x "$p" ]]; then
    exec "$p" "$@"
  fi
}

try_exec "${GM}/Geomagic_Touch_Diagnostic" "$@"
try_exec "${GM}/geomagic_touch_diagnostic" "$@"
try_exec "${GM}/Geomagic_Touch_Calibration" "$@"
try_exec "${GM}/geomagic_touch_calibration" "$@"

# Some vendor trees ship odd names — pick the first *Diagnostic* executable in the install root.
shopt -s nullglob
for p in "${GM}"/*[Dd]iagnostic* "${GM}"/bin/*[Dd]iagnostic*; do
  if [[ -f "$p" && -x "$p" ]]; then
    exec "$p" "$@"
  fi
done
shopt -u nullglob

echo "No diagnostic/calibration binary found under ${GM}." >&2
echo "Rebuild the image with geomagic_touch_device_driver in docker/vendor/." >&2
exit 1
