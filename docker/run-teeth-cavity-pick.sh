#!/usr/bin/env bash
# Build (once) and run OpenHaptics QuickHaptics example TeethCavityPickGLUT ("cavity" demo).
#
# If you see HD_COMM_CONFIG_ERROR / "Failed to initialize haptic device":
#   - Geomagic Touch must be USB-connected; container needs privileged + host network (see compose).
#   - Pair/calibrate once: docker compose --profile setup run --rm geomagic-setup
#
# Optional env overrides: OH_SDK_BASE, LIBGL_ALWAYS_SOFTWARE=0 (host GPU / DRI passthrough).
set -euo pipefail
ROOT="/opt/OpenHaptics/Developer/3.4-0/QuickHaptics/examples/TeethCavityPick/TeethCavityPickGLUT"
if [[ ! -d "${ROOT}" ]]; then
  echo "Missing ${ROOT}; OpenHaptics examples tree not installed in this image." >&2
  exit 1
fi

# Mesh/resources loaders in QuickHaptics still look for this legacy SensAble path.
export OH_SDK_BASE="${OH_SDK_BASE:-/opt/OpenHaptics/Developer/3.4-0}"

# Must match vendor install — same as /etc/profile.d on a native Touch driver install.
export GTDD_HOME="${GTDD_HOME:-/opt/geomagic_touch_device_driver}"

# PhantomIO / Geomagic Touch user-space driver (must resolve before generic /usr/lib).
export LD_LIBRARY_PATH="/opt/geomagic_touch_device_driver/lib:/usr/lib:${LD_LIBRARY_PATH:-}"

# Without GPU/device passthrough, DRI drivers (e.g. nouveau) often fail inside Docker — use Mesa
# software rasterizer. Set LIBGL_ALWAYS_SOFTWARE=0 and pass /dev/dri if you use host GPU.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

cd "${ROOT}"
if [[ ! -x ./TeethCavityPickGLUT ]]; then
  echo "[run-teeth-cavity-pick] building TeethCavityPickGLUT..."
  make CC=g++ CXX=g++ clean 2>/dev/null || true
  make CC=g++ CXX=g++
fi
exec ./TeethCavityPickGLUT "$@"
