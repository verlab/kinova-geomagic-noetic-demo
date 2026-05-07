#!/usr/bin/env bash
# Build (once) and run an OpenHaptics QuickHaptics GLUT binary from the 3.4-0 SDK tree.
#
# Usage:
#   run-quickhaptics-glut-example.sh <relative-path-under-QuickHaptics/examples> [args...]
# Env (alternative to first arg):
#   QH_EXAMPLE_REL   e.g. SimpleSphere/SimpleSphereGLUT (set by docker-compose per service)
#
# Optional env (see also docker-compose / teeth wrapper):
#   OH_SDK_BASE           default /opt/OpenHaptics/Developer/3.4-0
#   GTDD_HOME             pairing/config root (default /usr/share/3DSystems)
#   QH_TRY_GPU_GL / TEETH_TRY_GPU_GL   unset Mesa force when 1 (host GL; experimental)
#   QH_TARGET             override Makefile TARGET= binary name if needed
#
# "Communication Error: Check the device connection and configuration" (HD_COMM_ERROR /
# HL_DEVICE_ERROR during haptic rendering) appears after the device opens when the HL/HD
# servo engages — see README troubleshooting and comments in run-teeth-cavity-pick.sh.

set -euo pipefail

SDK_EXAMPLES="/opt/OpenHaptics/Developer/3.4-0/QuickHaptics/examples"

if [[ -n "${1:-}" ]]; then
  REL_PATH="$1"
  shift
else
  REL_PATH="${QH_EXAMPLE_REL:-}"
fi
if [[ -z "${REL_PATH}" ]]; then
  echo "usage: $0 <ExampleDir/ExampleGLUTsubdir>   or set QH_EXAMPLE_REL" >&2
  exit 1
fi

ROOT="${SDK_EXAMPLES}/${REL_PATH}"
if [[ ! -d "${ROOT}" ]]; then
  echo "Missing QuickHaptics example dir: ${ROOT}" >&2
  exit 1
fi

export OH_SDK_BASE="${OH_SDK_BASE:-/opt/OpenHaptics/Developer/3.4-0}"
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export LD_LIBRARY_PATH="/opt/geomagic_touch_device_driver/lib:/usr/lib:${LD_LIBRARY_PATH:-}"

unset vblank_mode 2>/dev/null || true
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD 2>/dev/null || true
case "${QH_TRY_GPU_GL:-${TEETH_TRY_GPU_GL:-0}}" in
  1|true|yes|on)
    unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE 2>/dev/null || true
    ;;
esac

cd "${ROOT}"
if [[ -n "${QH_TARGET:-}" ]]; then
  TARGET="${QH_TARGET}"
else
  TARGET="$(grep -E '^TARGET=' Makefile 2>/dev/null | head -1 | cut -d= -f2- | tr -d '\r')"
fi
if [[ -z "${TARGET}" ]]; then
  echo "Could not read TARGET= from ${ROOT}/Makefile; set QH_TARGET" >&2
  exit 1
fi

if [[ ! -x "./${TARGET}" ]]; then
  echo "[run-quickhaptics-glut-example] building ${TARGET} in ${ROOT}..."
  make CC=g++ CXX=g++ clean 2>/dev/null || true
  make CC=g++ CXX=g++
fi

exec "./${TARGET}" "$@"
