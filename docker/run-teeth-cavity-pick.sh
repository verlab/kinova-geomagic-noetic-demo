#!/usr/bin/env bash
# Build (once) and run OpenHaptics QuickHaptics example TeethCavityPickGLUT ("cavity" demo).
#
# If you see HD_COMM_CONFIG_ERROR / "Failed to initialize haptic device":
#   - Geomagic Touch must be USB-connected; container needs privileged + host network (see compose).
#   - Pair/calibrate once: docker compose --profile setup run --rm geomagic-setup
#
# Optional env: OH_SDK_BASE, TEETH_FORCE_SOFTWARE_GL=1 (llvmpipe if GPU/GLX falha).
set -euo pipefail
ROOT="/opt/OpenHaptics/Developer/3.4-0/QuickHaptics/examples/TeethCavityPick/TeethCavityPickGLUT"
if [[ ! -d "${ROOT}" ]]; then
  echo "Missing ${ROOT}; OpenHaptics examples tree not installed in this image." >&2
  exit 1
fi

# Mesh/resources loaders in QuickHaptics still look for this legacy SensAble path.
export OH_SDK_BASE="${OH_SDK_BASE:-/opt/OpenHaptics/Developer/3.4-0}"

# 3D Systems: GTDD_HOME is the *config* dir (pairing), not /opt/geomagic_touch_device_driver.
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"

# PhantomIO + Qt from vendor install path (libraries are not under GTDD_HOME).
export LD_LIBRARY_PATH="/opt/geomagic_touch_device_driver/lib:/usr/lib:${LD_LIBRARY_PATH:-}"

# QuickHaptics + GLUT: com LIBGL_ALWAYS_SOFTWARE=1 (llvmpipe) no Docker é comum o stylus sentir-força /
# mover, mas o ecrã 3D ficar **congelado**. Por omissão não fixamos llvmpipe — usa GLX acelerado
# junto com `ipc: host` no docker-compose (MIT-SHM). Para voltar ao software renderer:
#   TEETH_FORCE_SOFTWARE_GL=1 docker compose --profile examples up teeth-cavity-pick
case "${TEETH_FORCE_SOFTWARE_GL:-0}" in
  1|true|yes|on) export LIBGL_ALWAYS_SOFTWARE=1 ;;
  *)
    unset LIBGL_ALWAYS_SOFTWARE 2>/dev/null || true
    export vblank_mode=0
    ;;
esac

cd "${ROOT}"
if [[ ! -x ./TeethCavityPickGLUT ]]; then
  echo "[run-teeth-cavity-pick] building TeethCavityPickGLUT..."
  make CC=g++ CXX=g++ clean 2>/dev/null || true
  make CC=g++ CXX=g++
fi
exec ./TeethCavityPickGLUT "$@"
