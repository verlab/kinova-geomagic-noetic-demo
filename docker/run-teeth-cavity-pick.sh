#!/usr/bin/env bash
# Build (once) and run OpenHaptics QuickHaptics example TeethCavityPickGLUT ("cavity" demo).
set -euo pipefail
ROOT="/opt/OpenHaptics/Developer/3.4-0/QuickHaptics/examples/TeethCavityPick/TeethCavityPickGLUT"
if [[ ! -d "${ROOT}" ]]; then
  echo "Missing ${ROOT}; OpenHaptics examples tree not installed in this image." >&2
  exit 1
fi
export LD_LIBRARY_PATH="/usr/lib:${LD_LIBRARY_PATH:-}"
cd "${ROOT}"
if [[ ! -x ./TeethCavityPickGLUT ]]; then
  echo "[run-teeth-cavity-pick] building TeethCavityPickGLUT..."
  make CC=g++ CXX=g++ clean 2>/dev/null || true
  make CC=g++ CXX=g++
fi
exec ./TeethCavityPickGLUT "$@"
