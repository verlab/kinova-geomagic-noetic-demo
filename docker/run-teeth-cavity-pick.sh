#!/usr/bin/env bash
# Teeth Cavity QuickHaptics demo — thin wrapper around the generic runner (keeps backward
# compatibility with /usr/local/bin/run-teeth-cavity-pick and TEETH_TRY_GPU_GL).

set -euo pipefail
export QH_TRY_GPU_GL="${TEETH_TRY_GPU_GL:-${QH_TRY_GPU_GL:-0}}"
exec /usr/local/bin/run-quickhaptics-glut-example.sh TeethCavityPick/TeethCavityPickGLUT "$@"
