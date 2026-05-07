#!/usr/bin/env bash
# Run an HD API **console** example (no HL / no OpenGL) from OpenHaptics 3.4-0 — isolates HL+GL bugs.
#
# Usage: run-hd-console-example.sh [ExampleDirName]
# Env: HD_CONSOLE_EXAMPLE (default HelloHapticDevice) when no arg.
#
# If this still reports HD_COMM_ERROR in the servo callback → problem is vendor HD↔phantom/USB path,
# not QuickHaptics/RViz wrappers.

set -euo pipefail

BASE="/opt/OpenHaptics/Developer/3.4-0/examples/HD/console"

if [[ -n "${1:-}" ]]; then
  NAME="$1"
  shift
else
  NAME="${HD_CONSOLE_EXAMPLE:-HelloHapticDevice}"
fi

ROOT="${BASE}/${NAME}"
if [[ ! -d "${ROOT}" ]]; then
  echo "Missing ${ROOT}; check OpenHaptics examples tree." >&2
  exit 1
fi

export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export LD_LIBRARY_PATH="/opt/geomagic_touch_device_driver/lib:/usr/lib:${LD_LIBRARY_PATH:-}"

cd "${ROOT}"
TARGET="$(grep -E '^TARGET=' Makefile 2>/dev/null | head -1 | cut -d= -f2- | tr -d '\r')"
if [[ -z "${TARGET}" ]]; then
  echo "Could not parse TARGET= in ${ROOT}/Makefile" >&2
  exit 1
fi

if [[ ! -x "./${TARGET}" ]]; then
  echo "[run-hd-console-example] building ${TARGET} in ${ROOT}..."
  make CC=gcc CXX=g++ clean 2>/dev/null || true
  make CC=gcc CXX=g++
fi

exec "./${TARGET}" "$@"
