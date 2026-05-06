#!/usr/bin/env bash
set -e
unset vblank_mode 2>/dev/null || true
export GTDD_HOME="${GTDD_HOME:-/usr/share/3DSystems}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec "$@"
