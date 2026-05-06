#!/usr/bin/env bash
set -e
export GTDD_HOME="${GTDD_HOME:-/opt/geomagic_touch_device_driver}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/xdg-runtime-root}"
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec "$@"
