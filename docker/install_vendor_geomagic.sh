#!/usr/bin/env bash
# Installs OpenHaptics 3.4 (headers + libs) and Geomagic Touch device driver payload from
# tarballs shipped in docker/vendor (3D Systems / legacy SensAble packages).
set -euo pipefail

VENDOR_DIR="${1:?vendor dir with *.tar.gz}"

cd "${VENDOR_DIR}"

tar -xzf openhaptics_3.4-0-developer-edition-amd64.tar.gz
OH_ROOT="${VENDOR_DIR}/openhaptics_3.4-0-developer-edition-amd64"
cp -a "${OH_ROOT}/usr/include/HD" /usr/include/
cp -a "${OH_ROOT}/usr/include/HDU" /usr/include/
cp -a "${OH_ROOT}/usr/include/HL" /usr/include/
cp -a "${OH_ROOT}/usr/lib/"* /usr/lib/
ln -sf libHD.so.3.4.0 /usr/lib/libHD.so
ln -sf libHL.so.3.4.0 /usr/lib/libHL.so

tar -xzf geomagic_touch_device_driver_2016.1-1-amd64.tar.gz
GM_ROOT="${VENDOR_DIR}/geomagic_touch_device_driver_2016.1-1-amd64"
cp -a "${GM_ROOT}/usr/lib/libPhantomIOLib42.so" /usr/lib/
cp -a "${GM_ROOT}/opt/geomagic_touch_device_driver" /opt/

ldconfig

rm -rf "${OH_ROOT}" "${GM_ROOT}"
