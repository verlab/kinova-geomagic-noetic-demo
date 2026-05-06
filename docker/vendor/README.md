# Vendor archives (proprietary)

Place these files here before `docker compose build` (already copied from `2017-itv-teleoperacao-third-party/phantom_omni/` for this lab):

- `openhaptics_3.4-0-developer-edition-amd64.tar.gz` — OpenHaptics developer edition
- `geomagic_touch_device_driver_2016.1-1-amd64.tar.gz` — Geomagic Touch device driver (GUI setup + `libPhantomIOLib42.so`)

Redistribution may be restricted by 3D Systems / vendor license; keep compliance in mind if you publish the image or this folder publicly.

## After build

The Dockerfile sets **`GTDD_HOME=/usr/share/3DSystems`** (configuration / pairing directory per 3D Systems install guides). **`LD_LIBRARY_PATH`** still uses **`/opt/geomagic_touch_device_driver/lib`**. Do **not** point `GTDD_HOME` at `/opt/geomagic_touch_device_driver`; Touch Setup stores pairing under `$GTDD_HOME`, and OpenHaptics reads the same path.

Docker Compose mounts a named volume **`geomagic-touch-config`** on **`/usr/share/3DSystems`** so pairing persists across containers. Inspect with **`docker volume ls`** / **`docker volume inspect …`**. If you use an old image, rebuild.

If the **USB** Touch still misbehaves on **Ubuntu 20.04+**, 3D Systems / VeRLab notes report that **older 2016** USB stacks can misbehave while **newer Touch drivers (2019+)** work better; Ethernet Touch was often more reliable with the 2016-era USB quirks. Consider a newer vendor tarball if problems persist after **`GTDD_HOME`** is correct.

## Notes vs. older “Geomagic on ROS” / phantom_omni tutorials (e.g. OpenHaptics guide + `phantom_omni`)

| Topic | Typical legacy tutorial | This repo |
|--------|-------------------------|-----------|
| OpenHaptics tree | `/opt/OpenHaptics/Developer/3.4-0/` | Same content is copied into the image by `docker/install_vendor_geomagic.sh`. |
| Driver + Setup | `/opt/geomagic_touch_device_driver/`, `Geomagic_Touch_Setup` | Same paths; wrappers are `/usr/local/bin/geomagic-touch-setup` etc. |
| **`GTDD_HOME`** | Often omitted | Must be **`/usr/share/3DSystems`** (config + pairing), **not** `/opt/geomagic_touch_device_driver`. Legacy docs skip this; missing `GTDD_HOME` breaks OpenHaptics + pairing. |
| Persistence | N/A (bare metal) | **`geomagic-touch-config`** named volume on **`/usr/share/3DSystems`**. |
| ROS stack | `wstool` + [`fsuarez6/phantom_omni`](https://github.com/fsuarez6/phantom_omni) (`omni.launch`, Hydro-era) | **`geomagic_control`** + **`kinova_geomagic_demo`** on **Noetic**; not a drop-in substitute for `omni_common`. |
| Locale | Use `en_US.UTF-8` for joint/position reads | Already set in the Docker image (`LANG`, `LC_ALL`, **`LC_NUMERIC`**). |
| Ethernet / Touch X | IPv4 **Link-Local only** on the USB–Ethernet adapter + power-cycle device | Still valid; configure on the **host** network manager. |
| Diagnostic binary | Name should be **`Geomagic_Touch_Diagnostic`** (some copies have a typo `Geomagic_Touch_Geomagic_Touch_Diagnostic`) | `docker/geomagic-touch-diagnostic.sh` looks for the correct names under `/opt/geomagic_touch_device_driver/`. |

Install: tutorials run the vendor **`./install`** scripts; we unpack and **`cp`** the same payloads for reproducible Docker builds — same layout, different installer entrypoint.
