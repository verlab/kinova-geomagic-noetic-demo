# Vendor archives (proprietary)

Place these files here before `docker compose build` (already copied from `2017-itv-teleoperacao-third-party/phantom_omni/` for this lab):

- `openhaptics_3.4-0-developer-edition-amd64.tar.gz` — OpenHaptics developer edition
- `geomagic_touch_device_driver_2016.1-1-amd64.tar.gz` — Geomagic Touch device driver (GUI setup + `libPhantomIOLib42.so`)

Redistribution may be restricted by 3D Systems / vendor license; keep compliance in mind if you publish the image or this folder publicly.

## After build

The Dockerfile sets **`GTDD_HOME=/opt/geomagic_touch_device_driver`** (required for `libPhantomIOLib42` / OpenHaptics; without it you get “Config directory not set correctly” and the Touch does not enumerate). If you use an old image, rebuild or export **`GTDD_HOME`** in `docker compose`.

If the **USB** Touch still misbehaves on **Ubuntu 20.04+**, 3D Systems / VeRLab notes report that **older 2016** USB stacks can misbehave while **newer Touch drivers (2019+)** work better; Ethernet Touch was often more reliable with the 2016-era USB quirks. Consider a newer vendor tarball if problems persist after **`GTDD_HOME`** is correct.
