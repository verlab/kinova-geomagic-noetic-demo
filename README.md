# kinova-geomagic-noetic

[![Docker build and push to GHCR](https://github.com/verlab/kinova-geomagic-noetic-demo/actions/workflows/docker-publish.yml/badge.svg)](https://github.com/verlab/kinova-geomagic-noetic-demo/actions/workflows/docker-publish.yml)
[![GitHub release](https://img.shields.io/github/v/release/verlab/kinova-geomagic-noetic-demo?sort=semver&label=release)](https://github.com/verlab/kinova-geomagic-noetic-demo/releases/latest)
[![GHCR image](https://img.shields.io/badge/ghcr.io-package-2496ED?logo=github)](https://github.com/verlab/kinova-geomagic-noetic-demo/pkgs/container/kinova-geomagic-noetic-demo)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros)](https://wiki.ros.org/noetic)
[![Docker](https://img.shields.io/badge/stack-Docker-2496ED?logo=docker&logoColor=white)](https://docs.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Last commit](https://img.shields.io/github/last-commit/verlab/kinova-geomagic-noetic-demo)](https://github.com/verlab/kinova-geomagic-noetic-demo/commits/master)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/20.04/)
[![ros_kortex](https://img.shields.io/badge/ros__kortex-v2.5.2-informational)](https://github.com/Kinovarobotics/ros_kortex/releases/tag/v2.5.2)

ROS **Noetic** reference workspace for **Kinova Gen3** teleoperation with a **Geomagic Touch** haptic device: Cartesian teleoperation via KDL, optional haptic feedback from the arm, **Docker** images with vendor **OpenHaptics** and **Geomagic** payloads, and reproducible **[ros_kortex](https://github.com/Kinovarobotics/ros_kortex) `v2.5.2`**.

Maintained by [VeRLab](https://github.com/verlab) (UFMG).

---

## Prerequisites

| Requirement | Notes |
|-------------|--------|
| **Docker** & **Docker Compose** v2 | Recommended workflow |
| **X11** | For RViz and vendor GUIs (`xhost +local:root` on Linux if needed) |
| **Geomagic vendor archives** | Place both `.tar.gz` files under `docker/vendor/` before building (see [Vendor archives](#vendor-archives)) |
| **Hardware** (optional) | Gen3 on LAN; Touch via USB or LAN (Touch X) |

---

## Quick start (Docker)

### 1. Clone

Clone this repository **with submodules** (required for `ros_kortex`):

```bash
git clone --recurse-submodules https://github.com/verlab/kinova-geomagic-noetic-demo.git
cd kinova-geomagic-noetic-demo
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Vendor archives

Ensure `docker/vendor/` contains:

- `openhaptics_3.4-0-developer-edition-amd64.tar.gz`
- `geomagic_touch_device_driver_2016.1-1-amd64.tar.gz`

These files are **not** redistributed by Kinova/VeRLab; obtain them under your 3D Systems license and copy them into `docker/vendor/` before build.

### 3. Build the image

```bash
docker compose build
```

This installs vendor libraries and headers inside the image, builds `ros_kortex`, and compiles `geomagic_control` (`omni_cartesian`). Expect a long first build.

### 4. Run the default demo (hardware teleop)

```bash
xhost +local:root   # Linux X11, if required
docker compose up
```

Default service command: **`demo_hardware.launch`** — real **Kinova Gen3** + **Geomagic** Cartesian teleop (`robot_ip` defaults to **192.168.1.10**; set **`KINOVA_ROBOT_IP`** in a `.env` file or your shell if different).

**RViz-only** (no robot / no haptic driver):  
`docker compose run --rm demo roslaunch kinova_geomagic_demo demo_rviz.launch`

Stop with `Ctrl+C`.

---

## Execution variants

### Real Kinova + Geomagic (hardware)

`docker compose up` already starts **`demo_hardware.launch`** (see above). To run a one-off with a custom command:

```bash
docker compose run --rm demo \
  roslaunch kinova_geomagic_demo demo_hardware.launch robot_ip:=192.168.1.10
```

Adjust `robot_ip` to your Gen3 controller. Compose uses `network_mode: host` for ROS and LAN devices.

| Launch argument | Purpose |
|-----------------|--------|
| `geomagic_driver:=false` | Omit `omni_cartesian` if joint states are supplied elsewhere |
| `haptic_source:=robot_wrench` | Use measured tool wrench from the driver (default on hardware) |
| `haptic_source:=kdl_gravity` | Gravity-based KDL mapping (no F/T sensor; illustrative) |

**Device pairing (USB or LAN, especially Ethernet Touch):** same image, one-off command:

```bash
docker compose run --rm demo geomagic-touch-setup
```

**USB permissions on the host:** copy `docker/udev/70-geomagic-touch.rules` to `/etc/udev/rules.d/` if needed; update VID/PID from `lsusb`.

### Optional: OpenHaptics TeethCavityPick (“cavity”) example

Non-ROS QuickHaptics sample (builds on first run inside the container):

```bash
docker compose --profile examples run --rm teeth-cavity-pick
```

Requires a working `DISPLAY`.

---

## Repository layout

| Path | Content |
|------|---------|
| `catkin_ws/src/ros_kortex` | Git submodule at [`Kinovarobotics/ros_kortex`](https://github.com/Kinovarobotics/ros_kortex) tag **`v2.5.2`** |
| `catkin_ws/src/geomagic_control` | ROS node `omni_cartesian`, OpenHaptics-based |
| `catkin_ws/src/geomagic_description` | URDF + STL meshes |
| `catkin_ws/src/kinova_geomagic_demo` | Teleoperation and haptic bridge nodes |
| `docker/` | Dockerfile helpers, udev sample, vendor install script |

---

## Native build (Ubuntu 20.04, ROS Noetic)

Use when Docker is not preferred. Install Conan 1.x and ROS dependencies as in the Dockerfile, then:

```bash
rosdep update --include-eol-distros
rosdep install --from-paths catkin_ws/src --ignore-src -y --rosdistro=noetic \
  --skip-keys=openhaptics-ae --skip-keys=moveit_fake_controller_manager
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_BLACKLIST_PACKAGES='kortex_gazebo;kortex_examples'
source devel/setup.bash
roslaunch kinova_geomagic_demo demo_rviz.launch
```

Install OpenHaptics and Geomagic vendor files on the host separately if building `omni_cartesian`.

---

## Operator notes

- **Geomagic buttons:** dark button enables teleop (relative stylus vs end-effector); light button disables (zero joint speeds).
- **Haptics:** `omni_cartesian` uses `torque_mode:=false`; `/arm/force_feedback` carries `sensor_msgs/ChannelFloat32` Cartesian force components.

---

## Troubleshooting

| Issue | Action |
|-------|--------|
| Noetic packages missing in `rosdep` | Run `rosdep update --include-eol-distros` |
| `moveit_fake_controller_manager` unavailable | Dockerfile skips via rosdep; install manually if you need full MoveIt demos |
| Joint states read as zero | Use `LC_ALL=en_US.UTF-8` and `LC_NUMERIC=en_US.UTF-8` (set in image and Compose) |
| `HD_COMM_ERROR` in `omni_cartesian` while **Geomagic Diagnostic / Setup** work in Docker | Rarely Docker itself if the vendor tools succeed in the same stack. **`docker compose build`** after C++ fixes, quit any other Touch app, verify `/arm/force_feedback` has **three** values; scheduler / force feedback threading was hardened in `omni_cartesian`. |

### Known issues (tutorials / vendor tools)

- **Locale (`LANG` / `LC_NUMERIC` / `LC_ALL`):** tutorials warn that non‑English locales can make haptic **position read as constant or zero**. In **Docker**, the image and `docker-compose.yml` already set **`LANG`**, **`LC_ALL`**, and **`LC_NUMERIC`** to **`en_US.UTF-8`**, matching that guidance for processes running **inside the container**. Native installs on the host must set locale there yourself.
- **`QBackingStore::flush()… non-exposed window` when closing **Geomagic Touch Diagnostic**:** comes from the **vendor‑bundled Qt** in the Geomagic driver package, not from this repo. It is a **harmless console warning** on exit (see [QTBUG-28613](https://bugreports.qt.io/browse/QTBUG-28613)); you can ignore it. We do not ship a newer Qt for that binary.

No software stack can honestly “guarantee” every third‑party GUI or firmware edge case; the items above describe what **this Docker setup already applies** versus what remains **upstream / vendor**.

---

## License

See [`LICENSE`](LICENSE). Third-party packages (`ros_kortex`, `geomagic_control`, vendor binaries) remain under their respective licenses.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).
