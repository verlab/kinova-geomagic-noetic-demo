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
| **Network** | Kinova reachable on LAN; Geomagic Touch on USB or Ethernet (see [Geomagic and Kinova network setup](#geomagic-and-kinova-network-setup)) |
| **Geomagic vendor archives** | Only if you **build** the image locally—place both `.tar.gz` under `docker/vendor/` (see **[Quick start (build from source)](#quick-start-build-from-source)** § vendor archives). **Skipped** when using [GHCR](#quick-start-ghcr-pre-built-image). |

---

## Quick start (GHCR—pre-built image)

CI publishes the image to **[GHCR](https://github.com/verlab/kinova-geomagic-noetic-demo/pkgs/container/kinova-geomagic-noetic-demo)** (`ghcr.io/verlab/kinova-geomagic-noetic-demo`). Typical tags: **`latest`** (from `master`), **`vX.Y.Z`** on releases.

You still **clone only the repo** (compose, launch files, [`docker-compose.yml`](docker-compose.yml), Geomagic bind-mount folder)—**no submodules needed** if you stay inside Docker:

```bash
git clone https://github.com/verlab/kinova-geomagic-noetic-demo.git
cd kinova-geomagic-noetic-demo
```

If the package is **private**, log in to GHCR:

```bash
echo <YOUR_GITHUB_TOKEN> | docker login ghcr.io -u <GITHUB_USERNAME> --password-stdin
```

Pull and tag locally so Compose matches [`docker-compose.yml`](docker-compose.yml) image name **`kinova-geomagic-noetic:demo`**:

```bash
docker pull ghcr.io/verlab/kinova-geomagic-noetic-demo:latest
docker tag ghcr.io/verlab/kinova-geomagic-noetic-demo:latest kinova-geomagic-noetic:demo
```

Configure **`./docker/geomagic-config`** (Geomagic pairing) and robotics network as in [Geomagic and Kinova network setup](#geomagic-and-kinova-network-setup), then:

```bash
xhost +local:root   # Linux X11, if required
docker compose up --no-build
```

Use **`--no-build`** so Compose does not try to rebuild; it uses the tagged image above.

Robot IP defaults to **`192.168.1.10`**. Override with environment or `.env`:

```bash
export KINOVA_ROBOT_IP=192.168.1.100
docker compose up --no-build
```

Optional: pin a **[release](https://github.com/verlab/kinova-geomagic-noetic-demo/releases)** tag instead of `latest`, e.g. `docker pull ghcr.io/verlab/kinova-geomagic-noetic-demo:v0.2.0` then tag as `kinova-geomagic-noetic:demo`.

---

## Quick start (build from source)

### 1. Clone with submodules

Required for **`ros_kortex`** when building locally:

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

## Geomagic and Kinova network setup

Compose uses **`network_mode: host`** and **`privileged: true`** so ROS, the Kinova driver, Geomagic LAN (mDNS / link-local), and real-time servo limits behave like on the host.

### Kinova Gen3 (Ethernet)

| Step | What to do |
|------|-------------|
| 1 | Connect the Gen3 controller / base to your LAN. Default IP used in this demo is **`192.168.1.10`**; adjust if you changed it in **[Kinova/Kortex Networking](https://github.com/Kinovarobotics/ros_kortex#documentation)** docs. **`ping`** the robot from the workstation before **`docker compose up`**. |
| 2 | On the PC, set that NIC to a static IP on the **same subnet** as the robot (e.g. `192.168.1.50/24` gateway optional) **or** use DHCP if your controller was reconfigured accordingly. **`ping`** the robot IP before starting Docker. |
| 3 | Open the controller **web interface** at `http://<controller_ip>` (when enabled) and verify **operational / safety** state allows motion. |
| 4 | **Firewall**: allow outbound TCP sessions from the PC to the controller (Kinova ROS driver uses fixed ports per Kortex docs). Easiest sanity check on Ubuntu: **`sudo ufw status`**—if blocking, **`sudo ufw allow from YOUR_PC to any`** to the robot IP or briefly disable firewall for troubleshooting. |
| 5 | In this demo, export **`KINOVA_ROBOT_IP=<controller_ip>`** (or `.env`) so **`demo_hardware.launch`** passes **`robot_ip:=...`** into **`kortex_driver`**. |

### Geomagic Touch (Ethernet / LAN pairing)

Needed when **`Default Device.config`** shows **`CardType=LAN`** and a **`HostName=*.local`** machine name.

| Step | What to do |
|------|-------------|
| 1 | Use a **direct cable PC ↔ Touch dongle/brick** if possible (no flaky consumer switch path for link-local discovery). Touch side often gets **`169.254.x.x`** (APIPA/link-local only). |
| 2 | On the **Ubuntu host**, set **that NIC** to **link-local only**: *Settings → Wired → IPv4 → Link-local only*, or **`nmcli`**: `sudo nmcli connection edit <wired>` → set **`ipv4.method`** to **`link-local`**, reconnect. Your PC interface should appear as **`169.254.y.z/16`** (different host part than Touch). **`ping 169.254....`** to the Touch’s IP after **`Geomagic_Touch_Setup`** shows it—or **`ping hostname.local`**. |
| 3 | The image runs **Avahi** + **`nss-mdns`** in the **[entrypoint](docker/entrypoint.sh)** so **`.local`** resolves **inside** the container; **`network_mode: host`** attaches to the host stack for multicast DNS. Ensure **nothing** blocks multicast on that NIC. |
| 4 | **Pairing**: `docker compose --profile setup run --rm geomagic-setup` completes pairing; **`./docker/geomagic-config/config/Default Device.config`** is bind-mounted — copy that directory to another machine if you reuse the **same Touch** pairing. |

### Geomagic Touch (USB)

| Step | What to do |
|------|-------------|
| 1 | Prefer **USB 2 motherboard port**, cable **without hub**. |
| 2 | **`lsusb`** for vendor **256f**. Install **`docker/udev/70-geomagic-touch.rules`** → `/etc/udev/rules.d/`, **`sudo udevadm control --reload-rules && sudo udevadm trigger`**. |
| 3 | The entrypoint disables **autosuspend** for Geomagic USB where possible—also disable **`usbcore.autosuspend`** on hostile kernels if servo still drops. |

### Verify before `docker compose up`

```bash
ping -c 2 "${KINOVA_ROBOT_IP:-192.168.1.10}"
# LAN Touch — example:
getent hosts epXXXXXXXX.local || ping -c 1 169.254.x.x   # hostname from Geomagic Touch Setup
docker compose --profile calibration run --rm geomagic-diagnostic    # GUI: optional vendor check
```

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
| `robot_ip:=<IP>` | Kinova Gen3 controller IP (**default:** `192.168.1.10`) |
| `geomagic_driver:=false` | Omit `omni_cartesian` if joint states come from simulation or another publisher |
| `rviz:=false` | Launch without RViz |

**Teleop behaviour** (`geomagic_kinova_teleop.py`): Cartesian end-effector mapping; force feedback uses **`tool_external_wrench`** on `/arm/force_feedback` (**`torque_mode:=false`** on `omni_cartesian`). Tuning params are set on this node inside `demo_hardware.launch` (see **`workspace_scale`**, **`force_scale`**, etc.).

**Device pairing (required before demos):** OpenHaptics expects **`./docker/geomagic-config/config/Default Device.config`** (bind-mounted into the container at **`/usr/share/3DSystems`**). Missing file ⇒ **`HD_COMM_ERROR`** / **`HD_COMM_CONFIG_ERROR`**.

Run Setup once (writes into the mounted folder on the host):

```bash
docker compose --profile setup run --rm geomagic-setup
```

Verify on host:

```bash
ls -la docker/geomagic-config/config/
docker compose run --rm demo ls -la "/usr/share/3DSystems/config/"
```

You should see **`Default Device.config`** (and possibly calibration files). If missing, re-run Setup ([LAN vs USB networking](#geomagic-and-kinova-network-setup)).

**USB permissions on the host:** copy **`docker/udev/70-geomagic-touch.rules`** to **`/etc/udev/rules.d/`** if needed; update VID/PID from **`lsusb`**.

If **`docker/geomagic-config/config/`** looks wrong or corrupted, recreate only that directory (restore from backup—**avoid committing pairing secrets if the repo is public**), then re-run **`geomagic-setup`**.

### Optional: OpenHaptics QuickHaptics GLUT examples (`--profile examples`)

Non-ROS demos from the **OpenHaptics 3.4-0** SDK (they build on first run inside the container). **QuickHaptics GLUT** services need **`DISPLAY`**; **`qh-hd-console`** (**HelloHapticDevice**) uses the HD-only console examples and normally does **not** need OpenGL/X11.

| Compose service | Notes |
|-----------------|--------|
| **`qh-hd-console`** | **HD-only** (`HelloHapticDevice`): no HL, no OpenGL — separates QuickHaptics from the raw servo/USB path (**`HD_CONSOLE_EXAMPLE`** picks other console demos). Use **`docker compose … run --rm -it qh-hd-console`** (TTY for prompts). |
| `teeth-cavity-pick` | Teeth / cavity tutorial |
| `qh-simple-sphere` | Lightest QuickHaptics; **`TeapotTex.obj`** is baked into that example mesh path |
| `qh-complex-scene` | busier scene |
| `qh-earth-spin` | |
| `qh-pick-apples` | |
| `qh-shape-depth-feedback` | |
| `qh-skull-coulomb` | |
| `qh-spongy-cow` | |

```bash
docker compose --profile examples run --rm qh-simple-sphere
docker compose --profile examples run --rm -it qh-hd-console
# or: docker compose --profile examples up teeth-cavity-pick
```

Experimental host OpenGL instead of forced Mesa llvm-pipe: **`QH_TRY_GPU_GL=1`** (or legacy **`TEETH_TRY_GPU_GL=1`**), e.g.:

```bash
QH_TRY_GPU_GL=1 docker compose --profile examples run --rm qh-simple-sphere
```

**After a clean rebuild, if QuickHaptics still fails immediately after mesh load**, the culprit is unlikely to be our ROS/Makefile stack: **`TeapotTex`/SimpleSphere uses the smallest stock assets** and engages the servo the same way. Next step is **`qh-hd-console`** (HD API console example — no HL/OpenGL layers).

**Driver age:** On Ubuntu 20.04, **USB Geomagic Touch** units often behave better with a **Touch driver bundle newer than 2016.1‑1** (see informal packaging notes such as **[jhu-cisst-external/3ds-touch-openhaptics](https://github.com/jhu-cisst-external/3ds-touch-openhaptics)**; Ethernet adapters sometimes behaved better historically). Obtain a fresher tarball from **3D Systems**, update `docker/vendor` + `docker/install_vendor_geomagic.sh` if paths change, and rebuild — this demo repo ships **2016.1‑1** for reproducibility only.

**“Communication Error: Check the device connection and configuration”** is the vendor string for **`HD_COMM_ERROR`** when the servo session drops—not proof of bad X11. If Diagnostic opens but **`qh-simple-sphere`** / **`qh-hd-console`** fail after **Found device**, treat USB/kernel/driver as primary.

Scripts: **`docker/run-quickhaptics-glut-example.sh`** (env `QH_EXAMPLE_REL`), **`docker/run-hd-console-example.sh`** (env `HD_CONSOLE_EXAMPLE`).

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

- **Geomagic buttons:** dark button **hold** = teleop clutch (release to freeze arm); light button **press** = toggle **Robotiq 2F-85** gripper (open ↔ closed).
- **Haptics:** `omni_cartesian` uses **`torque_mode:=false`**; Cartesian forces arrive on **`/arm/force_feedback`** as **`sensor_msgs/ChannelFloat32`** (`fx_fy_fz`).

---

## Troubleshooting

| Issue | Action |
|-------|--------|
| Noetic packages missing in `rosdep` | Run `rosdep update --include-eol-distros` |
| `moveit_fake_controller_manager` unavailable | Dockerfile skips via rosdep; install manually if you need full MoveIt demos |
| Joint states read as zero | Use `LC_ALL=en_US.UTF-8` and `LC_NUMERIC=en_US.UTF-8` (set in image and Compose) |
| `HD_COMM_ERROR` / `HD_COMM_CONFIG_ERROR` in any demo while **Diagnostic / Setup** work | **Most likely: missing `Default Device.config`.** Run **`docker compose --profile setup run --rm geomagic-setup`**, then verify **`ls docker/geomagic-config/config/`** (host) or **`docker compose run --rm demo ls /usr/share/3DSystems/config/`**. |
| `HL_DEVICE_ERROR` / `HD_COMM_ERROR` (**LAN Touch**) | **`CardType=LAN`** in **`Default Device.config`**: NIC must use **link-local**, **Avahi**/mDNS must resolve **`*.local`**, **`network_mode: host`**. Use [Geomagic and Kinova network setup](#geomagic-and-kinova-network-setup). |
| `HL_DEVICE_ERROR` / `HD_COMM_ERROR` (TeapotTex / **`qh-simple-sphere`**) | **Not** a “heavy” GL mesh: **`TeapotTex.obj`** fails the same ⇒ servo/USB/Phantom path. **`qh-hd-console`** isolates HL+GL. Same failure ⇒ USB/kernel/driver (**[community bundles](https://github.com/jhu-cisst-external/3ds-touch-openhaptics)**), etc. Try **USB 2**, **no hub**, **`QH_TRY_GPU_GL=1`**. |

### Known issues (tutorials / vendor tools)

- **Locale (`LANG` / `LC_NUMERIC` / `LC_ALL`):** tutorials warn that non‑English locales can make haptic **position read as constant or zero**. In **Docker**, the image and `docker-compose.yml` already set **`LANG`**, **`LC_ALL`**, and **`LC_NUMERIC`** to **`en_US.UTF-8`**, matching that guidance for processes running **inside the container**. Native installs on the host must set locale there yourself.
- **`QBackingStore::flush()… non-exposed window` when closing **Geomagic Touch Diagnostic**:** comes from the **vendor‑bundled Qt** in the Geomagic driver package, not from this repo. It is a **harmless console warning** on exit (see [QTBUG-28613](https://bugreports.qt.io/browse/QTBUG-28613)); you can ignore it. We do not ship a newer Qt for that binary.

No software stack can honestly “guarantee” every third‑party GUI or firmware edge case; the items above describe what **this Docker setup already applies** versus what remains **upstream / vendor**.

---

## License

See [`LICENSE`](LICENSE). Third-party packages (`ros_kortex`, `geomagic_control`, vendor binaries) remain under their respective licenses.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).
