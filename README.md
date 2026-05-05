# Kinova Gen3 + Geomagic Touch — ROS Noetic demo

Tutorial-style workspace for **bilateral-style teleoperation**: drive a **Kinova Gen3** from a **3D Systems Geomagic Touch** (OpenHaptics) and feed simple haptic cues from the arm using **PyKDL** (gravity torques or the driver’s external-wrench estimate), similar in spirit to the older `2017-itv-teleoperacao-third-party` packages but **Python 3**, **Noetic**, and a **reproducible `ros_kortex` pin**.

## What is included

| Component | Source |
|-----------|--------|
| **Kinova driver / URDF / MoveIt configs** | Git submodule [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex) at tag **`v2.5.2`** (`noetic-devel` line; latest public release as of this repo’s creation). |
| **Geomagic driver (`omni_cartesian`) + messages** | Trimmed from `phantom_omni/geomagic_control` (Georgia Tech lineage); **optional OpenHaptics** build. |
| **Geomagic URDF** | `geomagic.urdf` + `meshes/*.stl` from `phantom_omni/geomagic_description` (visuals). Optional `geomagic_minimal.urdf` (primitives only) kept for quick tests. |
| **Demo nodes (`kinova_geomagic_demo`)** | New: Cartesian velocity IBVS-style teleop + haptic bridge node. |

The snapshot under `2017-itv-teleoperacao-third-party/ros_kortex` is **not** maintained as closely as upstream; this repo tracks **upstream `v2.5.2`** explicitly.

## Prerequisites (host)

- **Docker** and **Docker Compose** v2.
- For GUI: X11/Wayland setup (`xhost +local:root` or your usual X11 forwarding pattern).
- **Real robot**: Gen3 reachable on the LAN (default `192.168.1.10` in launch files).
- **Real Geomagic Touch**: [OpenHaptics / 3D Systems SDK](https://www.3dsystems.com/support/material-downloads) installed when building `geomagic_control` (see below).

## Troubleshooting

- **rosdep skips Noetic** — Noetic may be marked end-of-life in the rosdistro index; use `rosdep update --include-eol-distros` (as in the Dockerfile).
- **`moveit_fake_controller_manager`** — Some mirrors omit this deb; the Docker build skips it via `--skip-keys moveit_fake_controller_manager`. Full MoveIt simulations may need extra packages installed manually.
- **OpenHaptics** — Without the SDK, `geomagic_control` still builds but **does not** produce the `omni_cartesian` binary; use `geomagic_driver:=false` and supply joint states another way, or mount vendor libraries at build time.

## Clone

```bash
git clone --recurse-submodules <path-or-url-to-this-repo>
cd <repo-directory>
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Run with Docker Compose (RViz visualization)

This uses **`ros:noetic-ros-base`** plus apt-installed RViz stack (smaller than `desktop-full`). The image builds **`ros_kortex`** from source (Conan 1.x); `kortex_gazebo` and `kortex_examples` are **blacklisted** to save build time and image size.

```bash
xhost +local:root   # if needed for RViz on X11
docker compose build
docker compose up
```

Default command: `roslaunch kinova_geomagic_demo demo_rviz.launch` — dual **RobotModel** (Kinova with `sim:=true` + Geomagic model) and two **joint_state_publisher_gui** sliders.

### Hardware stack (Kinova + Geomagic)

1. **Build the image with OpenHaptics** (install SDK on the host, then copy headers/libs into the build context, e.g. `./openhaptics/include` and `./openhaptics/lib`), and extend the Dockerfile `COPY` + `ENV OPENHAPTICS_ROOT=/openhaptics`, **or** bind-mount the SDK at build time. Without OpenHaptics, `omni_cartesian` is skipped but the rest still builds.

2. **USB / udev**: ensure the Touch is accessible (often `/dev/hidraw*`). The compose file uses `privileged: true` for convenience; tighten this to specific `devices`/`group_add` in production.

3. **Run**:

```bash
docker compose run --rm demo \
  roslaunch kinova_geomagic_demo demo_hardware.launch robot_ip:=192.168.1.10
```

Useful launch arguments:

| Argument | Meaning |
|----------|---------|
| `geomagic_driver:=false` | Skip `omni_cartesian` if you inject joint states another way. |
| `haptic_source:=robot_wrench` | Use `/my_gen3/base_feedback` tool wrench (real robot; default). |
| `haptic_source:=kdl_gravity` | Use gravity torque + LS wrench mapping (no F/T sensor; educational). |

### Operating the demo

- **Geomagic dark button** (as in the legacy excavator launch): enable teleop relative to the current stylus pose vs end-effector pose.
- **Geomagic light button**: disable teleop (zero joint-speed command).
- **Haptics**: `omni_cartesian` is configured with `torque_mode:=false` so **`ChannelFloat32`** on `/arm/force_feedback` is interpreted as cartesian force components (matches the driver callback).

## Native build (Ubuntu 20.04 / ROS Noetic)

```bash
sudo apt install python3-catkin-tools python3-pip
pip3 install --user 'conan>=1.52,<2'
conan config set general.revisions_enabled=1
conan profile new default --detect  # if missing
conan profile update settings.compiler.libcxx=libstdc++11 default

mkdir -p ~/ws/src
ln -sf "$(pwd)/catkin_ws/src/*" ~/ws/src  # or copy
cd ~/ws
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -y --rosdistro=noetic \
  --skip-keys=openhaptics-ae --skip-keys=moveit_fake_controller_manager
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_BLACKLIST_PACKAGES='kortex_gazebo;kortex_examples'
source devel/setup.bash
roslaunch kinova_geomagic_demo demo_rviz.launch
```

## Nodes (minimal stack)

- **`geomagic_kinova_cartesian_teleop.py`** — Subscribes to `/geomagic/joint_states`, maps stylus pose to desired EE position (relative delta), damped Jacobian pseudoinverse → joint velocities → `SendJointSpeedsCommand`.
- **`kdl_haptic_feedback.py`** — Publishes `sensor_msgs/ChannelFloat32` on `/arm/force_feedback` for `omni_cartesian`.
- **`fake_geomagic_joints.py`** — Optional sinusoidal joint motion (no Touch).

## Relationship to the legacy VERLAB tree

Concepts and topic naming follow:

- `kinova-gen-3/launch/excavator_teleop.launch` — orchestration pattern.
- `kinova-gen-3/scripts/*kinova*.py` — Kinova services + button semantics (simplified here).
- `phantom_omni/geomagic_control` — OpenHaptics bridge.

This repo intentionally drops Pioneer/mobile-base coupling and Python 2 idioms.

## License

See `LICENSE`. Third-party packages retain their original licenses where noted (`geomagic_control`, `ros_kortex`).

## Contributing

See `CONTRIBUTING.md`.
