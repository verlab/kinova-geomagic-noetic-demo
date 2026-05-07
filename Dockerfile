FROM ros:noetic-ros-base
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV ROS_PYTHON_VERSION=3
# OpenHaptics / phantom_omni readme: numeric locale must be US English or joint reads become zero.
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LC_NUMERIC=en_US.UTF-8
ENV OPENHAPTICS_ROOT=/usr

# Mesa DRI (*.so incl. llvmpipe) para GL software no contentor — necessário quando LIBGL_ALWAYS_SOFTWARE=1
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  locales \
  python3-pip python3-rosdep python3-rosinstall python3-vcstools \
  build-essential cmake git curl pkg-config \
  libgoogle-glog-dev libprotobuf-dev protobuf-compiler \
  libssl-dev \
  libbullet-dev \
  libncurses5-dev \
  libncurses5 \
  libtinfo5 \
  freeglut3-dev \
  libglu1-mesa-dev \
  libgl1-mesa-dri \
  ros-noetic-rviz \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-xacro \
  ros-noetic-tf2-ros \
  ros-noetic-kdl-parser-py \
  ros-noetic-control-msgs \
  ros-noetic-actionlib \
  ros-noetic-actionlib-msgs \
  ros-noetic-moveit-ros-planning-interface \
  python3-numpy \
  python3-pykdl \
  libeigen3-dev \
  dbus avahi-daemon avahi-utils libnss-mdns \
  iproute2 iputils-ping \
  && rm -rf /var/lib/apt/lists/* \
  && locale-gen en_US.UTF-8 \
  && sed -i 's/^hosts:.*/hosts:          files mdns4_minimal [NOTFOUND=return] dns/' /etc/nsswitch.conf

# Conan 1.x (required by ros_kortex)
RUN pip3 install --no-cache-dir "conan>=1.52,<2" \
  && conan config set general.revisions_enabled=1 \
  && (conan profile new default --detect 2>/dev/null || true) \
  && conan profile update settings.compiler.libcxx=libstdc++11 default

COPY docker/vendor/openhaptics_3.4-0-developer-edition-amd64.tar.gz \
     docker/vendor/geomagic_touch_device_driver_2016.1-1-amd64.tar.gz \
     /tmp/geomagic-vendor/

COPY docker/install_vendor_geomagic.sh /tmp/install_vendor_geomagic.sh
RUN chmod +x /tmp/install_vendor_geomagic.sh \
  && /tmp/install_vendor_geomagic.sh /tmp/geomagic-vendor \
  && rm -rf /tmp/geomagic-vendor /tmp/install_vendor_geomagic.sh

# GTDD_HOME must be the *configuration* directory (3D Systems install guide), not the driver tree under /opt.
# Pairing from Geomagic_Touch_Setup is stored here; OpenHaptics + libPhantom read the same path.
# Driver binaries/libs stay in /opt/geomagic_touch_device_driver (see docker/geomagic-touch-setup.sh).
RUN mkdir -p /tmp/xdg-runtime-root && chmod 700 /tmp/xdg-runtime-root \
 && mkdir -p /usr/share/3DSystems/config && chmod 755 /usr/share/3DSystems /usr/share/3DSystems/config
ENV GTDD_HOME=/usr/share/3DSystems
ENV XDG_RUNTIME_DIR=/tmp/xdg-runtime-root

COPY docker/geomagic-touch-setup.sh /usr/local/bin/geomagic-touch-setup
COPY docker/geomagic-touch-diagnostic.sh /usr/local/bin/geomagic-touch-diagnostic
COPY docker/run-quickhaptics-glut-example.sh /usr/local/bin/run-quickhaptics-glut-example.sh
COPY docker/run-hd-console-example.sh /usr/local/bin/run-hd-console-example.sh
COPY docker/run-teeth-cavity-pick.sh /usr/local/bin/run-teeth-cavity-pick
RUN chmod +x /usr/local/bin/geomagic-touch-setup /usr/local/bin/geomagic-touch-diagnostic \
    /usr/local/bin/run-quickhaptics-glut-example.sh /usr/local/bin/run-hd-console-example.sh \
    /usr/local/bin/run-teeth-cavity-pick

COPY docker/udev/70-geomagic-touch.rules /etc/udev/rules.d/70-geomagic-touch.rules

WORKDIR /catkin_ws
COPY catkin_ws/src /catkin_ws/src

RUN rosdep init 2>/dev/null || true \
  && rosdep update --include-eol-distros \
  && apt-get update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src -y \
      --skip-keys="openhaptics-ae moveit_fake_controller_manager" \
      --rosdistro=${ROS_DISTRO} \
  && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
  && cd /catkin_ws \
  && catkin_make -DCMAKE_BUILD_TYPE=Release \
       -DCATKIN_BLACKLIST_PACKAGES='kortex_gazebo;kortex_examples' \
       -j$(nproc)"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "kinova_geomagic_demo", "demo_rviz.launch"]
