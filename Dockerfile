FROM ros:noetic-ros-base
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV ROS_PYTHON_VERSION=3

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  python3-pip python3-rosdep python3-rosinstall python3-vcstools \
  build-essential cmake git curl pkg-config \
  libgoogle-glog-dev libprotobuf-dev protobuf-compiler \
  libssl-dev \
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
  && rm -rf /var/lib/apt/lists/*

# Conan 1.x (required by ros_kortex)
RUN pip3 install --no-cache-dir "conan>=1.52,<2" \
  && conan config set general.revisions_enabled=1 \
  && (conan profile new default --detect 2>/dev/null || true) \
  && conan profile update settings.compiler.libcxx=libstdc++11 default

WORKDIR /catkin_ws
COPY catkin_ws/src /catkin_ws/src

# Optional: place OpenHaptics under ./openhaptics in the build context and set -DOPENHAPTICS_ROOT
ARG OPENHAPTICS_ROOT=
ENV OPENHAPTICS_ROOT=${OPENHAPTICS_ROOT}

RUN rosdep init 2>/dev/null || true \
  && rosdep update --include-eol-distros \
  && apt-get update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src -y \
      --skip-keys="openhaptics-ae moveit_fake_controller_manager" \
      --rosdistro=${ROS_DISTRO}

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
  && cd /catkin_ws \
  && catkin_make -DCMAKE_BUILD_TYPE=Release \
       -DCATKIN_BLACKLIST_PACKAGES='kortex_gazebo;kortex_examples' \
       -j$(nproc)"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "kinova_geomagic_demo", "demo_rviz.launch"]
