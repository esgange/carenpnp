FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    git \
    build-essential \
    cmake \
    libgflags-dev \
    nlohmann-json3-dev \
    libdw-dev \
    libssl-dev \
    mesa-utils \
    libgl1 \
    libgoogle-glog-dev \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-image-publisher \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-xacro \
    ros-humble-backward-ros \
    ros-humble-orbbec-camera \
    ros-humble-orbbec-description \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /workspaces/DOBOT_pickn_place

COPY . /workspaces/DOBOT_pickn_place

RUN source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys="opencv2 message_generation joint_state_publisher" \
    && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
