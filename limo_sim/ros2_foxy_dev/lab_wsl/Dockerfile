FROM ubuntu:20.04

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=foxy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Based on ../lab, with native dependencies found in workspace/src as well.
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion build-essential ca-certificates cmake curl git gnupg2 \
    less locales lsb-release nano software-properties-common sudo tree vim wget \
    pkg-config python3-dev python3-pip python3-setuptools swig \
    libboost-program-options-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
    libgl1-mesa-dri libgl1-mesa-glx libglu1-mesa libopencv-dev \
    libturbojpeg0-dev libudev-dev libusb-1.0-0-dev libuvc-dev \
    mesa-utils udev usbutils \
    && locale-gen en_US en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      python3-argcomplete python3-colcon-common-extensions python3-matplotlib \
      python3-numpy python3-opencv python3-pyqt5 python3-pytest python3-rosdep \
      python3-scipy python3-vcstool python3-yaml \
      ros-foxy-desktop ros-foxy-gazebo-ros-pkgs ros-foxy-cartographer-ros \
      ros-foxy-joint-state-publisher-gui ros-foxy-nav2-bringup \
      ros-foxy-robot-localization ros-foxy-rqt-robot-steering ros-foxy-slam-toolbox \
      ros-foxy-teleop-twist-keyboard ros-foxy-twist-mux ros-foxy-xacro \
      ros-foxy-camera-info-manager ros-foxy-cv-bridge ros-foxy-image-geometry \
      ros-foxy-image-publisher ros-foxy-image-transport ros-foxy-tf2-eigen \
      ros-foxy-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_foxy_dev/lab_wsl/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && python3 -m pip check

# Dockerfile.dockerignore copies only manifests and the local YDLidar SDK.
# COLCON_IGNORE is deliberately excluded here: install dependencies for the
# physical drivers too, without enabling them in the mounted workspace.
COPY workspace/src/ /tmp/workspace-src/
RUN rosdep init \
    && rosdep update --rosdistro foxy --include-eol-distros \
    && apt-get update \
    && rosdep install --from-paths /tmp/workspace-src --ignore-src -y --rosdistro foxy \
    && cmake -S /tmp/workspace-src/YDLidar-SDK -B /tmp/ydlidar-build \
      -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && cmake --build /tmp/ydlidar-build --parallel 2 \
    && cmake --install /tmp/ydlidar-build \
    && ldconfig \
    && rm -rf /tmp/workspace-src /tmp/ydlidar-build /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
    && usermod -aG dialout,video,sudo "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && echo 'source /opt/ros/foxy/setup.bash' >> "/home/${USERNAME}/.bashrc" \
    && echo 'test ! -f /workspace/.lab_wsl/install/setup.bash || source /workspace/.lab_wsl/install/setup.bash' \
      >> "/home/${USERNAME}/.bashrc"

COPY ros2_foxy_dev/lab_wsl/ros-entrypoint.sh /usr/local/bin/ros-entrypoint
COPY ros2_foxy_dev/lab_wsl/workspace-deps.sh /usr/local/bin/workspace-deps
COPY ros2_foxy_dev/lab_wsl/collect-manifests.py /usr/local/bin/collect-manifests
RUN sed -i 's/\r$//' /usr/local/bin/ros-entrypoint /usr/local/bin/workspace-deps /usr/local/bin/collect-manifests \
    && chmod 0755 /usr/local/bin/ros-entrypoint /usr/local/bin/workspace-deps /usr/local/bin/collect-manifests

USER ${USERNAME}
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
CMD ["bash"]
