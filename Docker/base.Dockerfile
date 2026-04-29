# syntax=docker/dockerfile:1

# Base image for the Unitree Go1 ROS 2 interface.
# Keep this Dockerfile deliberately simple: no BuildKit apt cache mounts,
# no strict nounset shell options, and no comments inside apt package lists.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Generic ROS 2 runtime defaults. Do not force CycloneDDS here; the interface
# entrypoint can select it when needed.
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=43 \
    LDS_MODEL=none

# Sanity check: the upstream ROS image must contain a valid ROS 2 Humble setup.
RUN set -ex; \
    test -f "/opt/ros/${ROS_DISTRO}/setup.bash"; \
    bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && echo ROS_DISTRO=${ROS_DISTRO}"

# Base OS tools, build toolchain, Python tooling, colcon and rosdep.
RUN set -ex; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        dirmngr \
        tzdata \
        software-properties-common \
        lsb-release \
        locales \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        git \
        nano \
        python3-dev \
        libpython3-dev \
        python3.10-dev \
        python3-wheel \
        python3-pip \
        python3-setuptools \
        python3-numpy \
        python3-argcomplete \
        python3-vcstool \
        python3-colcon-common-extensions \
        python3-rosdep; \
    python_minor="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"; \
    multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"; \
    test -f "/usr/include/python${python_minor}/Python.h"; \
    test -e "/usr/lib/${multiarch}/libpython${python_minor}.so"; \
    rm -rf /var/lib/apt/lists/*

# Debugging, networking, USB/udev, capabilities and common C++ libraries.
RUN set -ex; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        gdb \
        gdbserver \
        strace \
        lsof \
        procps \
        psmisc \
        iproute2 \
        iputils-ping \
        net-tools \
        dnsutils \
        ethtool \
        libcap2-bin \
        udev \
        libudev-dev \
        usbutils \
        pciutils \
        libusb-1.0-0 \
        libusb-1.0-0-dev \
        libboost-all-dev \
        libtinyxml2-dev \
        libyaml-cpp-dev \
        libeigen3-dev; \
    rm -rf /var/lib/apt/lists/*

# Camera/video stack: V4L2, FFmpeg, OpenCV and GStreamer.
RUN set -ex; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        v4l-utils \
        libv4l-dev \
        ffmpeg \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libavutil-dev \
        libopencv-dev \
        python3-opencv \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-alsa \
        gstreamer1.0-pulseaudio \
        gstreamer1.0-gl \
        gstreamer1.0-gtk3 \
        gstreamer1.0-x \
        libgstreamer1.0-0 \
        libgstreamer-plugins-base1.0-0 \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev; \
    rm -rf /var/lib/apt/lists/*

# ROS 2 packages used by the interface and by camera/debug workflows.
RUN set -ex; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-rcpputils \
        ros-${ROS_DISTRO}-rcutils \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-camera-calibration-parsers \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-image-tools \
        ros-${ROS_DISTRO}-image-view \
        ros-${ROS_DISTRO}-rqt-image-view \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-${ROS_DISTRO}-image-proc \
        ros-${ROS_DISTRO}-stereo-image-proc \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-demo-nodes-cpp; \
    rm -rf /var/lib/apt/lists/*

# rosdep initialization. This must be idempotent because some ROS images may
# already provide the default source list.
RUN set -ex; \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi; \
    rosdep update --rosdistro "${ROS_DISTRO}" || rosdep update

# Workspace skeleton.
RUN set -ex; \
    mkdir -p "${ROS_WS}/src"

WORKDIR ${ROS_WS}

# Make interactive shells useful when debugging inside the container.
RUN set -ex; \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc; \
    echo "cd ${ROS_WS}" >> /root/.bashrc
