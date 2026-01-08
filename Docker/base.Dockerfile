# ROS2 Humble base image for Unitree GO1 ROS2 interface and applications
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ROS 2 ENVs (generici; NON forziamo CycloneDDS qui)
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=43 \
    LDS_MODEL="none"

RUN apt-get update && apt-get install -y --no-install-recommends \
      # base
      ca-certificates curl gnupg2 dirmngr tzdata software-properties-common \
      lsb-release locales \
      \
      # build / toolchain
      build-essential cmake ninja-build pkg-config git nano \
      \
      # python / colcon
      python3-colcon-common-extensions python3-rosdep python3-pip python3-setuptools \
      python3-numpy \
      \
      # debug / inspection
      gdb gdbserver strace lsof procps psmisc \
      \
      # networking
      iproute2 iputils-ping net-tools dnsutils ethtool \
      \
      # capabilities / udev / usb
      libcap2-bin udev libudev-dev \
      usbutils pciutils \
      libusb-1.0-0 libusb-1.0-0-dev \
      \
      # common C++ deps
      libboost-all-dev libtinyxml2-dev \
      libyaml-cpp-dev libeigen3-dev \
      \
      # V4L2 tools + headers (v4l2-ctl, compliance, ecc.)
      v4l-utils libv4l-dev \
      \
      # FFmpeg stack (useful for OpenCV videoio backend / codecs)
      ffmpeg \
      libavcodec-dev libavformat-dev libswscale-dev libavutil-dev \
      \
      # OpenCV headers + libs (match distro OpenCV used by ROS2)
      libopencv-dev python3-opencv \
      \
      # GStreamer runtime + full plugin sets (base/good/bad/ugly + libav)
      gstreamer1.0-tools \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly \
      gstreamer1.0-libav \
      gstreamer1.0-alsa gstreamer1.0-pulseaudio \
      gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-x \
      libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 \
      \
      # GStreamer dev headers (se compili/usi pkg-config nel tuo CMake)
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      \
      # ROS2 core (quelli che già avevi)
      ros-${ROS_DISTRO}-rclcpp \
      ros-${ROS_DISTRO}-rcpputils \
      ros-${ROS_DISTRO}-rcutils \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      \
      # ROS2 image pipeline
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-geometry \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-camera-calibration-parsers \
      ros-${ROS_DISTRO}-camera-info-manager \
      \
      # ROS2 useful camera debugging tools
      ros-${ROS_DISTRO}-image-tools \
      ros-${ROS_DISTRO}-image-view \
      ros-${ROS_DISTRO}-rqt-image-view \
      ros-${ROS_DISTRO}-rosbag2 \
      \
      # TF (quasi sempre serve con camera frame)
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      \
      # (opzionale ma spesso utile in driver)
      ros-${ROS_DISTRO}-diagnostic-updater \
      \
      # pcl bridge (come avevi)
      ros-${ROS_DISTRO}-pcl-conversions \
      \
      # demo nodes (come avevi)
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*

# Workspace skeleton
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# rosdep init (una volta nella base)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
