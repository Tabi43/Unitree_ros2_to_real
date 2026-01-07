# ROS2 Humble base image for Unitree GO1 ROS2 interface and applications
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ROS 2 ENVs (generici; NON forziamo CycloneDDS qui)
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=6 \
    LDS_MODEL="none"

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg2 dirmngr tzdata software-properties-common \
      build-essential cmake git nano \
      python3-colcon-common-extensions python3-rosdep \
      libcap2-bin \
      iproute2 iputils-ping net-tools \
      libboost-all-dev \
      libtinyxml2-dev \
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
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Workspace skeleton
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# rosdep init (una volta nella base)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
