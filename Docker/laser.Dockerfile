# syntax=docker/dockerfile:1
# Laser scan driver image for Unitree GO1
FROM ros:humble-ros-base

# TARGETARCH is set automatically by BuildKit (amd64 | arm64)
ARG TARGETARCH

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ROS 2 ENVs
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=43

# ── System dependencies ──────────────────────────────────────────────────────
RUN --mount=type=cache,target=/var/cache/apt,id=laser-apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=laser-aptlists-${TARGETARCH},sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      # base
      ca-certificates curl gnupg2 software-properties-common \
      lsb-release locales \
      \
      # build / toolchain
      build-essential cmake pkg-config git nano \
      \
      # python / colcon
      python3-colcon-common-extensions python3-rosdep python3-pip python3-setuptools \
      \
      # serial port access
      libserial-dev \
      \
      # capabilities
      libcap2-bin udev libudev-dev \
      \
      # networking / debug
      iproute2 iputils-ping net-tools procps \
      \
      # ROS 2 packages needed for laser scan
      ros-${ROS_DISTRO}-rclcpp \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-std-msgs \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-laser-geometry \
      \
      # Hokuyo laser driver
      ros-${ROS_DISTRO}-urg-node \
    && apt-get -y autoremove && apt-get clean

# ── CycloneDDS config ────────────────────────────────────────────────────────
RUN mkdir -p /etc/cyclonedds
COPY Docker/cyclonedds/ /opt/cyclonedds/

# ── DDS env script ───────────────────────────────────────────────────────────
COPY Docker/unitree_dds_env.sh /usr/local/bin/unitree_dds_env.sh
RUN chmod +x /usr/local/bin/unitree_dds_env.sh

# rosdep init
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

# ─── Environment setup ───────────────────────────────────────────────────────

WORKDIR /root

RUN git clone https://github.com/RICE-unige/hokuyo_bringup

# ─────────────────────────────────────────────────────────────────────────────

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc 

# ── Entrypoint ───────────────────────────────────────────────────────────────
COPY Docker/laser_entrypoint.sh /usr/local/bin/laser_entrypoint.sh
RUN chmod +x /usr/local/bin/laser_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/laser_entrypoint.sh"]