ARG BASE_IMAGE=tabi43/unitree_ros2:base-if
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG COLCON_PARALLEL_WORKERS=2

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=43 \
    LDS_MODEL=none

# -----------------------------------------------------------------------------
# Sanity checks
# -----------------------------------------------------------------------------
RUN set -ex; \
    test -n "${ROS_DISTRO:-}"; \
    test -f "/opt/ros/${ROS_DISTRO}/setup.bash"; \
    command -v rosdep >/dev/null; \
    command -v colcon >/dev/null

# -----------------------------------------------------------------------------
# Runtime configuration files
# -----------------------------------------------------------------------------
RUN mkdir -p /etc/cyclonedds /opt/cyclonedds
COPY Docker/cyclonedds/ /opt/cyclonedds/

# -----------------------------------------------------------------------------
# Workspace skeleton
# -----------------------------------------------------------------------------
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# -----------------------------------------------------------------------------
# Dependency layer
#
# Only package.xml files are copied here. Therefore this layer is rebuilt only
# when dependencies change, not whenever C++/Python source files change.
#
# Deliberately no BuildKit cache mount for /var/lib/apt/lists here: for ROS
# multi-arch builds it is more robust to use a fresh apt index per build step.
# -----------------------------------------------------------------------------
COPY ros2_ws/src/unitree_legged_msgs/package.xml        src/unitree_legged_msgs/package.xml
COPY ros2_ws/src/unitree_legged_sdk/package.xml         src/unitree_legged_sdk/package.xml
COPY ros2_ws/src/unitree_ros2_interface/package.xml     src/unitree_ros2_interface/package.xml
COPY ros2_ws/src/sport_controller/package.xml           src/sport_controller/package.xml

RUN set -ex; \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi; \
    rosdep update --rosdistro "${ROS_DISTRO}" || rosdep update; \
    apt-get update; \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        -r \
        -y; \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Source layer and build
# -----------------------------------------------------------------------------
COPY ros2_ws/src/ src/

RUN set -ex; \
    source "/opt/ros/${ROS_DISTRO}/setup.bash"; \
    colcon build \
        --symlink-install \
        --event-handlers console_direct+ \
        --parallel-workers "${COLCON_PARALLEL_WORKERS}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Give real-time priority capability where possible. Do not fail the image build
# if setcap is unavailable or unsupported in the builder environment.
RUN find "${ROS_WS}/install" -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

# -----------------------------------------------------------------------------
# Entrypoint
# -----------------------------------------------------------------------------
COPY Docker/unitree_dds_env.sh /usr/local/bin/unitree_dds_env.sh
COPY Docker/interface_entrypoint.sh /usr/local/bin/interface_entrypoint.sh

RUN set -ex; \
    chmod +x /usr/local/bin/unitree_dds_env.sh /usr/local/bin/interface_entrypoint.sh; \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc; \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

WORKDIR ${ROS_WS}
ENTRYPOINT ["/usr/local/bin/interface_entrypoint.sh"]