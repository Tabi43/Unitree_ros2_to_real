# syntax=docker/dockerfile:1
ARG BASE_IMAGE=tabi43/unitree_ros2:base-if
FROM ${BASE_IMAGE}

# TARGETARCH is set automatically by BuildKit (amd64 | arm64)
ARG TARGETARCH

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ROS 2 ENVs
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=43 \
    LDS_MODEL="none"

# CycloneDDS
RUN mkdir -p /etc/cyclonedds
COPY Docker/cyclonedds/ /opt/cyclonedds/

# Workspace
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# --- Layer 1: dependency resolution (cached unless package.xml changes) ---
COPY ros2_ws/src/unitree_legged_msgs/package.xml  src/unitree_legged_msgs/package.xml
COPY ros2_ws/src/unitree_legged_sdk/package.xml   src/unitree_legged_sdk/package.xml
COPY ros2_ws/src/unitree_ros2_interface/package.xml src/unitree_ros2_interface/package.xml

RUN --mount=type=cache,target=/var/cache/apt,id=apt-${TARGETARCH},sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,id=aptlists-${TARGETARCH},sharing=locked \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# --- Layer 2: full source + build (only this layer rebuilds on code changes) ---
COPY ros2_ws/src src/

# NOTE: We intentionally do NOT use --mount=type=cache for colcon build/log.
# BuildKit cache-mount volumes persist inside the builder daemon and previously
# caused cross-arch contamination (amd64 CMake artifacts leaking into arm64).
# The BuildKit *layer* cache (--cache-from type=local) already caches this
# entire RUN layer when source hasn't changed, which covers the common case.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers "$(nproc)"

RUN find ${ROS_WS}/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

# Interface_setup.bash & entrypoint
COPY Docker/unitree_dds_env.sh /usr/local/bin/unitree_dds_env.sh
COPY Docker/interface_entrypoint.sh /usr/local/bin/interface_entrypoint.sh
RUN chmod +x /usr/local/bin/unitree_dds_env.sh /usr/local/bin/interface_entrypoint.sh

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

WORKDIR ${ROS_WS}

ENTRYPOINT ["/usr/local/bin/interface_entrypoint.sh"]
