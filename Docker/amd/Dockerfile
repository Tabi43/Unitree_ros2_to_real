# Build:
# docker buildx build -f Dockerfile.amd64 \
#   -t udp_ros2_if:humble-amd64 \
#   --platform linux/amd64 .

FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ROS 2 ENVs
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=6 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    LDS_MODEL="none"

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg2 dirmngr tzdata software-properties-common \
      build-essential cmake git \
      python3-colcon-common-extensions python3-rosdep \
      libcap2-bin \
      iproute2 iputils-ping net-tools \
      libboost-all-dev \
      ros-humble-rclcpp \
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-geometry-msgs \
      ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Workspace
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copia il codice (usa .dockerignore per escludere build/, install/, log/, .git/, ecc.)
COPY . ${ROS_WS}/src/

# rosdep
RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Permessi realtime (best-effort)
RUN find ${ROS_WS}/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

# Startup
COPY Docker/start_interface.sh /usr/local/bin/start_interface.sh
RUN chmod +x /usr/local/bin/start_interface.sh
ENV START_CMD="ros2 run unitree_ros2_interface interface_node"

# Comodità shell
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source ${ROS_WS}/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/start_interface.sh"]
