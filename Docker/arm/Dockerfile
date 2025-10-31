# Build:
# docker buildx build -f Dockerfile.arm64 \
#   -t udp_ros2_if:humble-arm64 \
#   --platform linux/arm64/v8 .

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
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

COPY . $ROS_WS/src/

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN find $ROS_WS/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

# Copy the Cyclone DDS configuration template
COPY Docker/cyclonedds.xml.template $ROS_WS/cyclonedds/cyclonedds.xml.template
COPY Docker/setup_cyclonedds.sh $ROS_WS/cyclonedds/setup_cyclonedds.sh
RUN chmod +x $ROS_WS/cyclonedds/setup_cyclonedds.sh

ENV TEMPLATE_PATH="$ROS_WS/cyclonedds/cyclonedds.xml.template" \
    FINAL_XML="/etc/cyclonedds/cyclonedds.xml"

# 3) PERSISTI l’URI per il runtime
ENV CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds.xml"

# Entrypoint
COPY Docker/interface_entrypoint.sh /usr/local/bin/interface_entrypoint.sh
RUN chmod +x /usr/local/bin/interface_entrypoint.sh

ENV START_CMD="ros2 run unitree_ros2_interface interface_node"

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'source ${ROS_WS}/install/setup.bash' >> /root/.bashrc

WORKDIR $ROS_WS

ENTRYPOINT ["/usr/local/bin/interface_entrypoint.sh"]
