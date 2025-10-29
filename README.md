# Unitree ROS2 to Real Robot Interface

A ROS2 interface for real-time communication with Unitree quadruped robots (Go1, A1, etc.) using the official Unitree Legged SDK. This project provides a containerized solution for robust robot control and monitoring through standard ROS2 topics and services.

## 🚀 Overview

This interface bridges the gap between ROS2 applications and Unitree robots by:
- **Real-time UDP Communication**: Direct low-level communication with robot hardware
- **ROS2 Integration**: Standard ROS2 topics for joint states, IMU data, and remote control
- **Docker Architecture**: Multi-architecture containerized deployment (AMD64/ARM64)
- **Safety Features**: Built-in safety mechanisms and emergency stop capabilities
- **Cross-platform**: Works on both x86_64 and ARM64 architectures

## 📋 Features

### Core Functionality
- **Low-level Robot Control**: Direct motor control via UDP communication
- **Sensor Data Publishing**: Real-time joint states, IMU data, and wireless remote state
- **Safety Service**: Enable/disable interface with safety checks
- **Multi-architecture Support**: Native builds for both AMD64 and ARM64 platforms

### ROS2 Topics & Services

#### Published Topics
- `/joint_state` (`sensor_msgs/JointState`) - Joint positions, velocities, and efforts
- `/imu` (`sensor_msgs/Imu`) - IMU orientation and angular velocity data
- `/remote` (`unitree_legged_msgs/WirelessRemote`) - Wireless remote controller state

#### Subscribed Topics
- `/low_cmd` (`unitree_legged_msgs/LowCmd`) - Low-level motor commands

#### Services
- `/enable_unitree_interface` (`std_srvs/SetBool`) - Enable/disable robot control interface

## 🏗️ Docker Architecture

The project uses a sophisticated multi-architecture Docker setup optimized for both development and production environments.

### Architecture Components

```
unitree_ros2_to_real/
├── Docker/
│   ├── amd/Dockerfile          # AMD64 architecture build
│   ├── arm/Dockerfile          # ARM64 architecture build
│   └── start_interface.sh      # Container entry point script
├── ros2_ws/                    # ROS2 workspace
│   └── src/
│       ├── unitree_ros2_interface/    # Main interface package
│       ├── unitree_legged_msgs/       # Unitree message definitions
│       └── unitree_legged_sdk/        # Unitree SDK libraries
├── create-ros-image.sh         # Automated image building script
└── start-container.sh          # Container deployment script
```

### Docker Build Process

#### 1. **Multi-Architecture Support**
The system automatically detects the host architecture and builds the appropriate image:
- **AMD64**: Uses `Docker/amd/Dockerfile` for x86_64 systems
- **ARM64**: Uses `Docker/arm/Dockerfile` for ARM64/AArch64 systems

#### 2. **Base Image Strategy**
```dockerfile
FROM ros:humble-ros-base-jammy
```
- Lightweight ROS2 Humble base image
- Ubuntu 22.04 LTS foundation
- Minimal footprint for production deployment

#### 3. **Dependency Management**
The Dockerfiles install essential dependencies:
```dockerfile
# Build tools and libraries
build-essential cmake git
python3-colcon-common-extensions python3-rosdep
libboost-all-dev

# ROS2 packages
ros-humble-rclcpp
ros-humble-sensor-msgs
ros-humble-std-msgs
ros-humble-geometry-msgs

# Network and system tools
iproute2 iputils-ping net-tools
libcap2-bin
```

#### 4. **Real-time Capabilities**
The container is configured for real-time performance:
```dockerfile
# Grant CAP_SYS_NICE for real-time thread scheduling
RUN find $ROS_WS/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true
```

### Container Deployment Strategy

#### **Automated Image Building** (`create-ros-image.sh`)
- **Architecture Detection**: Automatically identifies host architecture
- **Platform-specific Building**: Uses `docker buildx` for cross-platform builds
- **Conditional Rebuilding**: Optimizes build time by checking existing images

#### **Container Management** (`start-container.sh`)
- **Auto-restart Policy**: Containers automatically restart on system boot
- **Network Configuration**: Uses host networking for optimal ROS2/DDS performance
- **Resource Management**: Configures real-time priorities and memory limits
- **Device Access**: Mounts `/dev` for hardware device access

```bash
# Container runtime configuration
docker run \
  --name udp_ros2_if \
  --detach \
  --restart unless-stopped \
  --network host \
  --ipc host \
  --cap-add SYS_NICE \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  -v /dev:/dev \
  udp_ros2_if:humble-amd64
```

## 🔧 Installation & Usage

### Prerequisites
- Docker Engine with buildx support
- Network access to Unitree robot (default: `192.168.123.10:8007`)

### Quick Start

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd unitree_ros2_to_real
   ```

2. **Build the Docker image**:
   ```bash
   ./create-ros-image.sh
   ```

3. **Start the interface container**:
   ```bash
   ./start-container.sh
   ```

4. **Enable the interface** (from host or another container):
   ```bash
   ros2 service call /enable_unitree_interface std_srvs/srv/SetBool "{data: true}"
   ```

### Custom Configuration

#### Environment Variables
- `ROS_DISTRO`: ROS2 distribution (default: `humble`)
- `IMAGE_BASE`: Docker image base name (default: `udp_ros2_if`)
- `CONTAINER_NAME`: Container name (default: `udp_ros2_if`)
- `START_CMD`: Custom startup command

#### Network Configuration
The interface communicates with Unitree robots using these default settings:
- **Robot IP**: `192.168.123.10`
- **Robot Port**: `8007`
- **Interface Port**: `8091`

## 🔒 Safety Features

### Built-in Safety Mechanisms
- **Emergency Stop Service**: Immediate disable via ROS2 service
- **Safety State Management**: Automatic safety state enforcement
- **Signal Handling**: Graceful shutdown on SIGINT/SIGTERM
- **UDP Timeout Protection**: Automatic disconnection on communication loss

### Usage Example
```bash
# Enable robot control
ros2 service call /enable_unitree_interface std_srvs/srv/SetBool "{data: true}"

# Emergency stop
ros2 service call /enable_unitree_interface std_srvs/srv/SetBool "{data: false}"
```

## 🛠️ Development

### Building from Source (without Docker)

1. **Setup ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Install dependencies**:
   ```bash
   cd ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **Source and run**:
   ```bash
   source install/setup.bash
   ros2 run unitree_ros2_interface interface_node
   ```

### Architecture Details

#### **Thread Management**
The interface uses Unitree's LoopFunc for real-time communication:
- **UDP Send Loop**: Transmits motor commands at configurable frequency
- **UDP Receive Loop**: Receives robot state at configurable frequency
- **ROS2 Spin**: Handles ROS2 callbacks and service requests

#### **Memory Management**
- **Lock-free Communication**: Uses atomic operations for thread-safe data exchange
- **Aligned Memory**: 64-byte aligned buffers for optimal performance
- **Minimal Allocations**: Pre-allocated buffers to avoid runtime allocations

## 📊 Performance Considerations

### Real-time Performance
- **UDP Communication**: Direct UDP sockets for minimal latency
- **Thread Priorities**: Real-time thread scheduling with `CAP_SYS_NICE`
- **Memory Locking**: Prevents memory swapping with `memlock` limits
- **CPU Affinity**: Configurable CPU core binding for consistent performance

### Network Optimization
- **Host Networking**: Uses Docker host networking for optimal DDS performance
- **IPC Sharing**: Shared IPC namespace for efficient inter-process communication
- **Buffer Management**: Optimized buffer sizes for UDP communication

## 🔍 Troubleshooting

### Common Issues

1. **Permission Denied Errors**:
   ```bash
   # Ensure proper capabilities
   sudo setcap cap_sys_nice+ep /path/to/interface_node
   ```

2. **Network Connectivity**:
   ```bash
   # Test robot connectivity
   ping 192.168.123.10
   ```

3. **Container Issues**:
   ```bash
   # Check container logs
   docker logs udp_ros2_if
   
   # Restart container
   docker restart udp_ros2_if
   ```

## 📝 License

This project is licensed under the terms specified in the package.xml file.

## 🤝 Contributing

Contributions are welcome! Please ensure all changes are tested with both AMD64 and ARM64 architectures.

## 📧 Contact

Maintainer: tabi43 (marco.tabita@edu.unige.it)

---

**Note**: This interface requires physical access to a Unitree robot and proper network configuration. Always follow safety protocols when working with robotic hardware.
