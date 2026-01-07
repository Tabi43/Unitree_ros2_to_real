# Unitree ROS2 Interface

A comprehensive ROS2 interface package for controlling Unitree quadruped robots Go1 and managing their sensors systems. This package provides high-level and low-level control interfaces, stereo camera streaming, depth sensing, point cloud generation, RGB face leds control and ultrasound sensing.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Hardware Requirements](#hardware-requirements)
- [Package Components](#package-components)
- [Usage](#usage)
  - [Basic Robot Control](#basic-robot-control)
  - [Camera Interface](#camera-interface)
  - [Configuration](#configuration)
- [Launch Files](#launch-files)
- [ROS2 Topics and Services](#ros2-topics-and-services)
- [Parameters](#parameters)
- [Calibration](#calibration)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

The Unitree ROS2 Interface package enables seamless communication between ROS2 systems and Unitree quadruped robots. It provides:

- **Robot Control**: High-level and low-level motor control interfaces
- **Sensor Integration**: IMU, joint state, and odometry publishing
- **Vision System**: Multi-camera stereo vision with depth sensing
- **Point Cloud Generation**: 3D environmental mapping capabilities
- **Safety Features**: Command timeouts and emergency stop functionality

## Features

### Robot Control Features
- ✅ High-level control (velocity commands, gait control)
- ✅ Low-level control (direct motor commands)
- ✅ Safety timeouts for velocity commands
- ✅ Multiple robot modes (idle, standing, walking, recovery)
- ✅ IMU data publishing
- ✅ Joint state monitoring
- ✅ Odometry estimation

### Camera System Features
- ✅ Multi-camera support (5 cameras: front, chin, left, right, bottom)
- ✅ Stereo vision processing
- ✅ Depth map generation
- ✅ Point cloud generation with filtering
- ✅ Camera calibration support
- ✅ Image rectification
- ✅ ROS2 camera_info publishing

### Additional Features
- ✅ Modular compilation flags
- ✅ Distributed processing support
- ✅ Docker container support
- ✅ URDF robot description

## Installation

### Prerequisites

Ensure you have ROS2 Humble (or later) installed on your system.

```bash
# Install required system dependencies
sudo apt update
sudo apt install -y \
    libopencv-dev \
    libpcl-dev \
    libeigen3-dev \
    libudev-dev
```

### Build Instructions

1. **Clone the repository** (if not already done):
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> unitree_ros2_to_real
   ```

2. **Install ROS2 dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select unitree_ros2_interface
   ```

4. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Hardware Requirements

### Supported Robots
- Unitree Go1

### Network Configuration
- Robot should be connected via Ethernet or WiFi
- Default communication uses UDP on the robot's network interface
- Ensure proper network routing between host and robot

## Package Components

The package consists of several modular components that can be compiled independently:

### Control Interfaces
- **High-Level Interface**: Velocity-based control, gait selection
- **Low-Level Interface**: Direct motor torque/position control

### Camera Interface  
- **Stereo Camera Processing**: Multi-camera stereo vision
- **Depth Generation**: Real-time depth map computation
- **Point Cloud**: 3D environmental mapping

### Additional Modules
- **Face Light SDK**: LED control interface
- **Ultrasound Interface**: Sensor integration

## Usage

### Basic Robot Control

#### 1. High-Level Control (Recommended for most users)

**Start the high-level interface:**
```bash
ros2 run unitree_ros2_interface high_interface_node
```

**Control the robot with velocity commands:**
```bash
# Move forward at 0.3 m/s
ros2 topic pub /unitree_go1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Turn in place (0.5 rad/s)
ros2 topic pub /unitree_go1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

**Change robot mode:**
```bash
# Set to velocity control mode
ros2 service call /unitree_go1/set_high_mode unitree_ros2_interface/srv/SetHighMode "{mode: 2}"

# Available modes:
# 0: IDLE_MODE
# 1: FREE_STAND_MODE  
# 2: VELOCITY_MODE
# 5: STAND_DOWN_MODE
# 6: STAND_UP_MODE
# 7: DAMPING_MODE
# 8: RECOVERY_MODE
```

#### 2. Low-Level Control (Advanced users)

**Start the low-level interface:**
```bash
ros2 run unitree_ros2_interface low_interface_node
```

**Monitor joint states:**
```bash
ros2 topic echo /unitree_go1/joint_states
```

**Send low-level commands:**
```bash
# This requires publishing to unitree_legged_msgs/msg/LowCmd
# See the low-level interface documentation for detailed motor control
```

### Camera Interface

#### 1. Single Camera Launch

**Launch front camera:**
```bash
ros2 launch unitree_ros2_interface camera_base.launch.py \
  camera_name:=front_camera \
  publish_rectified:=true \
  publish_depth:=true \
  publish_pcl:=true
```

**Available cameras:**
- `front_camera`
- `chin_camera`  
- `left_camera`
- `right_camera`
- `bottom_camera`

#### 2. View Camera Streams

**View raw images:**
```bash
ros2 run rqt_image_view rqt_image_view /unitree_go1/front_camera/left/image_raw
```

**View depth images:**
```bash
ros2 run rqt_image_view rqt_image_view /unitree_go1/front_camera/depth/image
```

**Visualize point clouds:**
```bash
ros2 run rviz2 rviz2
# Add PointCloud2 display and set topic to /unitree_go1/front_camera/point_cloud
```

#### 3. Multi-Camera Setup

**Launch multiple cameras using board-specific launch files:**

```bash
# Head board (front + chin cameras)
ros2 launch unitree_ros2_interface head_board.launch.py \
  enable_camera:=true \
  publish_depth:=true \
  publish_pcl:=true

# Body board (left + right cameras)  
ros2 launch unitree_ros2_interface body_board.launch.py \
  enable_camera:=true

# Main board (bottom camera)
ros2 launch unitree_ros2_interface main_board.launch.py \
  enable_camera:=true
```

### Configuration

#### Robot Parameters

Edit `config/unitree_go1_params.yaml`:
```yaml
unitree_go1:
  imu_frequency: 1000             # IMU frequency [Hz]
  joint_state_frequency: 500      # Joint states frequency [Hz]  
  remote_frequency: 100           # Remote frequency [Hz]
```

#### Camera Configuration

Each camera has its own configuration file in `config/`:
- `stereo_front_camera_config.yaml`
- `stereo_chin_camera_config.yaml`
- `stereo_left_camera_config.yaml`
- `stereo_right_camera_config.yaml`
- `stereo_bottom_camera_config.yaml`

Example configuration:
```yaml
%YAML:1.0
---
LogLevel: !!opencv-matrix
   rows: 1
   cols: 1
   dt: d
   data: [ 1. ]
Threshold: !!opencv-matrix
   rows: 1
   cols: 1
   dt: d
   data: [ 190. ]
IpLastSegment: !!opencv-matrix
   rows: 1
   cols: 1
   dt: d
   data: [ 13. ]
```

## Launch Files

The package provides several launch files for different deployment scenarios:

### Core Launch Files

| Launch File | Description | Use Case |
|------------|-------------|----------|
| `interface.launch.py` | Main interface with all components | Complete system launch |
| `camera_base.launch.py` | Single camera interface | Individual camera testing |
| `head_board.launch.py` | Head board cameras | Distributed setup |
| `body_board.launch.py` | Body cameras | Distributed setup |
| `main_board.launch.py` | Main board camera | Distributed setup |
| `pi_board.launch.py` | Raspberry Pi board | Edge computing |

### Launch Arguments

**Common arguments for all launch files:**
```bash
ros2 launch unitree_ros2_interface <launch_file> \
  board_ip:=192.168.123.10 \
  board_role:=head \
  enable_camera:=true \
  enable_ultrasound:=false \
  enable_face_lights:=false \
  enable_low:=true \
  enable_high:=false \
  publish_rectified:=true \
  publish_depth:=false \
  publish_pcl:=true
```

## ROS2 Topics and Services

### Published Topics

#### Robot Control Topics
| Topic | Type | Description | Frequency |
|-------|------|-------------|-----------|
| `/unitree_go1/joint_states` | `sensor_msgs/JointState` | Joint positions and velocities | 500 Hz |
| `/unitree_go1/imu` | `sensor_msgs/Imu` | IMU orientation and acceleration | 1000 Hz |
| `/unitree_go1/odom` | `nav_msgs/Odometry` | Robot odometry | 500 Hz |
| `/unitree_go1/high_state` | `unitree_legged_msgs/HighState` | Robot high-level state | 50 Hz |
| `/unitree_go1/low_state` | `unitree_legged_msgs/LowState` | Robot low-level state | 50 Hz |

#### Camera Topics (per camera)
| Topic Pattern | Type | Description |
|---------------|------|-------------|
| `/unitree_go1/{camera}/left/image_raw` | `sensor_msgs/Image` | Left camera raw image |
| `/unitree_go1/{camera}/right/image_raw` | `sensor_msgs/Image` | Right camera raw image |
| `/unitree_go1/{camera}/left/image_rect` | `sensor_msgs/Image` | Left rectified image |
| `/unitree_go1/{camera}/right/image_rect` | `sensor_msgs/Image` | Right rectified image |
| `/unitree_go1/{camera}/depth/image` | `sensor_msgs/Image` | Depth image |
| `/unitree_go1/{camera}/point_cloud` | `sensor_msgs/PointCloud2` | Point cloud |
| `/unitree_go1/{camera}/left/camera_info` | `sensor_msgs/CameraInfo` | Left camera parameters |
| `/unitree_go1/{camera}/right/camera_info` | `sensor_msgs/CameraInfo` | Right camera parameters |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/unitree_go1/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/unitree_go1/high_cmd` | `unitree_legged_msgs/HighCmd` | High-level commands |
| `/unitree_go1/low_cmd` | `unitree_legged_msgs/LowCmd` | Low-level motor commands |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/unitree_go1/set_high_mode` | `unitree_ros2_interface/SetHighMode` | Change robot control mode |

## Parameters

### High-Level Interface Parameters

- `cmd_vel_timeout` (double, default: 0.5): Timeout for cmd_vel messages in seconds
- `prefix` (string, default: "/unitree_go1"): Robot namespace prefix

### Camera Interface Parameters

- `camera_name` (string): Camera identifier (front_camera, chin_camera, etc.)
- `publish_rectified` (bool): Enable rectified image publishing
- `publish_depth` (bool): Enable depth image publishing  
- `publish_pcl` (bool): Enable point cloud publishing
- `frame_rate` (int, default: 30): Camera frame rate
- `exposure_time` (int): Camera exposure time

### Compilation Options

The package supports selective compilation via CMake options in `CMakeLists.txt`:

```cmake
set(COMPILE_FACE_LIGHTS on)           # Face light control
set(COMPILE_CONTROL_INTERFACE on)     # Robot control interface
set(COMPILE_ULTRASOUND_INTERFACE on)  # Ultrasound sensors
set(COMPILE_CAMERA_INTERFACE on)      # Camera interface
```

## Calibration

### Camera Calibration

The package includes pre-calibrated parameters for each camera in the `calibrations/` directory:

```
calibrations/
├── bottom_camera_left.yaml    # Bottom camera left lens
├── bottom_camera_right.yaml   # Bottom camera right lens
├── chin_camera_left.yaml      # Chin camera left lens
├── chin_camera_right.yaml     # Chin camera right lens
├── front_camera_left.yaml     # Front camera left lens
├── front_camera_right.yaml    # Front camera right lens
├── left_camera_left.yaml      # Left side camera left lens
├── left_camera_right.yaml     # Left side camera right lens
├── right_camera_left.yaml     # Right side camera left lens
└── right_camera_right.yaml    # Right side camera right lens
```

### Re-calibration Process

If you need to recalibrate cameras:

1. **Collect calibration data:**
   ```bash
   ros2 run camera_calibration cameracalibrator.py \
     --size 8x6 --square 0.108 \
     image:=/unitree_go1/front_camera/left/image_raw \
     camera:=/unitree_go1/front_camera/left
   ```

2. **Update calibration files** in the `calibrations/` directory

3. **Restart camera nodes** to load new parameters

### Building for Development

**Debug build:**
```bash
colcon build --packages-select unitree_ros2_interface --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**Enable specific interfaces:**
```bash
colcon build --packages-select unitree_ros2_interface --cmake-args \
  -DCOMPILE_FACE_LIGHTS=OFF \
  -DCOMPILE_ULTRASOUND_INTERFACE=OFF
```

### Adding New Features

1. **For robot control**: Modify `high-interface-node.cpp` or `low-interface-node.cpp`
2. **For camera features**: Extend `camera-interface.cpp`
3. **For new sensors**: Add to libs/ directory and update CMakeLists.txt

## Troubleshooting

### Common Issues

#### 1. Robot Not Responding
- **Check network connection**: Ensure robot and host are on same network
- **Verify IP addresses**: Check robot's network configuration
- **Check permissions**: Some interfaces may require root privileges

```bash
# Test network connectivity
ping <robot_ip>

# Check for UDP communication
sudo netstat -tulpn | grep :8080
```

#### 2. Camera Issues
- **Camera not detected**: Check USB connections and device permissions
- **Poor image quality**: Verify lighting conditions and camera settings
- **High latency**: Reduce image resolution or frame rate

```bash
# Check camera devices
ls /dev/video*

# Test camera access
v4l2-ctl --list-devices
```

#### 3. Build Errors
- **Missing dependencies**: Ensure all ROS2 packages are installed
- **SDK issues**: Check unitree_legged_sdk compatibility
- **Architecture mismatch**: Verify correct library architecture (amd64/arm64)

```bash
# Check ROS2 dependencies
rosdep check --from-paths src --ignore-src

# Verify SDK libraries
file src/unitree_legged_sdk/lib/cpp/amd64/libunitree_legged_sdk.a
```

### Performance Optimization

#### 1. Real-time Performance
```bash
# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Set real-time priorities (optional)
sudo chrt -f 80 ros2 run unitree_ros2_interface high_interface_node
```

#### 2. Memory Usage
- Disable unused camera streams
- Reduce point cloud density
- Limit concurrent camera operations

#### 3. Network Optimization  
```bash
# Increase UDP buffer sizes
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728
```

### Logging and Debugging

**Enable debug logging:**
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 run unitree_ros2_interface high_interface_node
```

**Monitor topics:**
```bash
# List active topics
ros2 topic list

# Check topic frequency
ros2 topic hz /unitree_go1/joint_states

# Monitor system resources  
htop
```

## Contributing

We welcome contributions to improve the Unitree ROS2 Interface package!

### Guidelines

1. **Fork the repository** and create a feature branch
2. **Follow coding standards**: Use consistent formatting and documentation
3. **Test thoroughly**: Ensure new features don't break existing functionality
4. **Update documentation**: Add/modify README sections as needed
5. **Submit pull request**: Provide clear description of changes

### Development Setup

```bash
# Install pre-commit hooks (optional but recommended)
pip install pre-commit
pre-commit install

# Run tests before submitting
colcon test --packages-select unitree_ros2_interface
```

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

---

## Support and Contact

- **Maintainer**: Marco Tabita (marco.tabita@edu.unige.it)
- **Issues**: Please use GitHub Issues for bug reports and feature requests
- **Documentation**: This README and inline code documentation
- **Community**: ROS2 community forums and Unitree developer communities

For urgent issues or specific support questions, please contact the maintainer directly.

---

**Last Updated**: January 2026  
**ROS2 Version**: Humble and later  
**Tested Robots**: Not Yet :)