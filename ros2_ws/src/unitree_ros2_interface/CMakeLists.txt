cmake_minimum_required(VERSION 3.8)
project(unitree_ros2_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMPILE_FACE_LIGHTS on)                 # compile face lights interface 
set(COMPILE_CONTROL_INTERFACE on)           # compile control interface 
set(COMPILE_ULTRASOUND_INTERFACE on)        # compile ultrasound interface
set(COMPILE_CAMERA_INTERFACE on)            # compile camera interface republisher

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3 -Wno-reorder -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Control interface dependencies
if(COMPILE_CONTROL_INTERFACE)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(unitree_legged_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
endif()

# Camera interface dependencies
if(COMPILE_CAMERA_INTERFACE)
  find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc videoio calib3d imgcodecs)
  find_package(image_transport REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(camera_calibration_parsers REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(Eigen3 REQUIRED)
endif()

# Get the architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*")
  set(ARCH amd64)
  set(COMPILE_ULTRASOUND_INTERFACE off)  # Ultrasound interface not supported on amd64
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64.*")
  set(ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Generate interfaces for Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetHighMode.srv"
  "srv/SetLedColor.srv"
  "srv/SetLedAnimation.srv"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Common include directory
include_directories(include)

# Control interface includes
if(COMPILE_CONTROL_INTERFACE)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include)
endif()

# Camera interface includes
if(COMPILE_CAMERA_INTERFACE)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/UDPCameraInterface/include)
endif()

# Ultrasound interface includes
if(COMPILE_ULTRASOUND_INTERFACE)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/ultraSoundSDK/include)
endif()

# Face lights interface includes
if(COMPILE_FACE_LIGHTS)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/face_light_sdk/include)
endif()

if(COMPILE_CONTROL_INTERFACE)
  # Add executable for the low level interface node
  add_executable(low_interface_node 
    src/main-low-interface.cpp
    src/low-interface-node.cpp
  )
  
  # Add executable for the high level interface node
  add_executable(high_interface_node 
    src/main-high-interface.cpp
    src/high-interface-node.cpp
  )

  ament_target_dependencies(low_interface_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
    unitree_legged_msgs
  )
  
  ament_target_dependencies(high_interface_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
    unitree_legged_msgs
    nav_msgs
  )
  
  target_link_libraries(high_interface_node
    "${cpp_typesupport_target}"
  )
endif()

if(COMPILE_CAMERA_INTERFACE)
  # Add executable for the camera interface node
  add_executable(camera_interface_node 
    src/main-camera-interface.cpp
    src/camera-interface.cpp
  )
  
  # Add executable for the camera demo node
  add_executable(camera_demo_node 
    src/camera-demo.cpp
  )

  add_executable(camera_udp_node 
    src/main-udp-camera-interface.cpp
    src/udp-camera-interface.cpp
  )

  add_library(unitree_camera_interface_component SHARED
    src/camera-interface.cpp
  )

  add_library(unitree_udp_camera_interface_component SHARED
    src/udp-camera-interface.cpp
  )

  ament_target_dependencies(camera_interface_node
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    camera_calibration_parsers
    ament_index_cpp
  )
  
  ament_target_dependencies(camera_demo_node
    rclcpp
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    camera_calibration_parsers
    ament_index_cpp
  )

  ament_target_dependencies(camera_udp_node
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    camera_calibration_parsers
    camera_info_manager
    ament_index_cpp
  )

  ament_target_dependencies(unitree_camera_interface_component
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    camera_info_manager
    camera_calibration_parsers
    ament_index_cpp
    pcl_conversions
    Eigen3
  )

  ament_target_dependencies(unitree_udp_camera_interface_component
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    camera_info_manager
    camera_calibration_parsers
    ament_index_cpp
    pcl_conversions
    Eigen3
  ) 
endif()

if(COMPILE_FACE_LIGHTS)
  add_executable(face_lights_node
    src/main-face-lights.cpp
  )
    
  ament_target_dependencies(face_lights_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
  )
  
  target_link_libraries(face_lights_node
    "${cpp_typesupport_target}"
  )
endif()

if(COMPILE_ULTRASOUND_INTERFACE)
  add_executable(ultrasound_interface_node
    src/main-ultrasound-interface.cpp
  )
    
  ament_target_dependencies(ultrasound_interface_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
  )
endif()

# Determine board type based on IP address
execute_process(COMMAND hostname -I OUTPUT_VARIABLE IP_LIST OUTPUT_STRIP_TRAILING_WHITESPACE)
string(REGEX REPLACE " .*" "" FIRST_IP "${IP_LIST}")
message(STATUS "IP found: ${FIRST_IP}")
if(FIRST_IP MATCHES ".*\\.161$")
  set(BOARD "RasPi")
  add_definitions(-DRASPI)
else()
  set(BOARD "Nano")
  add_definitions(-DNANO)
endif()
message(STATUS "BOARD: ${BOARD}")

# Link with the unitree_legged_sdk library
if(COMPILE_CONTROL_INTERFACE)
  target_link_libraries(low_interface_node 
    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/${ARCH}/libunitree_legged_sdk.a
    pthread
  )
  
  target_link_libraries(high_interface_node 
    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/${ARCH}/libunitree_legged_sdk.a
    pthread
  )
endif()

if(COMPILE_CAMERA_INTERFACE)
  target_link_libraries(camera_interface_node 
    ${OpenCV_LIBRARIES}
    pthread
  )
  
  target_link_libraries(camera_demo_node 
    ${OpenCV_LIBRARIES}
    pthread
  )

  target_link_libraries(camera_udp_node 
    ${OpenCV_LIBRARIES}
    pthread
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/UDPCameraInterface/lib/libudp_camera_receiver.a
  )
  target_link_options(camera_udp_node PRIVATE -Wl,--copy-dt-needed-entries)

  target_include_directories(unitree_camera_interface_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(unitree_camera_interface_component
    pthread
    ${OpenCV_LIBRARIES}
  )

  target_include_directories(unitree_udp_camera_interface_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(unitree_udp_camera_interface_component
    pthread
    ${OpenCV_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/UDPCameraInterface/lib/libudp_camera_receiver.a
  )

endif()

if(COMPILE_FACE_LIGHTS)
  target_link_libraries(face_lights_node 
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/face_light_sdk/lib/libfaceLight_SDK_${ARCH}.so
    pthread
  )
endif()

if(COMPILE_ULTRASOUND_INTERFACE)
  target_link_libraries(ultrasound_interface_node 
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/ultraSoundSDK/lib/libultraSound_SDK_${BOARD}.so
    pthread
  )
endif()

# Set compile definitions and link options
if(COMPILE_CONTROL_INTERFACE)
  target_compile_definitions(low_interface_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
  target_link_options(low_interface_node PRIVATE -no-pie)
  
  target_compile_definitions(high_interface_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
  target_link_options(high_interface_node PRIVATE -no-pie)
  
  # Add include directories
  target_include_directories(low_interface_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include
  )
  
  target_include_directories(high_interface_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include
  )
endif()

if(COMPILE_CAMERA_INTERFACE)
  target_compile_definitions(camera_interface_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
  target_compile_definitions(camera_demo_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
  target_compile_definitions(camera_udp_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

if(COMPILE_FACE_LIGHTS)
  target_compile_definitions(face_lights_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

if(COMPILE_ULTRASOUND_INTERFACE)
  target_compile_definitions(ultrasound_interface_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

# Install the executables
if(COMPILE_CONTROL_INTERFACE)
  install(TARGETS low_interface_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS high_interface_node
    DESTINATION lib/${PROJECT_NAME}
  ) 
endif()

if(COMPILE_CAMERA_INTERFACE)
  install(TARGETS camera_interface_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS camera_demo_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS
    unitree_camera_interface_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  rclcpp_components_register_nodes(unitree_camera_interface_component
    "unitree_ros2_interface::UnitreeCameraInterface"
  )

  # UDP cameras
  install(TARGETS camera_udp_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS
    unitree_udp_camera_interface_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  rclcpp_components_register_nodes(unitree_udp_camera_interface_component
    "unitree_ros2_interface::UnitreeUdpCameraInterface"
  )
endif()

if(COMPILE_FACE_LIGHTS)
  install(TARGETS face_lights_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(COMPILE_ULTRASOUND_INTERFACE)
  install(TARGETS ultrasound_interface_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install resource directories
install(DIRECTORY
  config
  launch
  calibrations
  srv
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_runtime REQUIRED)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp_components)
ament_export_include_directories(include)
ament_export_libraries(unitree_camera_interface_component)
ament_export_libraries(unitree_udp_camera_interface_component)

ament_package()
