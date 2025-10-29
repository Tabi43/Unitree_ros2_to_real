cmake_minimum_required(VERSION 3.8)
project(unitree_ros2_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

include_directories(
  include
)

# Add executable for the interface node
add_executable(interface_node 
  src/main-interface.cpp
  src/Interface-node.cpp
)

# Set dependencies for the executable
ament_target_dependencies(interface_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  unitree_legged_msgs
)

# Get the architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*")
  set(ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64.*")
  set(ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Link with the unitree_legged_sdk library
target_link_libraries(interface_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  pthread
)

# Set compile definitions and link options
target_compile_definitions(interface_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
target_link_options(interface_node PRIVATE -no-pie)

# Add include directories
target_include_directories(interface_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include
)

# Install the executable
install(TARGETS interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
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

ament_package()
