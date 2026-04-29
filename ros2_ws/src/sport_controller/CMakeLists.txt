cmake_minimum_required(VERSION 3.8)
project(sport_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

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

set(CMAKE_CXX_STANDARD 11)
find_package(Boost REQUIRED)

# ROS 2 and Gazebo setup
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(unitree_ros2_interface REQUIRED)

include_directories(
    include
    ${Boost_INCLUDEDIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Executable compilation
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(sport_controller src/main.cpp ${SRC_LIST})

ament_target_dependencies(sport_controller 
    rclcpp 
    std_msgs 
    tf2 
    tf2_ros 
    nav_msgs 
    sensor_msgs 
    geometry_msgs 
    unitree_legged_msgs 
    unitree_ros2_interface
    Boost
)

target_link_libraries(sport_controller -pthread)

install(TARGETS sport_controller DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  FILES_MATCHING PATTERN "*.py" PATTERN "*.xml" PATTERN "*.yaml"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml" PATTERN "*.xml"
)

ament_package()
