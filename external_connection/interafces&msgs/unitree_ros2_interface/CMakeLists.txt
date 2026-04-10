cmake_minimum_required(VERSION 3.8)
project(unitree_ros2_interface)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces for Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetHighMode.srv"
  "srv/SetLedColor.srv"
  "srv/SetLedAnimation.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
and messa