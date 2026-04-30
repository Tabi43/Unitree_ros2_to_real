cmake_minimum_required(VERSION 3.8)
project(unitree_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(unitree_legged_sdk REQUIRED)

add_executable(stand_and_sit_node src/stand_and_sit_node.cpp)
ament_target_dependencies(stand_and_sit_node rclcpp unitree_legged_msgs)

add_executable(stand_and_sit_sdk_node src/stand_and_sit_sdk_node.cpp)
ament_target_dependencies(stand_and_sit_sdk_node rclcpp)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64.*")
  set(UNITREE_SDK_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64.*")
  set(UNITREE_SDK_ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture for SDK: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(UNITREE_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include")
set(UNITREE_SDK_STATIC_LIB
  "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/${UNITREE_SDK_ARCH}/libunitree_legged_sdk.a")

if(NOT EXISTS "${UNITREE_SDK_STATIC_LIB}")
  message(FATAL_ERROR "SDK static library not found: ${UNITREE_SDK_STATIC_LIB}")
endif()

target_include_directories(stand_and_sit_sdk_node PRIVATE "${UNITREE_SDK_INCLUDE_DIR}")
target_link_libraries(stand_and_sit_sdk_node "${UNITREE_SDK_STATIC_LIB}" pthread)
target_link_options(stand_and_sit_sdk_node PRIVATE -no-pie)
target_compile_definitions(stand_and_sit_sdk_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

install(
  TARGETS stand_and_sit_node stand_and_sit_sdk_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
