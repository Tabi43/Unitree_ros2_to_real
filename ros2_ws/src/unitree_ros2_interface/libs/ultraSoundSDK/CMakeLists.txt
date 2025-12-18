cmake_minimum_required(VERSION 3.10)
project(ultraSound)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include_directories(include)
link_directories(lib)

add_executable(example_UltraSoundGroup example/example_UltraSoundGroup_xx.cpp)
target_link_libraries(example_UltraSoundGroup libultraSound_SDK_xx.so)

add_executable(example_UltraSound example/example_UltraSound_xx.cpp)
target_link_libraries(example_UltraSound libultraSound_SDK_xx.so)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
