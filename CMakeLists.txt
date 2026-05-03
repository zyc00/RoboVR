cmake_minimum_required(VERSION 3.20)

project(RoboVR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(quest3_teleop_host
  host_bridge/src/tcp_server.cpp
)
target_include_directories(quest3_teleop_host
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/host_bridge/include
    ${CMAKE_CURRENT_SOURCE_DIR}/proto
)
target_compile_options(quest3_teleop_host PRIVATE -Wall -Wextra -Wpedantic)

add_executable(host_bridge_demo host_bridge/tools/host_bridge_demo.cpp)
target_link_libraries(host_bridge_demo PRIVATE quest3_teleop_host)
