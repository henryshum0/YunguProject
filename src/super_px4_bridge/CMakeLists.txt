cmake_minimum_required(VERSION 3.8)
project(super_px4_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Bridge node (standalone executable) ---
add_executable(super_px4_bridge_node
  src/main.cpp
  src/super_px4_bridge.cpp
)

target_include_directories(super_px4_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(super_px4_bridge_node
  rclcpp
  px4_msgs
  mars_quadrotor_msgs
)

# --- Bridge node as component (for composable nodes) ---
add_library(super_px4_bridge_component SHARED
  src/super_px4_bridge.cpp
)

target_include_directories(super_px4_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(super_px4_bridge_component
  rclcpp
  rclcpp_components
  px4_msgs
  mars_quadrotor_msgs
)

rclcpp_components_register_node(super_px4_bridge_component
  PLUGIN "super_px4_bridge::SuperPx4Bridge"
  EXECUTABLE super_px4_bridge_component_node
)

# --- Install ---
install(TARGETS
  super_px4_bridge_node
  super_px4_bridge_component
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components px4_msgs mars_quadrotor_msgs)

ament_package()
