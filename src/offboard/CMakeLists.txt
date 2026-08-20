cmake_minimum_required(VERSION 3.8)
project(offboard)

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
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# --- Standalone executable ---
add_executable(offboard_node
  src/main.cpp
  src/offboard.cpp
  src/px4_handler.cpp
  src/super_handler.cpp
  src/visualization_handler.cpp
  src/waypoint_handler.cpp
)

target_include_directories(offboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_node
  rclcpp
  rclcpp_components
  px4_msgs
  mars_quadrotor_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
)

# --- Goal marker node (RViz 2D Goal Pose -> /goal_marker markers) ---
add_executable(goal_marker_node
  src/goal_marker_node.cpp
)

ament_target_dependencies(goal_marker_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

# --- Super bridge (gz lidar_link cloud -> world-frame /cloud_registered, PX4 odom) ---
add_executable(super_bridge
  src/super_bridge.cpp
)

target_include_directories(super_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(super_bridge
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
)

# --- Composable component ---
add_library(offboard_component SHARED
  src/offboard.cpp
  src/px4_handler.cpp
  src/super_handler.cpp
  src/visualization_handler.cpp
  src/waypoint_handler.cpp
)

target_include_directories(offboard_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(offboard_component
  rclcpp
  rclcpp_components
  px4_msgs
  mars_quadrotor_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
)

rclcpp_components_register_node(offboard_component
  PLUGIN "offboard::OffboardNode"
  EXECUTABLE offboard_component_node
)

# --- Install ---
install(TARGETS
  offboard_node
  goal_marker_node
  super_bridge
  offboard_component
  DESTINATION lib/${PROJECT_NAME}
)

# --- Python scripts (birdview overlay publisher) ---
install(PROGRAMS
  scripts/birdview_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components px4_msgs mars_quadrotor_msgs std_srvs)

ament_package()
