cmake_minimum_required(VERSION 3.8)
project(offboard_fsm)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(super_planner REQUIRED)
find_package(fast_lio REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueueWaypoints.srv"
  "srv/ClearWaypoints.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# State-machine state handlers (each state lives in its own file).
set(OFFBOARD_STATE_SOURCES
  offboard_fsm/states/handle_init.cpp
  offboard_fsm/states/handle_arming.cpp
  offboard_fsm/states/handle_takeoff.cpp
  offboard_fsm/states/handle_idle.cpp
  offboard_fsm/states/handle_move.cpp
  offboard_fsm/states/handle_land.cpp
)

# --- Standalone executable ---
add_executable(offboard_node
  offboard_fsm/main.cpp
  offboard_fsm/offboard_fsm.cpp
  offboard_fsm/px4_handler.cpp
  offboard_fsm/super_handler.cpp
  offboard_fsm/waypoint_handler.cpp
  ${OFFBOARD_STATE_SOURCES}
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
  std_msgs
  geometry_msgs
  visualization_msgs
  super_planner
  fast_lio
)
target_link_libraries(offboard_node "${cpp_typesupport_target}")

# --- FAST-LIO -> PX4 visual odometry bridge (C++ replacement for the legacy
#     scripts/fastlio_px4_bridge.py) ---
add_executable(fastlio_handler
  offboard_fsm/fastlio_handler_main.cpp
  offboard_fsm/fastlio_handler.cpp
)

target_include_directories(fastlio_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fastlio_handler
  rclcpp
  px4_msgs
  nav_msgs
)

# --- Goal marker node (RViz 2D Goal Pose -> /goal_marker markers) ---
add_executable(goal_marker_node
  offboard_fsm/goal_marker_node.cpp
)

ament_target_dependencies(goal_marker_node
  rclcpp
  geometry_msgs
  visualization_msgs
)
target_link_libraries(goal_marker_node "${cpp_typesupport_target}")

# (super_bridge moved to gz_sensor_interface/super_lidar -> /gz/point_cloud_super)

# --- Composable component ---
add_library(offboard_component SHARED
  offboard_fsm/offboard_fsm.cpp
  offboard_fsm/px4_handler.cpp
  offboard_fsm/super_handler.cpp
  offboard_fsm/waypoint_handler.cpp
  ${OFFBOARD_STATE_SOURCES}
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
  std_msgs
  geometry_msgs
  visualization_msgs
  super_planner
  fast_lio
)
target_link_libraries(offboard_component "${cpp_typesupport_target}")

rclcpp_components_register_node(offboard_component
  PLUGIN "offboard::OffboardNode"
  EXECUTABLE offboard_component_node
)

# --- Install ---
install(TARGETS
  offboard_node
  goal_marker_node
  fastlio_handler
  offboard_component
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components px4_msgs mars_quadrotor_msgs std_srvs
  rosidl_default_runtime)

ament_package()
