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

# --- Standalone executable ---
add_executable(offboard_node
  src/main.cpp
  src/offboard_node.cpp
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

# --- Composable component ---
add_library(offboard_component SHARED
  src/offboard_node.cpp
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
  offboard_component
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
