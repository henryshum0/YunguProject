cmake_minimum_required(VERSION 3.8)
project(lidar_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Shared monotonic-timestamp helper (header only).
set(lidar_bridge_deps
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# add_time_field: zero 'time' field + monotonicize the scan stamp.
add_executable(add_time_field src/add_time_field.cpp)
target_include_directories(add_time_field PRIVATE include)
ament_target_dependencies(add_time_field ${lidar_bridge_deps})

# lidar_merge: fuse left/right LiDAR clouds into one base_link cloud.
add_executable(lidar_merge src/lidar_merge.cpp)
target_include_directories(lidar_merge PRIVATE include)
ament_target_dependencies(lidar_merge ${lidar_bridge_deps})

# imu_relay: monotonicize gz IMU timestamps.
add_executable(imu_relay src/imu_relay.cpp)
target_include_directories(imu_relay PRIVATE include)
ament_target_dependencies(imu_relay ${lidar_bridge_deps})

# tf_bridge: world->base_link (dynamic) + base_link->lidar_link (static) TF.
add_executable(tf_bridge src/tf_bridge.cpp)
target_include_directories(tf_bridge PRIVATE include)
ament_target_dependencies(tf_bridge ${lidar_bridge_deps})

install(TARGETS
  add_time_field
  lidar_merge
  imu_relay
  tf_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
