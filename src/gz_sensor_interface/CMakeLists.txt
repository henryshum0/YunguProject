cmake_minimum_required(VERSION 3.8)
project(gz_sensor_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(lidar_sensor src/lidar_sensor.cpp)
target_include_directories(lidar_sensor PRIVATE include)
ament_target_dependencies(lidar_sensor rclcpp sensor_msgs geometry_msgs tf2_ros builtin_interfaces)

add_executable(imu_bridge src/imu_bridge.cpp)
target_include_directories(imu_bridge PRIVATE include)
ament_target_dependencies(imu_bridge rclcpp sensor_msgs)

add_executable(truth_odom src/truth_odom.cpp)
target_include_directories(truth_odom PRIVATE include)
ament_target_dependencies(truth_odom rclcpp nav_msgs)

add_executable(super_lidar src/super_lidar.cpp)
target_include_directories(super_lidar PRIVATE include)
ament_target_dependencies(super_lidar rclcpp sensor_msgs nav_msgs px4_msgs)

install(TARGETS lidar_sensor imu_bridge truth_odom super_lidar
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
