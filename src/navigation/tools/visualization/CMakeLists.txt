cmake_minimum_required(VERSION 3.8)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(visual_tf src/visual_tf.cpp)
ament_target_dependencies(visual_tf rclcpp nav_msgs geometry_msgs tf2_ros)

install(TARGETS visual_tf DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gt_path.py scripts/fastlio_visual.py scripts/birdview_publisher.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()
