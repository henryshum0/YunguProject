cmake_minimum_required(VERSION 3.8)
project(offboard_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueueWaypoints.srv"
  "srv/ClearWaypoints.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

add_executable(offboard_node src/offboard_node.cpp)
target_include_directories(offboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(offboard_node
  geometry_msgs nav_msgs px4_msgs quadrotor_msgs rclcpp std_msgs std_srvs)
target_link_libraries(offboard_node "${cpp_typesupport_target}")

install(TARGETS offboard_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
