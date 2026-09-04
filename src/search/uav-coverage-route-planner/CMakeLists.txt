cmake_minimum_required(VERSION 3.8)
project(coverage_planner)

# Humble's apt-installed Python modules and rclpy use Ubuntu's system interpreter.
# Force it before ament loads FindPython so stale venv/Conda cache entries cannot win.
set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
  "ROS 2 Humble system Python interpreter" FORCE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanCoverage.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# rosidl_generator_py creates its own coverage_planner Python package for generated service
# modules. Install the handwritten package tree directly so both trees can share that namespace.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/coverage_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY docs launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(coverage_planner_tests tests/unit
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    TIMEOUT 180
  )
  add_launch_test(tests/launch/test_coverage_planner_launch.py
    TARGET coverage_planner_launch_test
    TIMEOUT 90
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
