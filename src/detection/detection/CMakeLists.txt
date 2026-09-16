cmake_minimum_required(VERSION 3.8)
project(detection)

# Humble's apt-installed Python modules and rclpy use Ubuntu's system interpreter.
# Force it before ament loads FindPython so stale venv/Conda cache entries cannot win.
set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
  "ROS 2 Humble system Python interpreter" FORCE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs the handwritten package tree and registers its PYTHONPATH hook, so
# `import detection` works from any node or script after sourcing the overlay.
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS
  scripts/mock_detector_node
  scripts/target_localizer_node
  scripts/detection_overlay_node
  scripts/spawn_targets
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(detection_tests tests/unit
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src
    TIMEOUT 180
  )
endif()

ament_package()
