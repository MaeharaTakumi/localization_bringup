cmake_minimum_required(VERSION 3.8)
project(localization_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/robot_localization_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
