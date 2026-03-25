cmake_minimum_required(VERSION 3.28.0)
project(mfja_robot_control_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS scripts/multi_robot_sync_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
