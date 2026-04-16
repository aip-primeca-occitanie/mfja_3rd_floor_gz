cmake_minimum_required(VERSION 3.28.0)
project(mfja_3rd_floor_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES THIRD_PARTY.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
