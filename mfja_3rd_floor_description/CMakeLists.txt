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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_room315_vla_camera_model
    test/test_room315_vla_camera_model.py
  )
endif()

ament_package()
