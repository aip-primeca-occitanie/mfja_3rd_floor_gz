cmake_minimum_required(VERSION 3.28.0)
project(mfja_3rd_floor_description)

find_package(ament_cmake REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(mfja-symmetric-gripper-controller-system SHARED
  src/SymmetricGripperController.cc
)
target_compile_features(mfja-symmetric-gripper-controller-system PUBLIC cxx_std_17)
target_compile_options(mfja-symmetric-gripper-controller-system PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
target_link_libraries(mfja-symmetric-gripper-controller-system PRIVATE
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

install(
  TARGETS mfja-symmetric-gripper-controller-system
  LIBRARY DESTINATION lib
)

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
    test_industrial_gripper_articulation
    test/test_industrial_gripper_articulation.py
  )
endif()

ament_export_libraries(mfja-symmetric-gripper-controller-system)

ament_package()
