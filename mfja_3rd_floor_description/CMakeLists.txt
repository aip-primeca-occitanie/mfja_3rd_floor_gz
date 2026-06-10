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
  ament_add_pytest_test(
    test_industrial_robot_collision_free
    test/test_industrial_robot_collision_free.py
  )
  ament_add_pytest_test(
    test_lightweight_world_physics
    test/test_lightweight_world_physics.py
  )
  ament_add_pytest_test(
    test_kuka_initial_joint_positions
    test/test_kuka_initial_joint_positions.py
  )
  ament_add_pytest_test(
    test_room315_shuttle_collision_free
    test/test_room315_shuttle_collision_free.py
  )
endif()

ament_package()
