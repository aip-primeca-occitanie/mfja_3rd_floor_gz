cmake_minimum_required(VERSION 3.28.0)
project(mfja_robot_control_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES
    config/gazebo_params.yaml
    config/gripper_command_defaults.yaml
    config/mfja_default.gui.config
    config/robots.yaml
    config/robots_room_315_only.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES
    config/room_315_kinematics/README.md
    config/room_315_kinematics/rail_devices_left.yaml
    config/room_315_kinematics/rail_devices_right.yaml
    config/room_315_kinematics/rail_network_left.yaml
    config/room_315_kinematics/rail_network_right.yaml
  DESTINATION share/${PROJECT_NAME}/config/room_315_kinematics
)

install(
  DIRECTORY
    config/room_315_kinematics/raw_segments
  DESTINATION share/${PROJECT_NAME}/config/room_315_kinematics
)

install(
  PROGRAMS
    scripts/conveyor_loop_mode_controller.py
    scripts/robot_gripper_command.py
    scripts/robot_joint_command.py
    scripts/room_315_device_position_tool.py
    scripts/room_315_kinematic_shuttle.py
    scripts/room_315_kinematic_shuttle_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_robot_gripper_bridge
    test/test_robot_gripper_bridge.py
  )
  ament_add_pytest_test(
    test_robot_gripper_command
    test/test_robot_gripper_command.py
  )
  ament_add_pytest_test(
    test_gripper_range_launch
    test/test_gripper_range_launch.py
  )
  ament_add_pytest_test(
    test_robot_joint_command
    test/test_robot_joint_command.py
  )
endif()

ament_package()
