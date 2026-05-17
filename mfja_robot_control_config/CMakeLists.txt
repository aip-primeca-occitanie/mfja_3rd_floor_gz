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
  PROGRAMS
    scripts/conveyor_loop_mode_controller.py
    scripts/room_315_csv_preprocessor.py
    scripts/room_315_continuous_path_validator.py
    scripts/room_315_segment_plot.py
    scripts/room_315_network_validator.py
    scripts/room_315_devices_validator.py
    scripts/room_315_device_position_tool.py
    scripts/room_315_kinematic_shuttle.py
    scripts/room_315_kinematic_shuttle_node.py
    scripts/watch_sensor_feedback.py
    scripts/multi_robot_sync_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
