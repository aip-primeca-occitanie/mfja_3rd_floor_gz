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
    config/room_315_kinematics/segment_summary.yaml
    config/room_315_kinematics/validation_report.yaml
  DESTINATION share/${PROJECT_NAME}/config/room_315_kinematics
)

install(
  DIRECTORY
    config/room_315_kinematics/normalized_segments
    config/room_315_kinematics/raw_segments
  DESTINATION share/${PROJECT_NAME}/config/room_315_kinematics
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
    scripts/room_315_report_figures.py
    scripts/watch_sensor_feedback.py
    scripts/multi_robot_sync_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
