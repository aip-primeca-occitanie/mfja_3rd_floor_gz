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
    config/mfja_light.gui.config
    config/robots.yaml
    config/robots_room_315_only.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES
    config/room_315_vla/action_space.yaml
    config/room_315_vla/payload_training_cases_expanded_160_speed_sweep.yaml
    config/room_315_vla/payload_scenarios.yaml
    config/room_315_vla/shuttle_identity.yaml
    config/room_315_vla/vla_supervisor.yaml
  DESTINATION share/${PROJECT_NAME}/config/room_315_vla
)

install(
  DIRECTORY
    config/room_315_vla/pddl
  DESTINATION share/${PROJECT_NAME}/config/room_315_vla
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
    scripts/presentation_left_slot3_kuka_slot2.py
    scripts/robot_joint_command.py
    scripts/room_315_device_position_tool.py
    scripts/room_315_kinematic_shuttle.py
    scripts/room_315_kinematic_shuttle_node.py
    scripts/room_315_multi_shuttle.py
    scripts/room_315_pddl_dataset_report.py
    scripts/room_315_pddl_language_generator.py
    scripts/room_315_pddl_plan_translator.py
    scripts/room_315_pddl_scenario_generator.py
    scripts/room_315_payload_case_batch_runner.py
    scripts/room_315_payload_case_review.py
    scripts/room_315_real_vla_agent.py
    scripts/room_315_vla_baseline_eval.py
    scripts/room_315_vla_dataset_recorder.py
    scripts/room_315_vla_event_extractor.py
    scripts/room_315_vla_obstacle_tool.py
    scripts/room_315_vla_shuttle_identity_tracker.py
    scripts/room_315_vla_split_dataset.py
    scripts/room_315_vla_supervisor.py
    scripts/room_315_vla_train_local.py
    scripts/room_315_vla_to_lerobot.py
    scripts/room_315_smolvla_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_robot_joint_command
    test/test_robot_joint_command.py
  )
  ament_add_pytest_test(
    test_room315_device_marker_lifecycle
    test/test_room315_device_marker_lifecycle.py
  )
  ament_add_pytest_test(
    test_presentation_left_slot3_kuka_slot2
    test/test_presentation_left_slot3_kuka_slot2.py
  )
  ament_add_pytest_test(
    test_room315_rail_devices
    test/test_room315_rail_devices.py
  )
  ament_add_pytest_test(
    test_room315_kinematic_shuttle_core
    test/test_room315_kinematic_shuttle_core.py
  )
  ament_add_pytest_test(
    test_room315_pddl_planning
    test/test_room315_pddl_planning.py
  )
  ament_add_pytest_test(
    test_room315_pddl_language_generator
    test/test_room315_pddl_language_generator.py
  )
  ament_add_pytest_test(
    test_room315_pddl_dataset_report
    test/test_room315_pddl_dataset_report.py
  )
  ament_add_pytest_test(
    test_room315_pddl_plan_translator
    test/test_room315_pddl_plan_translator.py
  )
  ament_add_pytest_test(
    test_room315_pddl_planner_backend
    test/test_room315_pddl_planner_backend.py
  )
  ament_add_pytest_test(
    test_room315_pddl_scenario_generator
    test/test_room315_pddl_scenario_generator.py
    TIMEOUT 180
  )
  ament_add_pytest_test(
    test_room315_action_schema_v3
    test/test_room315_action_schema_v3.py
  )
  ament_add_pytest_test(
    test_room315_multi_shuttle_registry
    test/test_room315_multi_shuttle_registry.py
  )
  ament_add_pytest_test(
    test_room315_fleet_supervisor_safety
    test/test_room315_fleet_supervisor_safety.py
  )
  ament_add_pytest_test(
    test_room315_block_reservation_runtime
    test/test_room315_block_reservation_runtime.py
  )
  ament_add_pytest_test(
    test_room315_headway_safety
    test/test_room315_headway_safety.py
  )
  ament_add_pytest_test(
    test_room315_shuttle_identity_config
    test/test_room315_shuttle_identity_config.py
  )
  ament_add_pytest_test(
    test_room315_shuttle_identity_visual_assets
    test/test_room315_shuttle_identity_visual_assets.py
  )
  ament_add_pytest_test(
    test_room315_shuttle_identity_tracker
    test/test_room315_shuttle_identity_tracker.py
  )
  ament_add_pytest_test(
    test_room315_multi_shuttle_dataset_recorder
    test/test_room315_multi_shuttle_dataset_recorder.py
  )
  ament_add_pytest_test(
    test_room315_payload_occlusion_metadata
    test/test_room315_payload_occlusion_metadata.py
  )
  ament_add_pytest_test(
    test_room315_payload_models
    test/test_room315_payload_models.py
  )
  ament_add_pytest_test(
    test_room315_shuttle_payload_visuals
    test/test_room315_shuttle_payload_visuals.py
  )
  ament_add_pytest_test(
    test_room315_identity_keepout_zones
    test/test_room315_identity_keepout_zones.py
  )
  ament_add_pytest_test(
    test_room315_multi_shuttle_pddl_planning
    test/test_room315_multi_shuttle_pddl_planning.py
  )
  ament_add_pytest_test(
    test_room315_multi_shuttle_dataset_report
    test/test_room315_multi_shuttle_dataset_report.py
  )
  ament_add_pytest_test(
    test_room315_vla_model_input_boundary
    test/test_room315_vla_model_input_boundary.py
  )
  ament_add_pytest_test(
    test_room315_multi_shuttle_launch_args
    test/test_room315_multi_shuttle_launch_args.py
  )
  ament_add_pytest_test(
    test_room315_vla_baseline_eval
    test/test_room315_vla_baseline_eval.py
  )
  ament_add_pytest_test(
    test_room315_vla_dataset_recorder
    test/test_room315_vla_dataset_recorder.py
  )
  ament_add_pytest_test(
    test_room315_real_vla_agent
    test/test_room315_real_vla_agent.py
  )
  ament_add_pytest_test(
    test_room315_real_vla_agent_schema_v3
    test/test_room315_real_vla_agent_schema_v3.py
  )
  ament_add_pytest_test(
    test_room315_vla_event_extractor
    test/test_room315_vla_event_extractor.py
  )
  ament_add_pytest_test(
    test_room315_vla_supervisor
    test/test_room315_vla_supervisor.py
  )
  ament_add_pytest_test(
    test_room315_vla_smoke
    test/test_room315_vla_smoke.py
  )
  ament_add_pytest_test(
    test_room315_vla_training_tools
    test/test_room315_vla_training_tools.py
  )
  ament_add_pytest_test(
    test_room315_visual_debug_colors
    test/test_room315_visual_debug_colors.py
  )
endif()

ament_package()
