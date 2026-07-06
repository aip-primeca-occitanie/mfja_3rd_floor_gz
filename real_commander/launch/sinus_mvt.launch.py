import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("staubli_tx2_60l")
        .robot_description(file_path="config/staubli_tx2_60l.urdf.xacro")
        .robot_description_semantic(file_path="config/staubli_tx2_60l.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Trajectory execution functionality
    controllers_yaml = xacro.load_yaml(
        os.path.join(
            get_package_share_directory("staubli_tx2_60l_moveit_config"),
            "config",
            "staubli_tx2_60l_controllers.yaml",
            )
        )
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }
    
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 100.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict(),
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters],
        arguments=["--ros-args", "--log-level", "info"],
        )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map",
                   "--child-frame-id", "base_link"],
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
        )

    cartesian_converter_node = Node(
        package='real_commander',
        executable='cartesian_converter',
        name='cartesian_converter',
        output='screen',
        )

    cartesian_publisher_node = Node(
        package='real_commander',
        executable='cartesian_publisher',
        name='cartesian_publisher',
        output='screen',
        #parameters=[{'velocity_ratio' : 0.5}], # % velocity
        )

    sinus_mvt = Node(
        package='real_commander',
        executable='sinus_mvt',
        name='sinus_mvt',
        output='screen',
        parameters=[{
            "AMPLITUDE" : 0.05,
            "OMEGA" : 0.8,
            "LOOP_FREQ" :250,
            }]
        )

    timer_action = TimerAction(
        period=3.0,
        actions=[cartesian_publisher_node, cartesian_converter_node]
        )

    force_pid_controller_delayed = TimerAction(
        period=7.0,
        actions=[sinus_mvt]
        )

    return LaunchDescription(
        [
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        timer_action,
        force_pid_controller_delayed
        ]
    )