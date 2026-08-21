import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    ctrl_pkg = get_package_share_directory("staubli_tx2_60l_controller")
    robot_description_config = xacro.process_file( os.path.join(ctrl_pkg, "config", "staubli_tx2_60l.urdf.xacro"))
    robot_description = {"robot_description": robot_description_config.toxml()}
    robot_controllers = os.path.join(ctrl_pkg, 'config', 'ros2_controllers.yaml')
    rviz_config_file = PathJoinSubstitution([FindPackageShare("staubli_tx2_60l_description"), "rviz", "view_tx2_60l.rviz"])

    plate_thickness = 0.4
    plate_z0 = -0.1

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[{'robot_description': robot_description}, robot_controllers],
        output='screen',
        )

    controller_node = Node(
        package='staubli_tx2_60l_controller',
        executable='staubli_controller',
        name='staubli_controller',
        output='screen',
        )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        )
    
    plate_marker_node = Node(
        package='simulation_controller',
        executable='plate_marker_publisher',
        name='plate_marker_publisher',
        output='screen',
        parameters=[{"thickness" : plate_thickness}]
        )

    plate_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="plate_tf",
        arguments=[
            "--x", "0.8",
            "--y", "0.0",
            "--z", str((2*plate_z0-plate_thickness)/2+0.5),
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "base_link",
            "--child-frame-id", "plate",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            ],
        )

    spawner_pos = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_controller',
            '--controller-manager', '/controller_manager',
            ],
        )

    cartesian_converter_node = Node(
        package='simulation_controller',
        executable='cartesian_converter',
        name='cartesian_converter',
        output='screen',
        )

    cartesian_publisher_node = Node(
        package='simulation_controller',
        executable='cartesian_publisher',
        name='cartesian_publisher',
        output='screen',
        )

    drill_controller_node = Node(
        package='simulation_controller',
        executable='drill_controller',
        name='drill_controller',
        output='screen',
        parameters=[{
            "KP0": 1e-2,
            "KP1": 1e-3,
            "KI1": 0.,
            "KD1": 0.,
            "MAX_OUTPUT": 0.02,
            "MAX_VELOCITY": 0.01,
            "TARGET": 150.0,
            "FORCE_THRESHOLD": 5.0,
            "FREQ_CONTROL": 80
            }]
        )

    drill_force_node = Node(
        package='simulation_controller',
        executable='drill_force_simulation',
        name='drill_force',
        output='screen',
        parameters=[{
            "drill_Dc": 0.006, #m
            "drill_n": 3000., #tr/min
            "mat_Kc1": 800., #MPa
            "mat_m0": 0.2,
            "mat_z0": plate_z0, #m
            "mat_zf": plate_z0-plate_thickness, #m
            "noise_sigma": 2.35, #N
            }]
        )

    filter_node = Node(
        package='simulation_controller',
        executable='sensor_filter',
        name='sensor_filter',
        output='screen',
        parameters=[{
            "kalman_q" : 2.825,
            "kalman_r" : 5.4,
            "median_window" : 10
            }]
        )

    timer_action = TimerAction(
        period=15.0,
        actions=[cartesian_publisher_node,cartesian_converter_node,drill_force_node]
        )

    drill_controller_delayed = TimerAction(
        period=20.0,
        actions=[drill_controller_node, filter_node]
        )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher_node,
        plate_marker_node,
        plate_tf,
        rviz,
        spawner_jsb,
        spawner_pos,
        controller_node,
        timer_action,
        drill_controller_delayed,
        ])