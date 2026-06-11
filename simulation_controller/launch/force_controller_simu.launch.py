import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    ctrl_pkg = get_package_share_directory("staubli_tx2_60l_controller")
    robot_description_config = xacro.process_file( os.path.join(ctrl_pkg, "config", "staubli_tx2_60l.urdf.xacro"))
    robot_description = {"robot_description": robot_description_config.toxml()}
    robot_controllers = os.path.join(ctrl_pkg, 'config', 'ros2_controllers.yaml')
    rviz_config_file = PathJoinSubstitution([FindPackageShare("staubli_tx2_60l_description"), "rviz", "view_tx2_60l.rviz"])

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

    force_pid_controller_node = Node(
        package='simulation_controller',
        executable='force_pid_controller',
        name='force_pid_controller',
        output='screen',
    )

    force_simulation_node = Node(
        package='simulation_controller',
        executable='force_simulation',
        name='force_simulation',
        output='screen',
    )

    timer_action = TimerAction(
        period=5.0,
        actions=[cartesian_publisher_node,cartesian_converter_node,force_simulation_node] #10 might not be sufficient, to improve
    )

    force_pid_controller_delayed = TimerAction(
        period=10.0,
        actions=[force_pid_controller_node]
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher_node,
        rviz,
        spawner_jsb,
        spawner_pos,
        controller_node,
        timer_action,
        force_pid_controller_delayed,
    ])