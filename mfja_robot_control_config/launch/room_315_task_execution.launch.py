from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    domain_path = PathJoinSubstitution([
        FindPackageShare('mfja_robot_control_config'),
        'config',
        'room_315_planning',
        'pddl',
        'domain_room315_runtime.pddl',
    ])
    planner = LifecycleNode(
        package='plansys2_planner',
        executable='planner_node',
        name='planner',
        namespace='',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_plansys2')),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('plansys2_bringup'),
                'params',
                'plansys2_params.yaml',
            ]),
        ],
    )
    configure_planner = TimerAction(
        period=1.0,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(planner),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )),
        ],
    )
    activate_planner = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=planner,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(planner),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    gateway = Node(
        package='mfja_robot_control_config',
        executable='room_315_task_execution_node.py',
        name='room_315_task_execution_node',
        output='screen',
        parameters=[
            LaunchConfiguration('runtime_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'execution_enabled': LaunchConfiguration('execution_enabled'),
                'planner_domain_path': domain_path,
                'planner_service': LaunchConfiguration('planner_service'),
                'external_obstacles_disabled': LaunchConfiguration(
                    'external_obstacles_disabled'
                ),
            },
        ],
    )
    shutdown_on_gateway_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gateway,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='Room 315 task execution gateway exited',
                )),
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'execution_enabled',
            default_value='false',
            choices=['true', 'false'],
            description='Explicit opt-in for supervised shuttle actuation.',
        ),
        DeclareLaunchArgument(
            'enable_plansys2',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'planner_service',
            default_value='/planner/get_plan',
        ),
        DeclareLaunchArgument(
            'external_obstacles_disabled',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Must be true only when the Gazebo removable-obstacle feature '
                'was launched disabled.'
            ),
        ),
        DeclareLaunchArgument(
            'runtime_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mfja_robot_control_config'),
                'config',
                'room_315_task_execution',
                'task_execution_runtime.yaml',
            ]),
        ),
        planner,
        configure_planner,
        activate_planner,
        gateway,
        shutdown_on_gateway_exit,
    ])
