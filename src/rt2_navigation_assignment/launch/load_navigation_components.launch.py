import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_container = LaunchConfiguration('target_container')

    # Both navigation nodes are composable nodes. This launch file assumes that a
    # multithreaded component container is already running in an interactive terminal.
    load_components = LoadComposableNodes(
        target_container=target_container,
        composable_node_descriptions=[
            ComposableNode(
                package='rt2_navigation_assignment',
                plugin='rt2_navigation_assignment::NavigationActionServer',
                name='navigation_action_server',
                parameters=[{
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                }],
            ),

            # The client owns the keyboard UI. Loading it into the visible container
            # keeps stdin connected, so commands such as "goal 2.0 1.0 0.0" work.
            ComposableNode(
                package='rt2_navigation_assignment',
                plugin='rt2_navigation_assignment::NavigationClientComponent',
                name='navigation_user_interface',
                parameters=[{
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'action_name': 'navigate_to_pose',
                }],
            ),
        ],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo.'
        ),

        DeclareLaunchArgument(
            'target_container',
            default_value='/ComponentManager',
            description='Name of the already running component container.'
        ),

        load_components,
    ])
