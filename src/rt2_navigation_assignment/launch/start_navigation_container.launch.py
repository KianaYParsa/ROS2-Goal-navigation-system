from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    container_name = LaunchConfiguration('container_name')

    navigation_container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package='rt2_navigation_assignment',
                plugin='rt2_navigation_assignment::NavigationActionServer',
                name='navigation_action_server',
                parameters=[{
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                }],
            ),
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo.'
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='rt2_navigation_container',
            description='Composable node container name.'
        ),
        navigation_container,
    ])
