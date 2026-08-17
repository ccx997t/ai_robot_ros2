"""Load the Git-tracked M5 map through a lifecycle-managed map server."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ai_robot_bringup'))
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map_file')
    map_topic = LaunchConfiguration('map_topic')
    use_sim_time = PythonExpression(["'", mode, "' == 'sim'"])

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='m5_map_server',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
        remappings=[('/map', map_topic)],
        output='screen',
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        parameters=[{
            'autostart': True,
            'use_sim_time': use_sim_time,
            'node_names': ['m5_map_server'],
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument(
            'map_file',
            default_value=str(share / 'maps' / 'm5_baseline.yaml')),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        map_server,
        lifecycle,
    ])
