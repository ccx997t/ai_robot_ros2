from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    behavior_share = Path(get_package_share_directory('ai_robot_behaviors'))
    sim_share = Path(get_package_share_directory('ai_robot_sim'))

    preview = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / 'launch' / 'yellow_robot_preview.launch.py')),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'enable_lidar': 'true',
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
        }.items(),
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/sim/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen',
    )
    scan_adapter = Node(
        package='ai_robot_sensors',
        executable='sensor_adapter',
        name='yellow_robot_lidar_adapter',
        parameters=[{
            'sensor_type': 'scan',
            'input_topic': '/sim/scan',
            'output_topic': '/scan',
            'frame_id': 'laser_link',
            'diagnostic_name': 'lidar',
            'expected_rate': 10.0,
            'use_sim_time': True,
        }],
        output='screen',
    )
    avoidance = Node(
        package='ai_robot_behaviors',
        executable='reactive_avoidance',
        name='reactive_avoidance',
        parameters=[str(behavior_share / 'config' / 'reactive_avoidance.yaml')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value=str(sim_share / 'worlds' / 'house_from_reference.sdf'),
        ),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-2.5'),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        preview,
        bridge,
        scan_adapter,
        avoidance,
    ])
