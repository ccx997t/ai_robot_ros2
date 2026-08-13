from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'sim_base.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_lidar': 'true', 'enable_camera': 'false', 'enable_imu': 'false',
        }.items(),
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/sim/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'], output='screen',
    )
    adapter = Node(
        package='ai_robot_sensors', executable='sensor_adapter', name='lidar_adapter',
        parameters=[{'sensor_type': 'scan', 'input_topic': '/sim/scan',
                     'output_topic': '/scan', 'frame_id': 'laser_link',
                     'expected_rate': 10.0, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        base, bridge, adapter,
    ])
