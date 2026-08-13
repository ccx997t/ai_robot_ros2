from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'sim_base.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_lidar': 'false',
            'enable_camera': 'false',
            'enable_imu': 'false',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        base,
    ])

