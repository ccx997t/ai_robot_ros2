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
            'enable_lidar': 'false',
            'enable_camera': 'false',
            'enable_imu': 'false',
        }.items(),
    )
    monitor = Node(
        package='ai_robot_sensors', executable='sensor_adapter', name='encoder_monitor',
        parameters=[{
            'sensor_type': 'joint_state',
            'input_topic': '/joint_states',
            'output_topic': '/joint_states',
            'frame_id': 'base_link',
            'diagnostic_name': 'encoder',
            'expected_rate': 100.0,
            'monitor_only': True,
            'required_joints': ['left_wheel_joint', 'right_wheel_joint'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        base, monitor,
    ])
