"""Simulation-only fault injection entry point for S1-M4 acceptance.

Place this relay between a source and its consumer. Toggle the configured fault
with `/fault_injector/enable` so a scenario can prove detection and recovery.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'message_type', default_value='scan',
            choices=['scan', 'imu', 'odometry', 'twist']),
        DeclareLaunchArgument(
            'fault_mode', default_value='drop',
            choices=['drop', 'zero_stamp', 'bad_frame', 'nonfinite_data']),
        DeclareLaunchArgument('input_topic', default_value='/fault/source'),
        DeclareLaunchArgument('output_topic', default_value='/fault/output'),
        DeclareLaunchArgument('bad_frame', default_value='fault_frame'),
        DeclareLaunchArgument(
            'initially_enabled', default_value='false', choices=['true', 'false']),
        Node(
            package='ai_robot_sensors', executable='fault_injector',
            name='fault_injector', output='screen',
            parameters=[{
                'use_sim_time': True,
                'message_type': LaunchConfiguration('message_type'),
                'fault_mode': LaunchConfiguration('fault_mode'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'bad_frame': LaunchConfiguration('bad_frame'),
                'initially_enabled': ParameterValue(
                    LaunchConfiguration('initially_enabled'), value_type=bool),
            }],
        ),
    ])
