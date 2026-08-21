"""Simulation-only M5 navigation sensor-timeout and recovery entry point."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ai_robot_bringup'))
    mode = LaunchConfiguration('mode')
    sim_condition = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    scan_source = '/m5_fault/scan_source'

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'navigation.launch.py')),
        launch_arguments={
            'mode': mode,
            'scan_output_topic': scan_source,
        }.items(),
    )
    scan_fault = Node(
        package='ai_robot_sensors', executable='fault_injector',
        name='navigation_scan_fault',
        parameters=[{
            'message_type': 'scan',
            'fault_mode': 'drop',
            'input_topic': scan_source,
            'output_topic': '/scan',
            'initially_enabled': False,
            'use_sim_time': True,
        }],
        condition=sim_condition,
        output='screen',
    )
    public_scan_monitor = Node(
        package='ai_robot_sensors', executable='sensor_adapter',
        name='navigation_lidar_monitor',
        parameters=[{
            'sensor_type': 'scan',
            'input_topic': '/scan',
            'output_topic': '/scan',
            'frame_id': 'laser_link',
            'diagnostic_name': 'navigation_lidar',
            'expected_rate': 10.0,
            'monitor_only': True,
            'validate_input_frame': True,
            'use_sim_time': True,
        }],
        condition=sim_condition,
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim', choices=['sim', 'real']),
        navigation,
        scan_fault,
        public_scan_monitor,
    ])
