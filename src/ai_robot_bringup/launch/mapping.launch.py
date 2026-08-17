"""S1-M5 SLAM mapping entry point.

Simulation mode reuses the M4 sensor, fused odometry and safety baseline with
the M5 navigation world. slam_toolbox is the sole map -> odom authority. Real
mode remains fail-safe and starts neither simulation nor SLAM until entity
drivers are admitted in a later milestone.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('ai_robot_bringup'))
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    mode = LaunchConfiguration('mode')
    world_file = LaunchConfiguration('world_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    sim_condition = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))

    m4_baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / 'launch' / 'm4_bringup.launch.py')),
        launch_arguments={
            'mode': mode,
            'world_file': world_file,
        }.items(),
    )
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': True}],
        condition=sim_condition,
        output='screen',
    )
    map_saver = Node(
        package='nav2_map_server', executable='map_saver_server',
        name='map_saver',
        parameters=[{
            'use_sim_time': True,
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.19,
            'occupied_thresh_default': 0.65,
        }],
        condition=sim_condition,
        output='screen',
    )
    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_saver'],
        }],
        condition=sim_condition,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument(
            'world_file',
            default_value=str(sim_share / 'worlds' / 'm5_navigation.sdf'),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(
                bringup_share / 'config' / 'slam_toolbox_m5.yaml'),
        ),
        m4_baseline,
        slam,
        map_saver,
        map_saver_lifecycle,
    ])
