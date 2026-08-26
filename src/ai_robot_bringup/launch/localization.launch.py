"""S1-M5 AMCL localization entry point with exclusive map -> odom ownership."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('ai_robot_bringup'))
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    mode = LaunchConfiguration('mode')
    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    sim_time_text = PythonExpression([
        "'true' if '", mode, "' == 'sim' else 'false'"])
    use_sim_time = ParameterValue(sim_time_text, value_type=bool)
    sim_condition = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))

    platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'm4_bringup.launch.py')),
        launch_arguments={
            'mode': mode,
            'world_file': world_file,
            'scan_output_topic': LaunchConfiguration('scan_output_topic'),
        }.items(),
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='m5_map_server',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
        condition=sim_condition,
        output='screen',
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        parameters=[amcl_params_file, {
            'use_sim_time': use_sim_time,
            'set_initial_pose': ParameterValue(
                set_initial_pose, value_type=bool),
        }],
        condition=sim_condition,
        output='screen',
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'autostart': True,
            'use_sim_time': use_sim_time,
            'node_names': ['m5_map_server', 'amcl'],
        }],
        condition=sim_condition,
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument(
            'world_file',
            default_value=str(sim_share / 'worlds' / 'm5_navigation.sdf')),
        DeclareLaunchArgument(
            'map_file',
            default_value=str(share / 'maps' / 'm5_complete.yaml')),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=str(share / 'config' / 'amcl_m5.yaml')),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='false',
            choices=['true', 'false']),
        DeclareLaunchArgument('scan_output_topic', default_value='/scan'),
        platform,
        map_server,
        amcl,
        lifecycle,
    ])
