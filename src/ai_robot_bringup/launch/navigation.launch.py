"""S1-M5 lifecycle-managed Nav2 planning and control entry point."""

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
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    sim_time_text = PythonExpression([
        "'true' if '", mode, "' == 'sim' else 'false'"])
    use_sim_time = ParameterValue(sim_time_text, value_type=bool)
    sim_condition = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'localization.launch.py')),
        launch_arguments={
            'mode': mode,
            'world_file': world_file,
            'map_file': map_file,
            'amcl_params_file': amcl_params_file,
            'set_initial_pose': 'true',
            'scan_output_topic': LaunchConfiguration('scan_output_topic'),
        }.items(),
    )
    managed_nodes = [
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ]
    nav_nodes = [
        Node(
            package=package, executable=executable, name=name,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            condition=sim_condition,
            output='screen',
        )
        for package, executable, name in managed_nodes
    ]
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'autostart': True,
            'use_sim_time': use_sim_time,
            'node_names': [name for _, _, name in managed_nodes],
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
            'nav2_params_file',
            default_value=str(share / 'config' / 'nav2_m5.yaml')),
        DeclareLaunchArgument('scan_output_topic', default_value='/scan'),
        localization,
        *nav_nodes,
        lifecycle,
    ])
