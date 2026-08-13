from launch import LaunchDescription
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    sim_time_text = PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])
    use_sim_time = ParameterValue(
        sim_time_text,
        value_type=bool,
    )
    parameters = {'mode': mode, 'use_sim_time': use_sim_time}
    sim_launch = Path(get_package_share_directory('ai_robot_sim')) / 'launch' / 'sim_base.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        Node(package='ai_robot_base', executable='base_status_node', parameters=[parameters], output='screen'),
        Node(
            package='ai_robot_base', executable='cmd_vel_safety_node',
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(
            package='ai_robot_base', executable='odom_contract_relay',
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(package='ai_robot_tools', executable='health_reporter', parameters=[parameters], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_launch)),
            launch_arguments={'use_sim_time': sim_time_text}.items(),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
        ),
    ])
