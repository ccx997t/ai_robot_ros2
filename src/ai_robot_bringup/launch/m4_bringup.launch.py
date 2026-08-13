"""S1-M4 unified integration entry point.

Startup phases are deliberately declared in this order:
1. mode-independent safety, odometry contract and diagnostics nodes;
2. the selected low-level implementation;
3. controller spawning inside the simulation implementation after robot creation.

Real mode remains fail-safe until real drivers are admitted: it starts no simulator,
sensor driver or motor controller and therefore cannot produce hardware motion.
"""

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
    mode = LaunchConfiguration('mode')
    sim_time_text = PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])
    use_sim_time = ParameterValue(sim_time_text, value_type=bool)
    common_parameters = {'mode': mode, 'use_sim_time': use_sim_time}
    sensors_launch = (
        Path(get_package_share_directory('ai_robot_sim')) / 'launch' / 'sensors.launch.py'
    )

    safety_layer = [
        Node(
            package='ai_robot_base', executable='base_status_node',
            parameters=[common_parameters], output='screen',
        ),
        Node(
            package='ai_robot_base', executable='cmd_vel_safety_node',
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(
            package='ai_robot_base', executable='odom_contract_relay',
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(
            package='ai_robot_tools', executable='health_reporter',
            parameters=[common_parameters], output='screen',
        ),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sensors_launch)),
        launch_arguments={'use_sim_time': sim_time_text}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        *safety_layer,
        simulation,
    ])
