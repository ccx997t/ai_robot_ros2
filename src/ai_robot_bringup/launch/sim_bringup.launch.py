from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    use_sim_time = ParameterValue(
        PythonExpression(["'", mode, "' == 'sim'"]),
        value_type=bool,
    )
    parameters = {'mode': mode, 'use_sim_time': use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        Node(package='ai_robot_base', executable='base_status_node', parameters=[parameters], output='screen'),
        Node(package='ai_robot_tools', executable='health_reporter', parameters=[parameters], output='screen'),
    ])
