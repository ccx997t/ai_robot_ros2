from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        Node(package='ai_robot_base', executable='base_status_node', parameters=[{'mode': mode}], output='screen'),
        Node(package='ai_robot_tools', executable='health_reporter', parameters=[{'mode': mode}], output='screen'),
    ])
