from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('sim', 'real'):
        raise RuntimeError('mode must be sim or real')
    return [Node(
        package='ai_robot_task_manager',
        executable='task_manager',
        name='task_manager',
        output='screen',
        parameters=[{
            'mode': mode,
            'use_sim_time': mode == 'sim',
            'navigation_enabled': False,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim'),
        OpaqueFunction(function=launch_setup),
    ])
