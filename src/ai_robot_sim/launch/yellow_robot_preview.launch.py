from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory('ai_robot_description'))
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    show_gui = LaunchConfiguration('gui')
    show_rviz = LaunchConfiguration('rviz')
    world_file = LaunchConfiguration('world_file')
    enable_lidar = LaunchConfiguration('enable_lidar')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'sim_base.launch.py')),
        launch_arguments={
            'robot_model_file': str(
                description_share / 'urdf' / 'yellow_robot_v1_2.urdf.xacro'
            ),
            'controllers_file': str(
                sim_share / 'config' / 'controllers_yellow_robot_v1_2.yaml'
            ),
            'world_file': world_file,
            'enable_lidar': enable_lidar,
            'enable_camera': 'false',
            'enable_imu': 'false',
            'robot_name': 'yellow_robot_v1_2',
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
        }.items(),
    )
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        condition=IfCondition(show_gui),
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='yellow_robot_rviz',
        arguments=['-d', str(sim_share / 'config' / 'yellow_robot_preview.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(show_rviz),
        output='screen',
    )
    command_safety = Node(
        package='ai_robot_base',
        executable='cmd_vel_safety_node',
        name='yellow_robot_cmd_vel_safety',
        parameters=[{
            'use_sim_time': True,
            'command_timeout_seconds': 0.5,
            'max_linear_speed_mps': 0.30,
            'max_angular_speed_rps': 0.80,
            'output_topic': '/base_controller/cmd_vel_unstamped',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'world_file',
            default_value=str(sim_share / 'worlds' / 'simple_maze.sdf'),
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-2.5'),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('enable_lidar', default_value='false', choices=['true', 'false']),
        simulation,
        command_safety,
        gazebo_gui,
        rviz,
    ])
