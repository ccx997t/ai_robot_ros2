from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    description_share = Path(get_package_share_directory('ai_robot_description'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))
    controllers = LaunchConfiguration('controllers_file')
    model = description_share / 'urdf' / 'ai_robot.urdf.xacro'
    world = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(model), ' controllers_file:=', controllers,
            ' enable_lidar:=', enable_lidar,
            ' enable_camera:=', enable_camera,
            ' enable_imu:=', enable_imu,
        ]),
        value_type=str,
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        # base_link is centered on the wheel axle. Spawn one wheel radius
        # above the floor so the chassis and caster do not penetrate it and
        # rob the drive wheels of traction.
        arguments=['-name', 'ai_robot', '-topic', 'robot_description', '-z', '0.075'],
        output='screen',
    )
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    base_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner, base_spawner])
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=str(sim_share / 'config' / 'controllers.yaml'),
        ),
        DeclareLaunchArgument(
            'world_file', default_value=str(sim_share / 'worlds' / 'm2_test.sdf'),
        ),
        DeclareLaunchArgument('enable_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_imu', default_value='false', choices=['true', 'false']),
        gazebo, state_publisher, spawn_robot, load_controllers,
    ])
