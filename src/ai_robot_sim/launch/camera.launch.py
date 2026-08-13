from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'sim_base.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_lidar': 'false', 'enable_camera': 'true', 'enable_imu': 'false',
        }.items(),
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/sim/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/sim/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ], output='screen',
    )
    image_adapter = Node(
        package='ai_robot_sensors', executable='sensor_adapter', name='camera_image_adapter',
        parameters=[{'sensor_type': 'image', 'input_topic': '/sim/camera/image_raw',
                     'output_topic': '/camera/image_raw', 'frame_id': 'camera_optical_link',
                     'diagnostic_name': 'camera_image',
                     'expected_rate': 15.0, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    info_adapter = Node(
        package='ai_robot_sensors', executable='sensor_adapter', name='camera_info_adapter',
        parameters=[{'sensor_type': 'camera_info', 'input_topic': '/sim/camera/camera_info',
                     'output_topic': '/camera/camera_info', 'frame_id': 'camera_optical_link',
                     'diagnostic_name': 'camera_info',
                     'expected_rate': 15.0, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        base, bridge, image_adapter, info_adapter,
    ])
