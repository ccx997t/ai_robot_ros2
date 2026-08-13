from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def adapter(name, sensor_type, input_topic, output_topic, frame_id, rate):
    return Node(
        package='ai_robot_sensors', executable='sensor_adapter', name=name,
        parameters=[{
            'sensor_type': sensor_type,
            'input_topic': input_topic,
            'output_topic': output_topic,
            'frame_id': frame_id,
            'diagnostic_name': name.removesuffix('_adapter'),
            'expected_rate': rate,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )


def generate_launch_description():
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'sim_base.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_lidar': 'true',
            'enable_camera': 'true',
            'enable_imu': 'true',
        }.items(),
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/sim/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/sim/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/sim/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/sim/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
    )
    encoder_monitor = Node(
        package='ai_robot_sensors', executable='sensor_adapter', name='encoder_monitor',
        parameters=[{
            'sensor_type': 'joint_state', 'input_topic': '/joint_states',
            'output_topic': '/joint_states', 'frame_id': 'base_link',
            'diagnostic_name': 'encoder', 'expected_rate': 100.0,
            'monitor_only': True,
            'required_joints': ['left_wheel_joint', 'right_wheel_joint'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        base,
        bridge,
        adapter('lidar_adapter', 'scan', '/sim/scan', '/scan', 'laser_link', 10.0),
        adapter('camera_image_adapter', 'image', '/sim/camera/image_raw',
                '/camera/image_raw', 'camera_optical_link', 15.0),
        adapter('camera_info_adapter', 'camera_info', '/sim/camera/camera_info',
                '/camera/camera_info', 'camera_optical_link', 15.0),
        adapter('imu_adapter', 'imu', '/sim/imu/data', '/imu/data', 'imu_link', 100.0),
        encoder_monitor,
    ])

