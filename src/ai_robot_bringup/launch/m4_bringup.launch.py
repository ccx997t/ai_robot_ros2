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
    bringup_share = Path(get_package_share_directory('ai_robot_bringup'))
    sim_share = Path(get_package_share_directory('ai_robot_sim'))
    sensors_launch = (
        sim_share / 'launch' / 'sensors.launch.py'
    )
    sim_condition = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    real_condition = IfCondition(PythonExpression(["'", mode, "' == 'real'"]))

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
            package='ai_robot_tools', executable='health_reporter',
            parameters=[common_parameters], output='screen',
        ),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sensors_launch)),
        launch_arguments={
            'use_sim_time': sim_time_text,
            'controllers_file': str(sim_share / 'config' / 'controllers_m4.yaml'),
        }.items(),
        condition=sim_condition,
    )
    wheel_odom_relay = Node(
        package='ai_robot_base', executable='odom_contract_relay',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/base_controller/odom',
            'output_topic': '/wheel/odom',
        }],
        condition=sim_condition,
        output='screen',
    )
    fused_odometry = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[str(bringup_share / 'config' / 'ekf_m4.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom')],
        condition=sim_condition,
        output='screen',
    )
    real_odom_relay = Node(
        package='ai_robot_base', executable='odom_contract_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=real_condition,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        *safety_layer,
        simulation,
        wheel_odom_relay,
        fused_odometry,
        real_odom_relay,
    ])
