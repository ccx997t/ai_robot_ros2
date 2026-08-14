from collections import deque
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(
            package='ai_robot_base', executable='cmd_vel_safety_node',
            parameters=[{
                'output_topic': '/test/base_cmd',
                'command_timeout_seconds': 0.5,
                'max_linear_speed_mps': 0.3,
                'max_angular_speed_rps': 0.8,
            }],
            output='screen',
        ),
        Node(
            package='ai_robot_sensors', executable='fault_injector',
            name='command_fault_injector',
            parameters=[{
                'message_type': 'twist',
                'fault_mode': 'drop',
                'input_topic': '/test/cmd_source',
                'output_topic': '/cmd_vel',
                'initially_enabled': False,
            }],
            output='screen',
        ),
        Node(
            package='ai_robot_sensors', executable='fault_injector',
            name='scan_fault_injector',
            parameters=[{
                'message_type': 'scan',
                'fault_mode': 'bad_frame',
                'input_topic': '/test/scan_source',
                'output_topic': '/test/scan_faulted',
                'bad_frame': 'invalid_laser_frame',
                'initially_enabled': False,
            }],
            output='screen',
        ),
        Node(
            package='ai_robot_sensors', executable='sensor_adapter',
            name='fault_scan_monitor',
            parameters=[{
                'sensor_type': 'scan',
                'input_topic': '/test/scan_faulted',
                'output_topic': '/test/scan_public',
                'frame_id': 'laser_link',
                'diagnostic_name': 'fault_lidar',
                'expected_rate': 20.0,
                'validate_input_frame': True,
            }],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM4FaultDiagnosticsAndSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m4_fault_safety_test')
        cls.base_commands = deque(maxlen=500)
        cls.diagnostics = {}
        cls.command_source = cls.node.create_publisher(
            Twist, '/test/cmd_source', 10)
        cls.scan_source = cls.node.create_publisher(
            LaserScan, '/test/scan_source', qos_profile_sensor_data)
        cls.subscriptions = [
            cls.node.create_subscription(
                Twist, '/test/base_cmd', cls.base_commands.append, 10),
            cls.node.create_subscription(
                DiagnosticArray, '/diagnostics', cls.receive_diagnostics, 10),
        ]
        cls.command_toggle = cls.node.create_client(
            SetBool, '/command_fault_injector/enable')
        cls.scan_toggle = cls.node.create_client(
            SetBool, '/scan_fault_injector/enable')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def receive_diagnostics(cls, message):
        for status in message.status:
            cls.diagnostics[status.name] = status

    @classmethod
    def spin_until(cls, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def toggle(cls, client, enabled):
        assert client.wait_for_service(timeout_sec=5.0)
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        assert cls.spin_until(future.done, 5.0)
        assert future.result().success

    @classmethod
    def publish_command_for(cls, speed, duration):
        message = Twist()
        message.linear.x = speed
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            cls.command_source.publish(message)
            rclpy.spin_once(cls.node, timeout_sec=0.01)
            time.sleep(0.04)

    @classmethod
    def publish_scan_for(cls, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = LaserScan()
            message.header.stamp = cls.node.get_clock().now().to_msg()
            message.header.frame_id = 'laser_link'
            message.angle_min = -0.1
            message.angle_max = 0.1
            message.angle_increment = 0.1
            message.range_min = 0.1
            message.range_max = 10.0
            message.ranges = [1.0, 1.1, 1.0]
            cls.scan_source.publish(message)
            rclpy.spin_once(cls.node, timeout_sec=0.01)
            time.sleep(0.04)

    def test_fault_diagnostics_limit_timeout_and_recovery(self):
        self.assertTrue(self.spin_until(
            lambda: self.command_source.get_subscription_count() == 1
            and self.scan_source.get_subscription_count() == 1,
            10.0,
        ))

        # Overspeed commands must still traverse the safety node and be limited.
        self.publish_command_for(1.0, 0.4)
        self.assertTrue(self.base_commands)
        self.assertAlmostEqual(0.3, self.base_commands[-1].linear.x, places=6)
        self.assertTrue(self.spin_until(
            lambda: (
                self.diagnostics.get('base/cmd_vel_safety') is not None
                and self.diagnostics['base/cmd_vel_safety'].message
                == 'command limited'
            ),
            2.0,
        ))
        safety = self.diagnostics['base/cmd_vel_safety']
        self.assertEqual(DiagnosticStatus.WARN, safety.level)
        self.assertEqual('command limited', safety.message)

        # Dropped upstream commands trigger the existing 0.5 s watchdog stop.
        self.toggle(self.command_toggle, True)
        self.assertTrue(self.spin_until(
            lambda: (
                self.diagnostics.get('fault_injection/twist') is not None
                and self.diagnostics['fault_injection/twist'].level
                == DiagnosticStatus.WARN
            ),
            2.0,
        ))
        self.base_commands.clear()
        start = 0
        self.publish_command_for(0.2, 0.8)
        self.assertTrue(self.spin_until(
            lambda: (
                self.diagnostics.get('base/cmd_vel_safety') is not None
                and self.diagnostics['base/cmd_vel_safety'].message
                == 'command timeout; stop sent'
                and any(
                    message.linear.x == 0.0
                    for message in list(self.base_commands)[start:]
                )
            ),
            2.0,
        ), msg=(
            f'safety={self.diagnostics.get("base/cmd_vel_safety")}, '
            f'fault={self.diagnostics.get("fault_injection/twist")}, '
            f'commands_after_fault={len(self.base_commands) - start}'
        ))
        safety = self.diagnostics['base/cmd_vel_safety']
        self.assertEqual(DiagnosticStatus.WARN, safety.level)
        self.assertEqual('command timeout; stop sent', safety.message)
        command_fault = self.diagnostics['fault_injection/twist']
        self.assertEqual(DiagnosticStatus.WARN, command_fault.level)

        # Disabling the injection restores accepted commands without restart.
        self.toggle(self.command_toggle, False)
        self.publish_command_for(0.2, 0.3)
        self.assertAlmostEqual(0.2, self.base_commands[-1].linear.x, places=6)
        safety = self.diagnostics['base/cmd_vel_safety']
        self.assertEqual(DiagnosticStatus.OK, safety.level)
        self.assertEqual('command accepted', safety.message)
        self.assertEqual(
            DiagnosticStatus.OK,
            self.diagnostics['fault_injection/twist'].level,
        )

        # A bad source frame must propagate to the sensor diagnostic, then clear.
        self.publish_scan_for(1.3)
        lidar = self.diagnostics.get('sensors/fault_lidar')
        self.assertIsNotNone(lidar)
        self.assertEqual(DiagnosticStatus.OK, lidar.level)

        self.toggle(self.scan_toggle, True)
        self.publish_scan_for(1.3)
        lidar = self.diagnostics['sensors/fault_lidar']
        self.assertEqual(DiagnosticStatus.ERROR, lidar.level)
        self.assertEqual('sensor contract invalid', lidar.message)
        values = {item.key: item.value for item in lidar.values}
        self.assertEqual('false', values['frame_valid'])

        self.toggle(self.scan_toggle, False)
        self.publish_scan_for(1.3)
        lidar = self.diagnostics['sensors/fault_lidar']
        self.assertEqual(DiagnosticStatus.OK, lidar.level)
        self.assertEqual('sensor contract valid', lidar.message)
