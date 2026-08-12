import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch',
        'sim_bringup.launch.py',
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'mode': 'sim'}.items(),
    )
    return LaunchDescription([
        bringup,
        launch_testing.actions.ReadyToTest(),
    ])


class TestBringupNodeGraph(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('bringup_contract_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_foundation_nodes_are_running(self):
        deadline = time.monotonic() + 10.0
        names = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            names = set(self.node.get_node_names())
            if {'base_status_node', 'health_reporter'} <= names:
                break

        self.assertIn('base_status_node', names)
        self.assertIn('health_reporter', names)

    def test_sim_mode_parameters(self):
        for node_name in ('base_status_node', 'health_reporter'):
            client = self.node.create_client(
                GetParameters, f'/{node_name}/get_parameters')
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))
            request = GetParameters.Request()
            request.names = ['mode', 'use_sim_time']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=5.0)
            self.assertTrue(future.done())
            values = future.result().values
            self.assertEqual('sim', values[0].string_value)
            self.assertTrue(values[1].bool_value)
            self.node.destroy_client(client)

    def test_diagnostics_contract(self):
        received = []
        subscription = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda message: received.append((time.monotonic(), message)),
            10,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(received) < 3:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreaterEqual(len(received), 3)
        status = received[-1][1].status[0]
        self.assertEqual('ai_robot_tools/health_reporter', status.name)
        self.assertIn('hardware control is disabled', status.message)
        self.assertEqual(
            'sim',
            {value.key: value.value for value in status.values}['mode'],
        )

        intervals = [
            current[0] - previous[0]
            for previous, current in zip(received, received[1:])
        ]
        for interval in intervals:
            self.assertGreater(interval, 0.5)
            self.assertLess(interval, 1.5)

        publishers = self.node.get_publishers_info_by_topic('/diagnostics')
        self.assertEqual(1, len(publishers))
        qos = publishers[0].qos_profile
        self.assertEqual(ReliabilityPolicy.RELIABLE, qos.reliability)
        self.assertEqual(DurabilityPolicy.VOLATILE, qos.durability)
        self.node.destroy_subscription(subscription)
