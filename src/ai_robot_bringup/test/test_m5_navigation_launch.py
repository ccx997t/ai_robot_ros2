from collections import deque
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m5_navigation_{os.getpid()}'


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch', 'navigation.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'mode': 'sim'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM5NavigationBaseline(unittest.TestCase):
    MANAGED_NODES = (
        'planner_server', 'controller_server', 'behavior_server',
        'bt_navigator')

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m5_navigation_baseline_test')
        cls.global_costmaps = deque(maxlen=5)
        cls.local_costmaps = deque(maxlen=5)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.global_sub = cls.node.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            cls.global_costmaps.append, qos)
        cls.local_sub = cls.node.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            cls.local_costmaps.append, qos)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def spin_until(cls, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def wait_for_active(self, node_name, timeout=60.0):
        client = self.node.create_client(
            GetState, f'/{node_name}/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=45.0))
        deadline = time.monotonic() + timeout
        state = None
        while time.monotonic() < deadline:
            future = client.call_async(GetState.Request())
            if self.spin_until(future.done, 5.0):
                state = future.result().current_state
                if state.id == 3:
                    self.node.destroy_client(client)
                    return state
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_client(client)
        label = state.label if state is not None else 'unavailable'
        self.fail(f'{node_name} did not become active: {label}')

    def test_servers_costmaps_and_safe_command_route(self):
        for node_name in self.MANAGED_NODES:
            self.wait_for_active(node_name)

        self.assertTrue(self.spin_until(
            lambda: self.global_costmaps and self.local_costmaps, 30.0),
            'global/local costmaps were not published')
        global_grid = self.global_costmaps[-1]
        local_grid = self.local_costmaps[-1]
        self.assertEqual('map', global_grid.header.frame_id)
        self.assertEqual('odom', local_grid.header.frame_id)
        self.assertEqual(235, global_grid.info.width)
        self.assertEqual(197, global_grid.info.height)
        self.assertAlmostEqual(0.05, global_grid.info.resolution, places=6)
        self.assertAlmostEqual(0.05, local_grid.info.resolution, places=6)
        self.assertEqual(80, local_grid.info.width)
        self.assertEqual(80, local_grid.info.height)

        publishers = self.node.get_publishers_info_by_topic('/cmd_vel')
        subscribers = self.node.get_subscriptions_info_by_topic('/cmd_vel')
        self.assertIn('controller_server', {
            endpoint.node_name for endpoint in publishers})
        self.assertIn('cmd_vel_safety', {
            endpoint.node_name for endpoint in subscribers})
        direct_publishers = self.node.get_publishers_info_by_topic(
            '/base_controller/cmd_vel_unstamped')
        self.assertEqual(
            {'cmd_vel_safety'},
            {endpoint.node_name for endpoint in direct_publishers})
