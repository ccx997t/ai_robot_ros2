import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch', 'map_server.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'mode': 'real',
                'map_topic': '/m5_reloaded_map',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM5MapReload(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m5_map_reload_test')
        cls.map = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.subscription = cls.node.create_subscription(
            OccupancyGrid, '/m5_reloaded_map', cls.receive_map, qos)

    @classmethod
    def receive_map(cls, message):
        cls.map = message

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_reloaded_grid_matches_git_manifest(self):
        deadline = time.monotonic() + 15.0
        while self.map is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIsNotNone(self.map, 'map_server did not publish the map')

        grid = self.map
        self.assertEqual('map', grid.header.frame_id)
        self.assertEqual(235, grid.info.width)
        self.assertEqual(197, grid.info.height)
        self.assertAlmostEqual(0.05, grid.info.resolution, places=6)
        self.assertAlmostEqual(-5.93, grid.info.origin.position.x, places=6)
        self.assertAlmostEqual(-4.93, grid.info.origin.position.y, places=6)
        self.assertEqual(57, sum(value == 100 for value in grid.data))
        self.assertEqual(1227, sum(value == 0 for value in grid.data))
        self.assertEqual(45011, sum(value == -1 for value in grid.data))
        self.assertEqual(235 * 197, len(grid.data))
