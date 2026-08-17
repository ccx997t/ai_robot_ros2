from collections import deque
import os
from pathlib import Path
import time
import unittest

import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

os.environ['ROS_DOMAIN_ID'] = str(130 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m5_mapping_test_{os.getpid()}'
SAVE_DIR = Path('/tmp/ai_robot_m5_map_test')
SAVE_PREFIX = SAVE_DIR / 'm5_baseline'


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch', 'mapping.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'mode': 'sim'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM5MappingQuality(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        for suffix in ('.yaml', '.pgm'):
            (Path(str(SAVE_PREFIX) + suffix)).unlink(missing_ok=True)
        rclpy.init()
        cls.node = rclpy.create_node('m5_mapping_quality_test')
        cls.maps = deque(maxlen=20)
        cls.tf_pairs = set()
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.command_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.map_sub = cls.node.create_subscription(
            OccupancyGrid, '/map', cls.maps.append, map_qos)
        cls.tf_sub = cls.node.create_subscription(
            TFMessage, '/tf', cls.receive_tf, 100)

    @classmethod
    def receive_tf(cls, message):
        cls.tf_pairs.update(
            (transform.header.frame_id, transform.child_frame_id)
            for transform in message.transforms)

    @classmethod
    def tearDownClass(cls):
        cls.command_pub.publish(Twist())
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

    @classmethod
    def rotate_for_mapping(cls, seconds):
        command = Twist()
        command.angular.z = 0.35
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            cls.command_pub.publish(command)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
        cls.command_pub.publish(Twist())

    def test_map_tf_interface_and_quality(self):
        self.assertTrue(
            self.spin_until(lambda: len(self.maps) >= 1, 55.0),
            'slam_toolbox did not publish an initial map')
        self.rotate_for_mapping(19.0)
        initial_stamp = (
            self.maps[-1].header.stamp.sec,
            self.maps[-1].header.stamp.nanosec,
        )
        self.assertTrue(self.spin_until(
            lambda: (
                self.maps[-1].header.stamp.sec,
                self.maps[-1].header.stamp.nanosec,
            ) != initial_stamp,
            8.0,
        ))
        self.assertTrue(self.spin_until(
            lambda: ('map', 'odom') in self.tf_pairs, 8.0))

        grid = self.maps[-1]
        self.assertEqual('map', grid.header.frame_id)
        self.assertAlmostEqual(0.05, grid.info.resolution, places=6)
        self.assertGreaterEqual(grid.info.width, 150)
        self.assertGreaterEqual(grid.info.height, 150)
        self.assertEqual(grid.info.width * grid.info.height, len(grid.data))

        occupied = sum(value >= 65 for value in grid.data)
        free = sum(0 <= value <= 25 for value in grid.data)
        unknown = sum(value < 0 for value in grid.data)
        # A stationary 360-degree baseline sees several interior wall segments.
        # The first measured map contained 56 occupied cells; 40 rejects an
        # empty/degenerate map while leaving margin for scan noise.
        self.assertGreaterEqual(occupied, 40)
        # This entry-point test freezes an initial local-map baseline. Full
        # coverage is validated when the saved map artifact is introduced.
        self.assertGreaterEqual(free, 1000)
        self.assertGreater(unknown, 0)
        self.assertGreaterEqual((occupied + free) / len(grid.data), 0.02)

        publishers = self.node.get_publishers_info_by_topic('/map')
        self.assertEqual(1, len(publishers))
        self.assertEqual(ReliabilityPolicy.RELIABLE,
                         publishers[0].qos_profile.reliability)
        self.assertEqual(DurabilityPolicy.TRANSIENT_LOCAL,
                         publishers[0].qos_profile.durability)

        save_client = self.node.create_client(
            SaveMap, '/map_saver/save_map')
        self.assertTrue(save_client.wait_for_service(timeout_sec=10.0))
        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = str(SAVE_PREFIX)
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.19
        request.occupied_thresh = 0.65
        future = save_client.call_async(request)
        self.assertTrue(self.spin_until(future.done, 10.0))
        self.assertTrue(future.result().result)
        self.assertTrue(Path(str(SAVE_PREFIX) + '.yaml').is_file())
        self.assertTrue(Path(str(SAVE_PREFIX) + '.pgm').is_file())
        saved = yaml.safe_load(
            Path(str(SAVE_PREFIX) + '.yaml').read_text())
        self.assertEqual('m5_baseline.pgm', saved['image'])
        self.assertEqual('trinary', saved['mode'])
        self.assertAlmostEqual(grid.info.resolution, saved['resolution'])
        self.assertAlmostEqual(
            grid.info.origin.position.x, saved['origin'][0], delta=0.005)
        self.assertAlmostEqual(
            grid.info.origin.position.y, saved['origin'][1], delta=0.005)
        pgm_header = Path(str(SAVE_PREFIX) + '.pgm').read_bytes()[:80]
        dimensions = f'{grid.info.width} {grid.info.height}'.encode()
        self.assertIn(dimensions, pgm_header)
        self.node.destroy_client(save_client)
