from collections import deque
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 20)
os.environ['IGN_PARTITION'] = f'ai_robot_m5_mapping_test_{os.getpid()}'


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
