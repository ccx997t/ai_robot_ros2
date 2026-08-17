from collections import deque
import math
import os
import time
import unittest

# Every invocation gets a fresh DDS / Gazebo discovery scope.  This prevents
# recently terminated launch tests from being observed during rapid reruns.
os.environ['ROS_DOMAIN_ID'] = str(110 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m3_sensor_test_{os.getpid()}'

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_sim'), 'launch', 'sensors.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        Node(
            package='ai_robot_base', executable='cmd_vel_safety_node',
            parameters=[{'use_sim_time': True}], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSensorInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sensor_interfaces_acceptance_test')
        cls.messages = {
            name: deque(maxlen=300)
            for name in ('scan', 'image', 'camera_info', 'imu', 'encoder')
        }
        cls.diagnostics = {}
        contracts = [
            ('scan', LaserScan, '/scan'),
            ('image', Image, '/camera/image_raw'),
            ('camera_info', CameraInfo, '/camera/camera_info'),
            ('imu', Imu, '/imu/data'),
            ('encoder', JointState, '/joint_states'),
        ]
        cls.subscriptions = [
            cls.node.create_subscription(
                message_type, topic,
                lambda message, key=key: cls.messages[key].append(
                    (time.monotonic(), message)),
                qos_profile_sensor_data)
            for key, message_type, topic in contracts
        ]
        cls.subscriptions.append(cls.node.create_subscription(
            DiagnosticArray, '/diagnostics', cls.receive_diagnostics, 10))
        cls.command_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

    @classmethod
    def receive_diagnostics(cls, message):
        for status in message.status:
            cls.diagnostics[status.name] = status

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
    def command_for(cls, linear_x=0.0, angular_z=0.0, seconds=1.0):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            cls.command_pub.publish(command)
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    def test_types_qos_frequency_frame_content_tf_and_diagnostics(self):
        expected_counts = {'scan': 20, 'image': 25, 'camera_info': 25,
                           'imu': 150, 'encoder': 150}
        self.assertTrue(self.spin_until(
            lambda: all(len(self.messages[key]) >= count
                        for key, count in expected_counts.items()), 45.0))

        topics = {
            'scan': ('/scan', LaserScan, ReliabilityPolicy.BEST_EFFORT,
                     DurabilityPolicy.VOLATILE),
            'image': ('/camera/image_raw', Image, ReliabilityPolicy.BEST_EFFORT,
                      DurabilityPolicy.VOLATILE),
            'camera_info': ('/camera/camera_info', CameraInfo,
                            ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
            'imu': ('/imu/data', Imu, ReliabilityPolicy.BEST_EFFORT,
                    DurabilityPolicy.VOLATILE),
            'encoder': ('/joint_states', JointState, ReliabilityPolicy.RELIABLE,
                        DurabilityPolicy.TRANSIENT_LOCAL),
        }
        for key, (topic, message_type, reliability, durability) in topics.items():
            message = self.messages[key][-1][1]
            self.assertIsInstance(message, message_type)
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual(1, len(publishers), topic)
            self.assertEqual(reliability, publishers[0].qos_profile.reliability, topic)
            self.assertEqual(durability, publishers[0].qos_profile.durability, topic)

        expected_rates = {'scan': 10.0, 'image': 15.0, 'camera_info': 15.0,
                          'imu': 100.0, 'encoder': 100.0}
        for key, expected in expected_rates.items():
            samples = self.messages[key]
            measured = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
            self.assertGreater(measured, expected * 0.7, key)
            self.assertLess(measured, expected * 1.3, key)

        scan = self.messages['scan'][-1][1]
        self.assertEqual('laser_link', scan.header.frame_id)
        self.assertGreater(len(scan.ranges), 0)
        self.assertLess(scan.angle_min, scan.angle_max)
        self.assertLess(scan.range_min, scan.range_max)

        image = self.messages['image'][-1][1]
        self.assertEqual('camera_optical_link', image.header.frame_id)
        self.assertEqual((320, 240, 'rgb8'), (image.width, image.height, image.encoding))
        self.assertEqual(image.step * image.height, len(image.data))

        info = self.messages['camera_info'][-1][1]
        self.assertEqual('camera_optical_link', info.header.frame_id)
        self.assertEqual((320, 240), (info.width, info.height))
        self.assertEqual(9, len(info.k))
        self.assertGreater(info.k[0], 0.0)

        imu = self.messages['imu'][-1][1]
        self.assertEqual('imu_link', imu.header.frame_id)
        quaternion = [imu.orientation.x, imu.orientation.y,
                      imu.orientation.z, imu.orientation.w]
        self.assertTrue(all(math.isfinite(value) for value in quaternion))
        self.assertGreater(sum(value * value for value in quaternion), 0.0)
        self.assertTrue(math.isfinite(imu.linear_acceleration.z))

        encoder = self.messages['encoder'][-1][1]
        self.assertEqual('base_link', encoder.header.frame_id)
        self.assertEqual({'left_wheel_joint', 'right_wheel_joint'}, set(encoder.name))
        self.assertEqual(len(encoder.name), len(encoder.position))
        self.assertEqual(len(encoder.name), len(encoder.velocity))

        for frame in ('laser_link', 'camera_optical_link', 'imu_link'):
            self.assertTrue(self.spin_until(
                lambda frame=frame: self.tf_buffer.can_transform(
                    'base_link', frame, Time()), 10.0), frame)

        names = {'sensors/lidar', 'sensors/camera_image', 'sensors/camera_info',
                 'sensors/imu', 'sensors/encoder'}
        self.assertTrue(self.spin_until(
            lambda: names <= self.diagnostics.keys()
            and all(self.diagnostics[name].level == DiagnosticStatus.OK
                    for name in names), 10.0))
        for name in names:
            self.assertEqual(DiagnosticStatus.OK, self.diagnostics[name].level, name)

    def test_lidar_noise_imu_motion_and_encoder_accumulation(self):
        def finite_center_samples():
            values = []
            for _, scan in self.messages['scan']:
                center_index = round(-scan.angle_min / scan.angle_increment)
                value = scan.ranges[center_index]
                if math.isfinite(value):
                    values.append(value)
            return values

        self.assertTrue(self.spin_until(
            lambda: len(finite_center_samples()) >= 40
            and len(self.messages['imu']) >= 100
            and len(self.messages['encoder']) >= 100, 45.0))

        center_ranges = finite_center_samples()[-40:]
        mean_range = sum(center_ranges) / len(center_ranges)
        variance = sum((value - mean_range) ** 2 for value in center_ranges) / len(center_ranges)
        noise_stddev = math.sqrt(variance)
        self.assertGreater(mean_range, 1.5)
        self.assertLess(mean_range, 2.0)
        self.assertGreater(noise_stddev, 0.001)
        self.assertLess(noise_stddev, 0.05)

        def wheel_positions():
            message = self.messages['encoder'][-1][1]
            values = dict(zip(message.name, message.position))
            return values['left_wheel_joint'], values['right_wheel_joint']

        start_left, start_right = wheel_positions()
        self.command_for(linear_x=0.2, seconds=1.5)
        forward_left, forward_right = wheel_positions()
        self.assertGreater(forward_left - start_left, 1.0)
        self.assertGreater(forward_right - start_right, 1.0)

        self.command_for(linear_x=-0.2, seconds=1.5)
        reverse_left, reverse_right = wheel_positions()
        self.assertLess(reverse_left - forward_left, -1.0)
        self.assertLess(reverse_right - forward_right, -1.0)

        turn_started = time.monotonic()
        turn_left, turn_right = wheel_positions()
        self.command_for(angular_z=0.5, seconds=1.5)
        after_left, after_right = wheel_positions()
        self.assertLess(after_left - turn_left, -0.5)
        self.assertGreater(after_right - turn_right, 0.5)
        motion_imu = [message for received, message in self.messages['imu']
                      if received >= turn_started]
        self.assertTrue(any(message.angular_velocity.z > 0.2 for message in motion_imu))

        self.command_for(seconds=0.2)
        final_left, final_right = wheel_positions()
        self.assertGreater(abs(final_left - start_left), 0.1)
        self.assertGreater(abs(final_right - start_right), 0.1)
