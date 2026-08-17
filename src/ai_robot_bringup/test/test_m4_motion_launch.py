from collections import deque
import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m4_motion_test_{os.getpid()}'

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch',
        'm4_bringup.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'mode': 'sim'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM4MotionIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m4_motion_integration_test')
        cls.odometry = deque(maxlen=500)
        cls.wheel_odometry = deque(maxlen=500)
        cls.joints = deque(maxlen=500)
        cls.scans = deque(maxlen=100)
        cls.raw_images = deque(maxlen=100)
        cls.mono_images = deque(maxlen=100)
        cls.imu_messages = deque(maxlen=300)
        cls.clocks = deque(maxlen=300)
        cls.tf_pairs = set()
        static_tf_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.command_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.subscriptions = [
            cls.node.create_subscription(
                Odometry, '/odom', cls.odometry.append, 10),
            cls.node.create_subscription(
                Odometry, '/wheel/odom', cls.wheel_odometry.append, 10),
            cls.node.create_subscription(
                JointState, '/joint_states', cls.joints.append,
                qos_profile_sensor_data),
            cls.node.create_subscription(
                LaserScan, '/scan', cls.scans.append, qos_profile_sensor_data),
            cls.node.create_subscription(
                Image, '/camera/image_raw',
                lambda message: cls.raw_images.append((time.monotonic(), message)),
                qos_profile_sensor_data),
            cls.node.create_subscription(
                Image, '/camera/image_mono',
                lambda message: cls.mono_images.append((time.monotonic(), message)),
                qos_profile_sensor_data),
            cls.node.create_subscription(
                Imu, '/imu/data', cls.imu_messages.append,
                qos_profile_sensor_data),
            cls.node.create_subscription(
                Clock, '/clock', cls.clocks.append, qos_profile_sensor_data),
            cls.node.create_subscription(TFMessage, '/tf', cls.receive_tf, 100),
            cls.node.create_subscription(
                TFMessage, '/tf_static', cls.receive_tf, static_tf_qos),
        ]

    @classmethod
    def receive_tf(cls, message):
        cls.tf_pairs.update(
            (transform.header.frame_id, transform.child_frame_id)
            for transform in message.transforms
        )

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
    def command_for(cls, linear_x, seconds):
        command = Twist()
        command.linear.x = linear_x
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            cls.command_pub.publish(command)
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @staticmethod
    def wheel_positions(message):
        positions = dict(zip(message.name, message.position))
        return positions['left_wheel_joint'], positions['right_wheel_joint']

    def test_a_lidar_tf_and_fixed_obstacle_geometry(self):
        self.assertTrue(
            self.spin_until(lambda: len(self.scans) >= 20, 45.0),
            f'laser scan count={len(self.scans)}')
        self.assertTrue(self.spin_until(
            lambda: {('base_link', 'sensor_link'), ('sensor_link', 'laser_link')}
            <= self.tf_pairs,
            10.0,
        ))

        distances = []
        bearings = []
        left_edges = []
        right_edges = []
        for scan in list(self.scans)[-20:]:
            front_returns = [
                (scan.angle_min + index * scan.angle_increment, distance)
                for index, distance in enumerate(scan.ranges)
                if -0.35 <= scan.angle_min + index * scan.angle_increment <= 0.35
                and math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
            ]
            self.assertTrue(front_returns)
            bearing, distance = min(front_returns, key=lambda item: item[1])
            target_returns = [item for item in front_returns if item[1] < 1.80]
            self.assertTrue(target_returns)
            distances.append(distance)
            bearings.append(bearing)
            right_edges.append(min(item[0] for item in target_returns))
            left_edges.append(max(item[0] for item in target_returns))

        distances.sort()
        bearings.sort()
        right_edges.sort()
        left_edges.sort()
        median = len(distances) // 2
        # World contract: a 0.5 m box centered at x=2.0 m. The lidar is
        # mounted at base x=0.1 m, so its front face is about 1.65 m away.
        self.assertGreater(distances[median], 1.55)
        self.assertLess(distances[median], 1.75)
        self.assertLess(abs(bearings[median]), 0.05)
        self.assertLess(right_edges[median], -0.10)
        self.assertGreater(left_edges[median], 0.10)

    def test_b_camera_minimal_processing_contract(self):
        self.assertTrue(self.spin_until(
            lambda: len(self.raw_images) >= 25 and len(self.mono_images) >= 25,
            45.0,
        ))
        output = self.mono_images[-1][1]
        self.assertEqual('camera_optical_link', output.header.frame_id)
        self.assertEqual((320, 240, 'mono8', 320),
                         (output.width, output.height, output.encoding, output.step))
        self.assertEqual(output.width * output.height, len(output.data))

        samples = list(self.mono_images)[-25:]
        measured_rate = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
        self.assertGreater(measured_rate, 10.5)
        self.assertLess(measured_rate, 19.5)

        raw_arrivals = {
            (message.header.stamp.sec, message.header.stamp.nanosec): received
            for received, message in self.raw_images
        }
        latencies_ms = [
            abs(received - raw_arrivals[
                (message.header.stamp.sec, message.header.stamp.nanosec)]) * 1000.0
            for received, message in self.mono_images
            if (message.header.stamp.sec, message.header.stamp.nanosec) in raw_arrivals
        ]
        self.assertGreaterEqual(len(latencies_ms), 20)
        latencies_ms.sort()
        p95_index = math.ceil(len(latencies_ms) * 0.95) - 1
        self.assertLess(latencies_ms[p95_index], 100.0)

        publishers = self.node.get_publishers_info_by_topic('/camera/image_mono')
        self.assertEqual(1, len(publishers))
        self.assertEqual(ReliabilityPolicy.BEST_EFFORT,
                         publishers[0].qos_profile.reliability)
        self.assertEqual(DurabilityPolicy.VOLATILE,
                         publishers[0].qos_profile.durability)

    def test_b_time_qos_tf_and_startup_contracts(self):
        self.assertTrue(self.spin_until(
            lambda: len(self.clocks) >= 20 and len(self.imu_messages) >= 50
            and len(self.odometry) >= 10 and len(self.scans) >= 10
            and len(self.raw_images) >= 10 and len(self.mono_images) >= 10,
            45.0,
        ))

        required_tf = {
            ('odom', 'base_link'),
            ('base_link', 'sensor_link'),
            ('sensor_link', 'laser_link'),
            ('sensor_link', 'camera_link'),
            ('camera_link', 'camera_optical_link'),
            ('sensor_link', 'imu_link'),
        }
        self.assertTrue(self.spin_until(
            lambda: required_tf <= self.tf_pairs, 10.0))

        def stamp_ns(message):
            return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

        streams = {
            '/odom': list(self.odometry)[-10:],
            '/scan': list(self.scans)[-10:],
            '/imu/data': list(self.imu_messages)[-20:],
            '/camera/image_raw': [item[1] for item in list(self.raw_images)[-10:]],
            '/camera/image_mono': [item[1] for item in list(self.mono_images)[-10:]],
        }
        clock_ns = (self.clocks[-1].clock.sec * 1_000_000_000
                    + self.clocks[-1].clock.nanosec)
        self.assertGreater(clock_ns, 0)
        for topic, messages in streams.items():
            stamps = [stamp_ns(message) for message in messages]
            self.assertTrue(all(stamp > 0 for stamp in stamps), topic)
            self.assertEqual(stamps, sorted(stamps), topic)
            self.assertLessEqual(stamps[-1], clock_ns + 100_000_000, topic)
            self.assertLess(clock_ns - stamps[-1], 500_000_000, topic)

        qos_contracts = {
            '/odom': (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
            '/scan': (ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
            '/imu/data': (ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
            '/camera/image_raw': (
                ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
            '/camera/image_mono': (
                ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
        }
        for topic, (reliability, durability) in qos_contracts.items():
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual(1, len(publishers), topic)
            self.assertEqual(reliability, publishers[0].qos_profile.reliability, topic)
            self.assertEqual(durability, publishers[0].qos_profile.durability, topic)

    def test_base_encoder_odometry_and_tf_are_consistent(self):
        # Wait for controller outputs before querying controller_manager. Calling
        # list_controllers while both spawners are configuring can trigger an
        # upstream Humble/gz_ros2_control service-response race.
        self.assertTrue(
            self.spin_until(lambda: len(self.wheel_odometry) >= 20, 45.0),
            f'wheel odometry count={len(self.wheel_odometry)}')
        self.assertTrue(
            self.spin_until(lambda: len(self.odometry) >= 20, 10.0),
            f'fused odometry count={len(self.odometry)}')
        self.assertTrue(
            self.spin_until(lambda: len(self.joints) >= 20, 10.0),
            f'joint state count={len(self.joints)}')

        controller_client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(controller_client.wait_for_service(timeout_sec=5.0))
        future = controller_client.call_async(ListControllers.Request())
        self.assertTrue(self.spin_until(future.done, 5.0))
        states = {item.name: item.state for item in future.result().controller}
        self.assertEqual('active', states.get('joint_state_broadcaster'))
        self.assertEqual('active', states.get('base_controller'))
        self.assertTrue(self.spin_until(
            lambda: {('odom', 'base_link'), ('base_link', 'sensor_link')}
            <= self.tf_pairs,
            15.0,
        ))

        start_odom = self.odometry[-1].pose.pose.position.x
        start_wheel_odom = self.wheel_odometry[-1].pose.pose.position.x
        start_left, start_right = self.wheel_positions(self.joints[-1])
        self.command_for(0.2, 1.5)
        forward_odom = self.odometry[-1].pose.pose.position.x
        forward_wheel_odom = self.wheel_odometry[-1].pose.pose.position.x
        forward_left, forward_right = self.wheel_positions(self.joints[-1])

        odom_delta = forward_odom - start_odom
        wheel_distance = (
            (forward_left - start_left) + (forward_right - start_right)
        ) * 0.075 / 2.0
        self.assertGreater(odom_delta, 0.10)
        self.assertGreater(forward_wheel_odom - start_wheel_odom, 0.10)
        self.assertGreater(forward_left - start_left, 1.0)
        self.assertGreater(forward_right - start_right, 1.0)
        self.assertTrue(math.isclose(odom_delta, wheel_distance, abs_tol=0.04))
        self.assertTrue(math.isclose(
            odom_delta, forward_wheel_odom - start_wheel_odom, abs_tol=0.04))

        fused = self.odometry[-1]
        self.assertEqual('odom', fused.header.frame_id)
        self.assertEqual('base_link', fused.child_frame_id)
        self.assertGreater(fused.pose.covariance[0], 0.0)
        self.assertGreater(fused.twist.covariance[0], 0.0)
        self.assertEqual(1, len(self.node.get_publishers_info_by_topic('/odom')))

        self.command_for(-0.2, 1.5)
        reverse_odom = self.odometry[-1].pose.pose.position.x
        reverse_left, reverse_right = self.wheel_positions(self.joints[-1])
        self.assertLess(reverse_odom, forward_odom - 0.10)
        self.assertLess(reverse_left, forward_left - 1.0)
        self.assertLess(reverse_right, forward_right - 1.0)

        self.command_for(0.0, 0.3)
        self.node.destroy_client(controller_client)
