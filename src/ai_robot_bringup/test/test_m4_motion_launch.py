from collections import deque
import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(120 + os.getpid() % 80)
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
from sensor_msgs.msg import JointState
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
