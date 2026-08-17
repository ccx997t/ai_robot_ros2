import math
import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m2_test_{os.getpid()}'

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    sim_launch = os.path.join(
        get_package_share_directory('ai_robot_sim'),
        'launch',
        'sim_base.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        Node(
            package='ai_robot_base',
            executable='cmd_vel_safety_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ai_robot_base',
            executable='odom_contract_relay',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSimBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sim_base_acceptance_test')
        cls.commands = []
        cls.odometry = []
        cls.command_pub = cls.node.create_publisher(Twist, '/cmd_vel', 1)
        cls.command_sub = cls.node.create_subscription(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            lambda message: cls.commands.append(message),
            10,
        )
        cls.odom_sub = cls.node.create_subscription(
            Odometry,
            '/odom',
            lambda message: cls.odometry.append(message),
            10,
        )
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)

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
    def publish_for(cls, linear_x=0.0, angular_z=0.0, seconds=1.0):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            cls.command_pub.publish(command)
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    def test_controller_tf_motion_and_safety_contract(self):
        controller_client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(controller_client.wait_for_service(timeout_sec=30.0))
        deadline = time.monotonic() + 15.0
        states = {}
        while time.monotonic() < deadline:
            future = controller_client.call_async(ListControllers.Request())
            if self.spin_until(future.done, 1.0):
                states = {
                    item.name: item.state for item in future.result().controller
                }
                if states.get('joint_state_broadcaster') == 'active' and \
                        states.get('base_controller') == 'active':
                    break
            time.sleep(0.1)
        self.assertEqual('active', states.get('joint_state_broadcaster'))
        self.assertEqual('active', states.get('base_controller'))

        self.assertTrue(self.spin_until(lambda: bool(self.odometry), 15.0))
        odom = self.odometry[-1]
        self.assertEqual('odom', odom.header.frame_id)
        self.assertEqual('base_link', odom.child_frame_id)
        publishers = self.node.get_publishers_info_by_topic('/odom')
        self.assertEqual(1, len(publishers))
        self.assertEqual(ReliabilityPolicy.RELIABLE,
                         publishers[0].qos_profile.reliability)
        self.assertEqual(DurabilityPolicy.VOLATILE,
                         publishers[0].qos_profile.durability)

        def required_tf_available():
            try:
                self.tf_buffer.lookup_transform(
                    'odom', 'sensor_link', Time(), timeout=Duration(seconds=0.1))
                return True
            except Exception:
                return False

        self.assertTrue(self.spin_until(required_tf_available, 15.0))

        start_x = self.odometry[-1].pose.pose.position.x
        self.publish_for(linear_x=0.2, seconds=1.5)
        forward_x = self.odometry[-1].pose.pose.position.x
        self.assertGreater(forward_x - start_x, 0.10)

        self.publish_for(linear_x=-0.2, seconds=1.5)
        reverse_x = self.odometry[-1].pose.pose.position.x
        self.assertLess(reverse_x, forward_x - 0.10)

        before = self.odometry[-1].pose.pose.orientation
        before_yaw = math.atan2(
            2.0 * (before.w * before.z + before.x * before.y),
            1.0 - 2.0 * (before.y * before.y + before.z * before.z),
        )
        self.publish_for(angular_z=0.5, seconds=1.5)
        after = self.odometry[-1].pose.pose.orientation
        after_yaw = math.atan2(
            2.0 * (after.w * after.z + after.x * after.y),
            1.0 - 2.0 * (after.y * after.y + after.z * after.z),
        )
        yaw_delta = math.atan2(
            math.sin(after_yaw - before_yaw), math.cos(after_yaw - before_yaw))
        self.assertGreater(yaw_delta, 0.20)

        self.commands.clear()
        self.publish_for(linear_x=1.0, angular_z=-2.0, seconds=0.2)
        limited = next(
            (item for item in reversed(self.commands)
             if abs(item.linear.x) > 0.0 or abs(item.angular.z) > 0.0),
            None,
        )
        self.assertIsNotNone(limited)
        self.assertAlmostEqual(0.30, limited.linear.x, places=6)
        self.assertAlmostEqual(-0.80, limited.angular.z, places=6)

        self.commands.clear()
        self.assertTrue(self.spin_until(
            lambda: any(
                item.linear.x == 0.0 and item.angular.z == 0.0
                for item in self.commands),
            1.0,
        ))
        self.assertTrue(self.spin_until(
            lambda: abs(self.odometry[-1].twist.twist.linear.x) < 0.01
            and abs(self.odometry[-1].twist.twist.angular.z) < 0.01,
            3.0,
        ))
