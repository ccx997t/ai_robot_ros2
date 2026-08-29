from math import atan2, cos, hypot, pi, sin
import os
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import yaml


os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m5_global_localization_{os.getpid()}'


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch', 'navigation.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'mode': 'sim',
                'set_initial_pose': 'false',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM5GlobalLocalization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m5_global_localization_test')
        cls.poses = []
        cls.odometry = []
        cls.safe_commands = []
        cls.pose_sub = cls.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', cls.poses.append, 10)
        cls.odom_sub = cls.node.create_subscription(
            Odometry, '/odom', cls.odometry.append, 50)
        cls.safe_sub = cls.node.create_subscription(
            Twist, '/base_controller/cmd_vel_unstamped',
            cls.safe_commands.append, 20)
        cls.initial_pose_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        cls.command_pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.global_localization = cls.node.create_client(
            Empty, '/reinitialize_global_localization')
        cls.navigator = ActionClient(
            cls.node, NavigateToPose, '/navigate_to_pose')
        cls.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        scenario_path = os.path.join(
            get_package_share_directory('ai_robot_sim'),
            'config', 'm5_scenario.yaml')
        with open(scenario_path, encoding='utf-8') as stream:
            cls.goals = yaml.safe_load(stream)['scenario']['goals']

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
    def publish_false_pose(cls):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.pose.pose.position.x = 4.0
        message.pose.pose.position.y = 3.0
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.03
        for _ in range(5):
            cls.initial_pose_pub.publish(message)
            rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def rotate_for_global_observation(cls, timeout=60.0):
        command = Twist()
        command.angular.z = 0.35
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.command_pub.publish(command)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if cls.poses:
                pose = cls.poses[-1].pose.pose.position
                covariance = cls.poses[-1].pose.covariance
                if hypot(pose.x, pose.y) < 0.55 and covariance[0] < 0.5:
                    break
        cls.command_pub.publish(Twist())

    @classmethod
    def navigate(cls, target, timeout=100.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = cls.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target['x'])
        goal.pose.pose.position.y = float(target['y'])
        half_yaw = float(target['yaw']) / 2.0
        goal.pose.pose.orientation.z = sin(half_yaw)
        goal.pose.pose.orientation.w = cos(half_yaw)
        sent = cls.navigator.send_goal_async(goal)
        assert cls.spin_until(sent.done, 10.0)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        if not cls.spin_until(result.done, timeout):
            cancel = handle.cancel_goal_async()
            cls.spin_until(cancel.done, 5.0)
            raise AssertionError(f'long-route goal timed out: {target}')
        return result.result().status

    @staticmethod
    def yaw_error(orientation, target_yaw):
        actual = atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z))
        difference = actual - float(target_yaw)
        return abs((difference + pi) % (2.0 * pi) - pi)

    def test_global_recovery_and_long_route_drift(self):
        self.assertTrue(self.global_localization.wait_for_service(60.0))
        self.assertTrue(self.navigator.wait_for_server(90.0))
        self.assertTrue(self.spin_until(lambda: self.odometry, 45.0))

        # Inject a confident but false localization estimate. Recovery then
        # uses AMCL's public global-localization service and lidar motion; no
        # correct pose is supplied through /initialpose.
        self.publish_false_pose()
        self.assertTrue(self.spin_until(
            lambda: self.poses and hypot(
                self.poses[-1].pose.pose.position.x,
                self.poses[-1].pose.pose.position.y) > 2.0,
            10.0), 'false kidnapped estimate was not accepted')
        recovery_started = time.monotonic()
        recovered_globally = False
        attempts = 0
        for attempts in range(1, 4):
            future = self.global_localization.call_async(Empty.Request())
            self.assertTrue(self.spin_until(future.done, 10.0))
            self.rotate_for_global_observation()
            if self.poses:
                candidate = self.poses[-1].pose.pose.position
                if hypot(candidate.x, candidate.y) < 0.55:
                    recovered_globally = True
                    break
        self.assertTrue(
            recovered_globally,
            'AMCL global recovery failed after three observation sweeps')
        self.assertTrue(self.poses, 'AMCL did not publish after global reset')
        recovered = self.poses[-1]
        recovery_error = hypot(
            recovered.pose.pose.position.x,
            recovered.pose.pose.position.y)
        self.assertLess(recovery_error, 0.55)
        recovery_seconds = time.monotonic() - recovery_started

        # Traverse three distant rooms. Arrival residual is measured from the
        # AMCL estimate at each successful action result, while odometry gives
        # an independent lower bound for route length.
        odom_start = len(self.odometry)
        results = []
        residuals = []
        yaw_residuals = []
        segment_seconds = []
        started = time.monotonic()
        route = [
            self.goals['east_room'],
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            self.goals['west_bay'],
        ]
        for target in route:
            segment_started = time.monotonic()
            results.append(self.navigate(target))
            segment_seconds.append(time.monotonic() - segment_started)
            self.assertTrue(self.spin_until(
                lambda: self.tf_buffer.can_transform(
                    'map', 'base_link', Time()), 3.0))
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            residuals.append(hypot(
                translation.x - float(target['x']),
                translation.y - float(target['y'])))
            yaw_residuals.append(self.yaw_error(
                rotation, target['yaw']))
        route_seconds = time.monotonic() - started
        self.assertEqual([GoalStatus.STATUS_SUCCEEDED] * 3, results)
        self.assertLess(max(residuals), 0.55)
        self.assertLess(max(yaw_residuals), 0.50)
        self.assertLess(max(segment_seconds), 100.0)
        self.assertLess(route_seconds, 240.0)

        samples = self.odometry[odom_start:]
        route_length = sum(hypot(
            second.pose.pose.position.x - first.pose.pose.position.x,
            second.pose.pose.position.y - first.pose.pose.position.y)
            for first, second in zip(samples, samples[1:]))
        self.assertGreater(route_length, 12.0)
        self.assertTrue(self.safe_commands)
        self.assertLessEqual(max(
            hypot(command.linear.x, command.linear.y)
            for command in self.safe_commands), 0.2501)
        self.node.get_logger().info(
            'M5 metrics: recovery=%.3fs attempts=%d recovery_error=%.3fm '
            'route=%.3fs segments=%s length=%.3fm '
            'max_arrival_residual=%.3fm max_yaw_residual=%.3frad' % (
                recovery_seconds, attempts, recovery_error, route_seconds,
                ','.join(f'{value:.3f}' for value in segment_seconds),
                route_length, max(residuals), max(yaw_residuals)))
