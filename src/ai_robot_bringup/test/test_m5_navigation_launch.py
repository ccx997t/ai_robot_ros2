from collections import deque
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 10)
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
        cls.nav_commands = deque(maxlen=1000)
        cls.safe_commands = deque(maxlen=1000)
        cls.odometry = deque(maxlen=1000)
        cls.safety_diagnostics = deque(maxlen=100)
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
        cls.subscriptions = [
            cls.node.create_subscription(
                Twist, '/cmd_vel', cls.receive_nav_command, 10),
            cls.node.create_subscription(
                Twist, '/base_controller/cmd_vel_unstamped',
                cls.receive_safe_command, 10),
            cls.node.create_subscription(
                Odometry, '/odom', cls.odometry.append, 10),
            cls.node.create_subscription(
                DiagnosticArray, '/diagnostics',
                cls.receive_diagnostics, 10),
        ]
        cls.navigation = ActionClient(
            cls.node, NavigateToPose, '/navigate_to_pose')

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

    @classmethod
    def receive_nav_command(cls, message):
        cls.nav_commands.append((time.monotonic(), message))

    @classmethod
    def receive_safe_command(cls, message):
        cls.safe_commands.append((time.monotonic(), message))

    @classmethod
    def receive_diagnostics(cls, message):
        for status in message.status:
            if status.name == 'base/cmd_vel_safety':
                cls.safety_diagnostics.append((time.monotonic(), status))

    @classmethod
    def send_goal(cls, x, y):
        assert cls.navigation.wait_for_server(timeout_sec=60.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = cls.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        future = cls.navigation.send_goal_async(goal)
        assert cls.spin_until(future.done, 10.0)
        handle = future.result()
        assert handle.accepted
        return handle

    @classmethod
    def switch_base_controller(cls, activate):
        client = cls.node.create_client(
            SwitchController, '/controller_manager/switch_controller')
        assert client.wait_for_service(timeout_sec=10.0)
        request = SwitchController.Request()
        if activate:
            request.activate_controllers = ['base_controller']
        else:
            request.deactivate_controllers = ['base_controller']
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        future = client.call_async(request)
        assert cls.spin_until(future.done, 10.0)
        assert future.result().ok
        cls.node.destroy_client(client)

    @classmethod
    def controller_state(cls):
        client = cls.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        assert client.wait_for_service(timeout_sec=10.0)
        future = client.call_async(ListControllers.Request())
        assert cls.spin_until(future.done, 10.0)
        states = {item.name: item.state for item in future.result().controller}
        cls.node.destroy_client(client)
        return states.get('base_controller')

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

    def test_z_navigation_limit_timeout_and_base_disconnect(self):
        for node_name in self.MANAGED_NODES:
            self.wait_for_active(node_name)
        self.assertEqual('active', self.controller_state())

        # A real NavigateToPose goal must produce motion through both sides of
        # the safety boundary, without exceeding Nav2 or base safety limits.
        self.nav_commands.clear()
        self.safe_commands.clear()
        goal = self.send_goal(-1.0, 0.0)

        def moving(item):
            return (abs(item[1].linear.x) > 0.02
                    or abs(item[1].angular.z) > 0.05)
        self.assertTrue(self.spin_until(
            lambda: any(moving(item) for item in self.nav_commands)
            and any(moving(item) for item in self.safe_commands), 20.0),
            'Nav2 did not produce a non-zero command through the safety chain')
        for _, message in self.nav_commands:
            self.assertLessEqual(abs(message.linear.x), 0.250001)
            self.assertLessEqual(abs(message.angular.z), 0.700001)
        for _, message in self.safe_commands:
            self.assertLessEqual(abs(message.linear.x), 0.300001)
            self.assertLessEqual(abs(message.angular.z), 0.800001)

        # Cancelling the active navigation stops its command stream. The base
        # watchdog must independently publish a stop after its 0.5 s timeout.
        self.safety_diagnostics.clear()
        cancel_started = time.monotonic()
        cancel_future = goal.cancel_goal_async()
        self.assertTrue(self.spin_until(cancel_future.done, 10.0))
        self.assertTrue(self.spin_until(
            lambda: any(
                status.message == 'command timeout; stop sent'
                for _, status in self.safety_diagnostics), 2.0))
        timeout_time = next(
            stamp for stamp, status in self.safety_diagnostics
            if status.message == 'command timeout; stop sent')
        self.assertGreaterEqual(timeout_time - cancel_started, 0.40)
        self.assertLessEqual(timeout_time - cancel_started, 1.20)
        self.assertTrue(self.spin_until(
            lambda: any(
                stamp >= timeout_time and message.linear.x == 0.0
                and message.angular.z == 0.0
                for stamp, message in self.safe_commands),
            1.0))

        # Deactivating the ros2_control base controller simulates a lost lower
        # layer. Commands may reach its input, but an inactive controller must
        # not actuate the robot. Reactivation proves the fault is recoverable.
        second_goal = self.send_goal(-1.8, 0.0)
        command_start = len(self.safe_commands)
        self.assertTrue(self.spin_until(
            lambda: any(moving(item)
                        for item in list(self.safe_commands)[command_start:]),
            15.0))
        self.switch_base_controller(False)
        self.assertEqual('inactive', self.controller_state())
        self.assertTrue(self.spin_until(lambda: bool(self.odometry), 5.0))
        start = self.odometry[-1].pose.pose.position
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        end = self.odometry[-1].pose.pose.position
        displacement = ((end.x - start.x) ** 2 + (end.y - start.y) ** 2) ** 0.5
        self.assertLess(displacement, 0.03)
        self.switch_base_controller(True)
        self.assertEqual('active', self.controller_state())
        cancel_future = second_goal.cancel_goal_async()
        self.assertTrue(self.spin_until(cancel_future.done, 10.0))
