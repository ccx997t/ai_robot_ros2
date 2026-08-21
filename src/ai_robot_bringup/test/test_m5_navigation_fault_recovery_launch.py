import os
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


os.environ['ROS_DOMAIN_ID'] = str(170 + os.getpid() % 10)
os.environ['IGN_PARTITION'] = f'ai_robot_m5_fault_recovery_{os.getpid()}'


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_bringup'),
        'launch', 'navigation_fault_recovery.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'mode': 'sim'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM5NavigationFaultRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m5_navigation_fault_recovery_test')
        cls.diagnostics = {}
        cls.scans = []
        cls.nav_commands = []
        cls.safe_commands = []
        cls.subscriptions = [
            cls.node.create_subscription(
                DiagnosticArray, '/diagnostics', cls.receive_diagnostics, 10),
            cls.node.create_subscription(
                LaserScan, '/scan', cls.scans.append,
                qos_profile_sensor_data),
            cls.node.create_subscription(
                Twist, '/cmd_vel', cls.nav_commands.append, 10),
            cls.node.create_subscription(
                Twist, '/base_controller/cmd_vel_unstamped',
                cls.safe_commands.append, 10),
        ]
        cls.navigator = ActionClient(
            cls.node, NavigateToPose, '/navigate_to_pose')
        cls.scan_toggle = cls.node.create_client(
            SetBool, '/navigation_scan_fault/enable')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @classmethod
    def receive_diagnostics(cls, message):
        for status in message.status:
            cls.diagnostics[status.name] = status

    @classmethod
    def spin_until(cls, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def spin_for(cls, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def wait_active(cls, node_name):
        client = cls.node.create_client(GetState, f'/{node_name}/get_state')
        assert client.wait_for_service(timeout_sec=60.0)
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            future = client.call_async(GetState.Request())
            if cls.spin_until(future.done, 5.0):
                if future.result().current_state.id == 3:
                    cls.node.destroy_client(client)
                    return
        cls.node.destroy_client(client)
        raise AssertionError(f'{node_name} did not become active')

    @classmethod
    def send_goal(cls, x, y):
        assert cls.navigator.wait_for_server(timeout_sec=60.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = cls.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        future = cls.navigator.send_goal_async(goal)
        assert cls.spin_until(future.done, 10.0)
        handle = future.result()
        assert handle.accepted
        return handle

    @classmethod
    def wait_result(cls, handle, timeout):
        future = handle.get_result_async()
        assert cls.spin_until(future.done, timeout)
        return future.result().status

    @classmethod
    def toggle_fault(cls, enabled):
        assert cls.scan_toggle.wait_for_service(timeout_sec=10.0)
        future = cls.scan_toggle.call_async(SetBool.Request(data=enabled))
        assert cls.spin_until(future.done, 5.0)
        assert future.result().success

    @staticmethod
    def moving(message):
        return abs(message.linear.x) > 0.02 or abs(message.angular.z) > 0.05

    @staticmethod
    def stopped(message):
        return message.linear.x == 0.0 and message.angular.z == 0.0

    def test_cancel_scan_timeout_diagnostics_and_navigation_recovery(self):
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            self.wait_active(name)
        self.assertTrue(self.spin_until(
            lambda: len(self.scans) >= 5
            and self.diagnostics.get('sensors/navigation_lidar') is not None,
            30.0))

        # Cancel a real navigation action while it is producing motion.
        self.nav_commands.clear()
        active_goal = self.send_goal(-1.6, 0.0)
        self.assertTrue(self.spin_until(
            lambda: any(self.moving(item) for item in self.nav_commands),
            15.0))
        safe_start = len(self.safe_commands)
        cancel = active_goal.cancel_goal_async()
        self.assertTrue(self.spin_until(cancel.done, 10.0))
        self.assertEqual(
            GoalStatus.STATUS_CANCELED,
            self.wait_result(active_goal, 10.0))
        self.assertTrue(self.spin_until(
            lambda: any(self.stopped(item)
                        for item in self.safe_commands[safe_start:]),
            2.0), 'cancel did not propagate a zero command to the base')

        # Drop the public scan stream and require both injection and consumer
        # diagnostics to expose the fault.
        self.toggle_fault(True)
        self.assertTrue(self.spin_until(
            lambda: (
                self.diagnostics.get('fault_injection/scan') is not None
                and self.diagnostics['fault_injection/scan'].level
                == DiagnosticStatus.WARN),
            3.0))
        self.spin_for(0.4)
        scan_count = len(self.scans)
        self.spin_for(1.3)
        self.assertEqual(scan_count, len(self.scans))
        public_lidar = self.diagnostics.get('sensors/navigation_lidar')
        self.assertIsNotNone(public_lidar)
        self.assertEqual(DiagnosticStatus.STALE, public_lidar.level)
        self.assertEqual(
            'sensor data stale or not received', public_lidar.message)

        # Restore the relay, require diagnostics to clear, then prove Nav2 can
        # complete a fresh route in both directions.
        scans_before_restore = len(self.scans)
        self.toggle_fault(False)
        self.assertTrue(self.spin_until(
            lambda: len(self.scans) >= scans_before_restore + 5, 5.0))
        self.assertTrue(self.spin_until(
            lambda: (
                self.diagnostics.get('fault_injection/scan') is not None
                and self.diagnostics['fault_injection/scan'].level
                == DiagnosticStatus.OK
                and self.diagnostics.get('sensors/navigation_lidar')
                is not None
                and self.diagnostics['sensors/navigation_lidar'].level
                == DiagnosticStatus.OK),
            5.0))

        recovered_goal = self.send_goal(-0.6, 0.0)
        self.assertEqual(
            GoalStatus.STATUS_SUCCEEDED,
            self.wait_result(recovered_goal, 45.0))
        home_goal = self.send_goal(0.0, 0.0)
        self.assertEqual(
            GoalStatus.STATUS_SUCCEEDED,
            self.wait_result(home_goal, 45.0))
