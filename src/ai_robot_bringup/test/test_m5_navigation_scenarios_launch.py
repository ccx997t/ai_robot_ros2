from math import cos, hypot, sin
import os
import subprocess
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


os.environ.setdefault('ROS_DOMAIN_ID', str(160 + os.getpid() % 10))
os.environ.setdefault(
    'IGN_PARTITION', f'ai_robot_m5_scenarios_{os.getpid()}')


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


class TestM5NavigationScenarios(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('m5_navigation_scenarios_test')
        cls.odometry = []
        cls.global_costmaps = []
        cls.nav_commands = []
        cls.safe_commands = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.subscriptions = [
            cls.node.create_subscription(
                Odometry, '/odom', cls.odometry.append, 10),
            cls.node.create_subscription(
                OccupancyGrid, '/global_costmap/costmap',
                cls.global_costmaps.append, qos),
            cls.node.create_subscription(
                Twist, '/cmd_vel', cls.nav_commands.append, 10),
            cls.node.create_subscription(
                Twist, '/base_controller/cmd_vel_unstamped',
                cls.safe_commands.append, 10),
        ]
        cls.navigator = ActionClient(
            cls.node, NavigateToPose, '/navigate_to_pose')
        cls.planner = ActionClient(
            cls.node, ComputePathToPose, '/compute_path_to_pose')
        sim_share = get_package_share_directory('ai_robot_sim')
        scenario_file = os.path.join(
            sim_share, 'config', 'm5_scenario.yaml')
        with open(scenario_file, encoding='utf-8') as stream:
            scenario = yaml.safe_load(stream)['scenario']
            cls.scenario = scenario['local_acceptance']
        cls.blocker_file = os.path.join(
            sim_share, 'models', 'temporary_route_blocker', 'model.sdf')

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
    def pose(cls, target):
        message = NavigateToPose.Goal().pose
        message.header.frame_id = 'map'
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.pose.position.x = float(target['x'])
        message.pose.position.y = float(target['y'])
        half_yaw = float(target.get('yaw', 0.0)) / 2.0
        message.pose.orientation.z = sin(half_yaw)
        message.pose.orientation.w = cos(half_yaw)
        return message

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
    def navigate(cls, target, timeout=45.0):
        assert cls.navigator.wait_for_server(timeout_sec=60.0)
        goal = NavigateToPose.Goal()
        goal.pose = cls.pose(target)
        sent = cls.navigator.send_goal_async(goal)
        assert cls.spin_until(sent.done, 10.0)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        if not cls.spin_until(result.done, timeout):
            cancel = handle.cancel_goal_async()
            cls.spin_until(cancel.done, 5.0)
            odom = cls.odometry[-1].pose.pose if cls.odometry else None
            nav = cls.nav_commands[-1] if cls.nav_commands else None
            safe = cls.safe_commands[-1] if cls.safe_commands else None
            raise AssertionError(
                f'navigation timed out for {target}; '
                f'odom={odom}; nav={nav}; safe={safe}')
        return result.result().status

    @classmethod
    def plan(cls, target, timeout=15.0):
        assert cls.planner.wait_for_server(timeout_sec=60.0)
        goal = ComputePathToPose.Goal()
        goal.goal = cls.pose(target)
        goal.planner_id = 'GridBased'
        goal.use_start = False
        sent = cls.planner.send_goal_async(goal)
        assert cls.spin_until(sent.done, 10.0)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        assert cls.spin_until(result.done, timeout)
        wrapped = result.result()
        return wrapped.status, wrapped.result.path

    @staticmethod
    def cost_at(grid, x, y):
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if column < 0 or row < 0:
            return -1
        if column >= grid.info.width or row >= grid.info.height:
            return -1
        return grid.data[row * grid.info.width + column]

    @classmethod
    def max_cost_near(cls, grid, x, y, radius=0.10):
        offsets = (-radius, 0.0, radius)
        return max(cls.cost_at(grid, x + dx, y + dy)
                   for dx in offsets for dy in offsets)

    def test_repeated_routes_static_temporary_and_unreachable(self):
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            self.wait_active(name)
        self.assertTrue(self.spin_until(
            lambda: self.odometry and self.global_costmaps, 30.0))

        # Repeat the same two-point route twice. Each accepted goal must finish
        # successfully, giving a deterministic 4/4 local arrival baseline.
        results = []
        started = time.monotonic()
        for _ in range(int(self.scenario['repeat_count'])):
            for target in self.scenario['route']:
                results.append(self.navigate(target))
        elapsed = time.monotonic() - started
        self.assertEqual(
            [GoalStatus.STATUS_SUCCEEDED] * 4, results,
            f'repeated route results={results}, elapsed={elapsed:.3f}s')

        # The known center wall must remain lethal and reject a goal placed in
        # its footprint before a controller command can be generated.
        static_goal = self.scenario['static_blocked_goal']
        wall_marked = self.spin_until(
            lambda: self.max_cost_near(
                self.global_costmaps[-1],
                static_goal['x'], static_goal['y']) >= 99,
            10.0)
        latest_cost = self.max_cost_near(
            self.global_costmaps[-1], static_goal['x'], static_goal['y'])
        self.assertTrue(
            wall_marked,
            f'static wall cost={latest_cost}, expected at least 99')
        status, path = self.plan(static_goal)
        self.assertEqual(GoalStatus.STATUS_ABORTED, status)
        self.assertFalse(path.poses)

        # Spawn the configured physical blocker at runtime. Laser data must
        # add it to the obstacle layer and the new path must detour around it.
        blocker = self.scenario['temporary_blocker_pose']
        command = [
            '/opt/ros/humble/lib/ros_gz_sim/create',
            '-world', 'm5_navigation', '-file', self.blocker_file,
            '-name', 'temporary_route_blocker',
            '-x', str(blocker['x']), '-y', str(blocker['y']),
            '-z', str(blocker['z']),
        ]
        spawned = subprocess.run(
            command, check=False, capture_output=True, text=True, timeout=15)
        self.assertEqual(0, spawned.returncode, spawned.stderr)
        # A lidar marks the first visible surface, not the solid model center.
        # The robot approaches this blocker from +x, so inspect its near face.
        blocker_face_x = blocker['x'] + 0.35
        self.assertTrue(self.spin_until(
            lambda: self.cost_at(
                self.global_costmaps[-1],
                blocker_face_x, blocker['y']) >= 99,
            12.0), 'temporary blocker did not enter the obstacle layer')
        status, path = self.plan(self.scenario['temporary_detour_goal'])
        self.assertEqual(GoalStatus.STATUS_SUCCEEDED, status)
        self.assertTrue(path.poses)
        # With a stationary robot the lidar observes the near face, not the
        # occluded volume behind it. Freeze clearance against that observed
        # surface point, which is the obstacle source used by the costmap.
        minimum_clearance = min(
            hypot(pose.pose.position.x - blocker_face_x,
                  pose.pose.position.y - blocker['y'])
            for pose in path.poses)
        self.assertGreaterEqual(minimum_clearance, 0.35)
        self.node.get_logger().info(
            'M5 obstacle metric: minimum path-to-observed-surface '
            f'clearance={minimum_clearance:.3f}m threshold=0.350m')

        # The named enclosed pocket lies outside the current known free-space
        # component. With allow_unknown=false it must be rejected explicitly.
        status, path = self.plan(self.scenario['unreachable_goal'])
        self.assertEqual(GoalStatus.STATUS_ABORTED, status)
        self.assertFalse(path.poses)
