import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(190 + os.getpid() % 10)
os.environ['ROS_LOG_DIR'] = f'/tmp/ai_robot_task_manager_test_{os.getpid()}'

from ai_robot_interfaces.action import ExecuteRobotTask
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ai_robot_task_manager'),
        'launch', 'task_manager.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'mode': 'sim'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestTaskManagerAction(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('task_manager_action_test')
        cls.client = ActionClient(
            cls.node, ExecuteRobotTask, '/execute_robot_task')

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
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
    def execute(cls, task_id, frame_id):
        goal = ExecuteRobotTask.Goal()
        goal.task_id = task_id
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose.orientation.w = 1.0
        future = cls.client.send_goal_async(goal)
        assert cls.spin_until(future.done, 5.0)
        handle = future.result()
        assert handle.accepted
        result_future = handle.get_result_async()
        assert cls.spin_until(result_future.done, 5.0)
        return result_future.result().result

    def test_validation_result_and_state_release_between_tasks(self):
        self.assertTrue(self.client.wait_for_server(timeout_sec=10.0))
        invalid = self.execute('invalid-frame', 'odom')
        self.assertFalse(invalid.success)
        self.assertEqual(invalid.ERROR_INVALID_GOAL, invalid.error_code)

        first = self.execute('valid-1', 'map')
        second = self.execute('valid-2', 'map')
        for result in (first, second):
            self.assertFalse(result.success)
            self.assertEqual(
                result.ERROR_NAVIGATION_REJECTED, result.error_code)
            self.assertNotEqual(result.ERROR_BUSY, result.error_code)
