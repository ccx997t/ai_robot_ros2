"""ROS 2 Action endpoint and ownership boundary for composite tasks."""

import time

from ai_robot_interfaces.action import ExecuteRobotTask
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from .goal_validation import validate_goal
from .state_machine import TaskPhase, TaskStateMachine


class TaskManagerNode(Node):
    """Validate and own tasks before navigation/perception integration."""

    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('min_x', -5.7)
        self.declare_parameter('max_x', 5.7)
        self.declare_parameter('min_y', -4.7)
        self.declare_parameter('max_y', 4.7)
        self.declare_parameter('navigation_enabled', False)
        mode = self.get_parameter('mode').value
        if mode not in ('sim', 'real'):
            raise ValueError('mode must be sim or real')
        self._state = TaskStateMachine()
        self._server = ActionServer(
            self,
            ExecuteRobotTask,
            '/execute_robot_task',
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
        )
        self.get_logger().info(
            f'task manager active (mode={mode}); '
            'navigation integration is disabled')

    @staticmethod
    def accept_goal(_goal_request):
        """Accept so validation and busy failures return stable result codes."""
        return GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _bounds(self):
        return tuple(self.get_parameter(name).value for name in (
            'min_x', 'max_x', 'min_y', 'max_y'))

    @staticmethod
    def _duration(seconds):
        seconds = max(0.0, seconds)
        whole = int(seconds)
        return Duration(
            sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))

    @staticmethod
    def _result(code, message, started):
        result = ExecuteRobotTask.Result()
        result.success = code == result.ERROR_NONE
        result.error_code = code
        result.message = message
        result.elapsed = TaskManagerNode._duration(time.monotonic() - started)
        return result

    def _feedback(self, goal_handle, phase, progress, detail):
        feedback = ExecuteRobotTask.Feedback()
        feedback.phase = int(phase)
        feedback.progress = progress
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    async def execute(self, goal_handle):
        """Validate and own a task; downstream execution follows in M6 item 3."""
        started = time.monotonic()
        goal = goal_handle.request
        if not self._state.start(goal.task_id):
            goal_handle.abort()
            return self._result(
                ExecuteRobotTask.Result.ERROR_BUSY,
                'another task is active or task_id is invalid', started)

        try:
            self._feedback(
                goal_handle, TaskPhase.VALIDATING, 0.0, 'validating goal')
            code, detail = validate_goal(goal, self._bounds())
            if code != ExecuteRobotTask.Result.ERROR_NONE:
                self._state.transition(TaskPhase.FINALIZING)
                goal_handle.abort()
                return self._result(code, detail, started)
            if goal_handle.is_cancel_requested:
                self._state.request_cancel(goal.task_id)
                goal_handle.canceled()
                return self._result(
                    ExecuteRobotTask.Result.ERROR_CANCELED,
                    'task canceled during validation', started)

            # A valid goal must never be reported successful before Nav2 and
            # perception are integrated.  This explicit result is testable and
            # prevents a placeholder node from masquerading as M6 completion.
            self._state.transition(TaskPhase.FINALIZING)
            goal_handle.abort()
            return self._result(
                ExecuteRobotTask.Result.ERROR_NAVIGATION_REJECTED,
                'navigation subtask is not integrated', started)
        finally:
            snapshot = self._state.snapshot()
            if snapshot.phase != TaskPhase.FINALIZING:
                self._state.transition(TaskPhase.FINALIZING)
            self._state.finish(goal.task_id)

    def destroy_node(self):
        self._server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TaskManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
