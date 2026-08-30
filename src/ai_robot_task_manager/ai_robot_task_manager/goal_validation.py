"""Pure validation for ExecuteRobotTask goals."""

import math

from ai_robot_interfaces.action import ExecuteRobotTask


def _finite(values):
    return all(math.isfinite(value) for value in values)


def validate_goal(goal, bounds):
    """Return (result_code, detail) without causing ROS side effects."""
    result = ExecuteRobotTask.Result
    if not goal.task_id.strip():
        return result.ERROR_INVALID_GOAL, 'task_id must not be empty'
    if goal.target_pose.header.frame_id != 'map':
        return result.ERROR_INVALID_GOAL, 'target_pose frame_id must be map'

    position = goal.target_pose.pose.position
    orientation = goal.target_pose.pose.orientation
    pose_values = (
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
    )
    if not _finite(pose_values):
        return result.ERROR_INVALID_GOAL, 'target_pose must be finite'
    quaternion_norm = math.sqrt(
        orientation.x ** 2 + orientation.y ** 2
        + orientation.z ** 2 + orientation.w ** 2)
    if abs(quaternion_norm - 1.0) > 0.01:
        return result.ERROR_INVALID_GOAL, 'orientation must be normalized'

    min_x, max_x, min_y, max_y = bounds
    if not (min_x <= position.x <= max_x
            and min_y <= position.y <= max_y):
        return result.ERROR_OUT_OF_BOUNDS, 'target is outside task bounds'

    if goal.perception_mode not in (
            goal.PERCEPTION_NONE, goal.PERCEPTION_IMAGE_CHECK):
        return result.ERROR_INVALID_GOAL, 'unknown perception_mode'
    for name, duration in (
            ('navigation_timeout', goal.navigation_timeout),
            ('perception_timeout', goal.perception_timeout)):
        if duration.sec < 0:
            return result.ERROR_INVALID_GOAL, f'{name} must not be negative'
    return result.ERROR_NONE, ''
