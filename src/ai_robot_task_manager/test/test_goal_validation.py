import math

from ai_robot_interfaces.action import ExecuteRobotTask

from ai_robot_task_manager.goal_validation import validate_goal


BOUNDS = (-5.7, 5.7, -4.7, 4.7)


def goal():
    value = ExecuteRobotTask.Goal()
    value.task_id = 'task-1'
    value.target_pose.header.frame_id = 'map'
    value.target_pose.pose.orientation.w = 1.0
    value.perception_mode = value.PERCEPTION_IMAGE_CHECK
    return value


def test_valid_goal_and_boundary_are_accepted():
    value = goal()
    value.target_pose.pose.position.x = BOUNDS[1]
    value.target_pose.pose.position.y = BOUNDS[2]
    assert validate_goal(value, BOUNDS) == (0, '')


def test_identity_frame_pose_and_mode_validation():
    result = ExecuteRobotTask.Result
    cases = []
    value = goal()
    value.task_id = ' '
    cases.append(value)
    value = goal()
    value.target_pose.header.frame_id = 'odom'
    cases.append(value)
    value = goal()
    value.target_pose.pose.position.x = math.nan
    cases.append(value)
    value = goal()
    value.target_pose.pose.orientation.w = 0.5
    cases.append(value)
    value = goal()
    value.perception_mode = 99
    cases.append(value)
    for invalid in cases:
        assert validate_goal(invalid, BOUNDS)[0] == result.ERROR_INVALID_GOAL


def test_bounds_and_negative_timeout_have_distinct_results():
    result = ExecuteRobotTask.Result
    value = goal()
    value.target_pose.pose.position.x = 5.71
    assert validate_goal(value, BOUNDS)[0] == result.ERROR_OUT_OF_BOUNDS
    value = goal()
    value.navigation_timeout.sec = -1
    assert validate_goal(value, BOUNDS)[0] == result.ERROR_INVALID_GOAL
