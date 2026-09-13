from pathlib import Path
import re


ROOT = Path(__file__).parents[1]
ACTION = ROOT / 'action' / 'ExecuteRobotTask.action'


def sections():
    content = ACTION.read_text(encoding='utf-8')
    result = content.split('---')
    assert len(result) == 3
    return tuple(result)


def constants(section, prefix):
    pattern = rf'^uint(?:8|16) ({prefix}[A-Z0-9_]+)=([0-9]+)$'
    return {
        name: int(value)
        for name, value in re.findall(pattern, section, re.MULTILINE)
    }


def test_goal_contract_freezes_identity_pose_modes_and_timeouts():
    goal, _, _ = sections()
    assert 'string task_id' in goal
    assert 'geometry_msgs/PoseStamped target_pose' in goal
    assert 'uint8 perception_mode' in goal
    assert 'builtin_interfaces/Duration navigation_timeout' in goal
    assert 'builtin_interfaces/Duration perception_timeout' in goal
    assert constants(goal, 'PERCEPTION_') == {
        'PERCEPTION_NONE': 0,
        'PERCEPTION_IMAGE_CHECK': 1,
    }


def test_result_error_codes_are_unique_and_stable():
    _, result, _ = sections()
    expected = {
        'ERROR_NONE': 0,
        'ERROR_INVALID_GOAL': 100,
        'ERROR_OUT_OF_BOUNDS': 101,
        'ERROR_BUSY': 102,
        'ERROR_NAVIGATION_REJECTED': 200,
        'ERROR_NAVIGATION_UNREACHABLE': 201,
        'ERROR_NAVIGATION_TIMEOUT': 202,
        'ERROR_CANCELED': 300,
        'ERROR_PERCEPTION_UNAVAILABLE': 400,
        'ERROR_PERCEPTION_TIMEOUT': 401,
        'ERROR_PERCEPTION_INVALID': 402,
        'ERROR_SAFETY_STOPPED': 500,
        'ERROR_INTERNAL': 900,
    }
    actual = constants(result, 'ERROR_')
    assert actual == expected
    assert len(actual.values()) == len(set(actual.values()))
    for field in (
            'bool success', 'uint16 error_code', 'string message',
            'builtin_interfaces/Duration elapsed',
            'string perception_result'):
        assert field in result


def test_feedback_phases_and_fields_are_stable():
    _, _, feedback = sections()
    assert constants(feedback, 'PHASE_') == {
        'PHASE_VALIDATING': 0,
        'PHASE_NAVIGATING': 1,
        'PHASE_PERCEIVING': 2,
        'PHASE_FINALIZING': 3,
    }
    assert 'uint8 phase' in feedback
    assert 'float32 progress' in feedback
    assert 'string detail' in feedback
