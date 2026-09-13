import pytest

from ai_robot_task_manager.state_machine import TaskPhase, TaskStateMachine


def test_happy_path_and_release():
    state = TaskStateMachine()
    assert state.start('task-1')
    assert state.transition(TaskPhase.NAVIGATING)
    assert state.transition(TaskPhase.PERCEIVING)
    assert state.transition(TaskPhase.FINALIZING)
    assert state.finish('task-1')
    assert state.snapshot().phase == TaskPhase.IDLE


def test_single_owner_and_forward_only_transitions():
    state = TaskStateMachine()
    assert not state.start('')
    assert state.start('task-1')
    assert not state.start('task-2')
    assert not state.transition(TaskPhase.PERCEIVING)
    assert state.snapshot().task_id == 'task-1'


def test_cancel_requires_matching_active_task_and_cleans_up():
    state = TaskStateMachine()
    assert state.start('task-1')
    assert not state.request_cancel('task-2')
    assert state.request_cancel('task-1')
    assert state.snapshot().cancel_requested
    assert state.snapshot().phase == TaskPhase.FINALIZING
    assert not state.finish('task-2')
    assert state.finish('task-1')


def test_invalid_phase_value_is_rejected():
    state = TaskStateMachine()
    state.start('task-1')
    with pytest.raises(ValueError):
        state.transition(99)
