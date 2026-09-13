"""Thread-safe single-task state machine for robot capability execution."""

from dataclasses import dataclass
from enum import IntEnum
from threading import Lock


class TaskPhase(IntEnum):
    """Phases aligned with ExecuteRobotTask feedback constants."""

    IDLE = -1
    VALIDATING = 0
    NAVIGATING = 1
    PERCEIVING = 2
    FINALIZING = 3


@dataclass(frozen=True)
class TaskSnapshot:
    """Immutable task state exposed to the ROS node and tests."""

    task_id: str
    phase: TaskPhase
    cancel_requested: bool


class TaskStateMachine:
    """Own exactly one task and enforce forward-only phase transitions."""

    _allowed = {
        TaskPhase.VALIDATING: {
            TaskPhase.NAVIGATING, TaskPhase.FINALIZING},
        TaskPhase.NAVIGATING: {
            TaskPhase.PERCEIVING, TaskPhase.FINALIZING},
        TaskPhase.PERCEIVING: {TaskPhase.FINALIZING},
        TaskPhase.FINALIZING: set(),
    }

    def __init__(self):
        self._lock = Lock()
        self._task_id = ''
        self._phase = TaskPhase.IDLE
        self._cancel_requested = False

    def snapshot(self):
        """Return a consistent immutable snapshot."""
        with self._lock:
            return TaskSnapshot(
                self._task_id, self._phase, self._cancel_requested)

    def start(self, task_id):
        """Reserve the manager for a non-empty task ID."""
        with self._lock:
            if self._phase != TaskPhase.IDLE or not task_id.strip():
                return False
            self._task_id = task_id
            self._phase = TaskPhase.VALIDATING
            self._cancel_requested = False
            return True

    def transition(self, phase):
        """Move forward along a legal execution or finalization edge."""
        phase = TaskPhase(phase)
        with self._lock:
            if phase not in self._allowed.get(self._phase, set()):
                return False
            self._phase = phase
            return True

    def request_cancel(self, task_id):
        """Mark the active matching task canceled and begin finalization."""
        with self._lock:
            if self._phase == TaskPhase.IDLE or task_id != self._task_id:
                return False
            self._cancel_requested = True
            self._phase = TaskPhase.FINALIZING
            return True

    def finish(self, task_id):
        """Release a finalized task; unfinished tasks cannot be discarded."""
        with self._lock:
            if (self._phase != TaskPhase.FINALIZING
                    or task_id != self._task_id):
                return False
            self._task_id = ''
            self._phase = TaskPhase.IDLE
            self._cancel_requested = False
            return True
