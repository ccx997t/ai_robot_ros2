import math

import pytest

from ai_robot_behaviors.reactive_avoidance import AvoidanceConfig, ReactiveAvoidance


def scan(front=5.0, left=5.0, right=5.0):
    values = []
    for degree in range(-180, 181):
        if -25 <= degree <= 25:
            values.append(front)
        elif 25 < degree <= 100:
            values.append(left)
        elif -100 <= degree < -25:
            values.append(right)
        else:
            values.append(5.0)
    return values


def decide(behavior, values):
    return behavior.decide(values, -math.pi, math.radians(1.0), 8.0)


def test_clear_path_moves_forward():
    result = decide(ReactiveAvoidance(AvoidanceConfig()), scan())
    assert result.state == 'forward'
    assert result.linear_x == pytest.approx(0.10)
    assert result.angular_z == 0.0


def test_obstacle_turns_toward_larger_clearance():
    behavior = ReactiveAvoidance(AvoidanceConfig())
    left = decide(behavior, scan(front=0.5, left=2.0, right=0.8))
    assert left.state == 'turning'
    assert left.linear_x == 0.0
    assert left.angular_z > 0.0


def test_turn_hysteresis_prevents_oscillation_until_clear():
    behavior = ReactiveAvoidance(AvoidanceConfig())
    first = decide(behavior, scan(front=0.5, left=2.0, right=0.8))
    still_turning = decide(behavior, scan(front=0.85, left=0.5, right=3.0))
    clear = decide(behavior, scan(front=1.2, left=0.5, right=3.0))
    assert first.angular_z > 0.0
    assert still_turning.angular_z > 0.0
    assert clear.state == 'forward'


def test_invalid_safety_limits_are_rejected():
    with pytest.raises(ValueError):
        ReactiveAvoidance(AvoidanceConfig(forward_speed=0.5))
