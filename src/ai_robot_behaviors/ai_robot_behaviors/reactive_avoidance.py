from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class AvoidanceConfig:
    forward_speed: float = 0.10
    turn_speed: float = 0.45
    stop_distance: float = 0.70
    clear_distance: float = 1.00
    front_half_angle: float = math.radians(25.0)
    side_angle: float = math.radians(100.0)

    def validate(self):
        if not 0.0 < self.forward_speed <= 0.30:
            raise ValueError('forward_speed must be in (0.0, 0.30]')
        if not 0.0 < self.turn_speed <= 0.80:
            raise ValueError('turn_speed must be in (0.0, 0.80]')
        if not 0.0 < self.stop_distance < self.clear_distance:
            raise ValueError('require 0 < stop_distance < clear_distance')
        if not 0.0 < self.front_half_angle < self.side_angle <= math.pi:
            raise ValueError('invalid scan sector angles')


@dataclass(frozen=True)
class MotionDecision:
    linear_x: float
    angular_z: float
    state: str
    front_distance: float
    left_distance: float
    right_distance: float


def _sector_min(samples: Iterable[tuple[float, float]], minimum: float,
                maximum: float, range_max: float) -> float:
    values = []
    for angle, value in samples:
        if minimum <= angle <= maximum and not math.isnan(value):
            values.append(range_max if math.isinf(value) else value)
    return min(values, default=0.0)


class ReactiveAvoidance:
    def __init__(self, config: AvoidanceConfig):
        config.validate()
        self.config = config
        self.turn_direction = 0

    def decide(self, ranges: Sequence[float], angle_min: float,
               angle_increment: float, range_max: float) -> MotionDecision:
        samples = [
            (angle_min + index * angle_increment, value)
            for index, value in enumerate(ranges)
        ]
        cfg = self.config
        front = _sector_min(samples, -cfg.front_half_angle,
                            cfg.front_half_angle, range_max)
        left = _sector_min(samples, cfg.front_half_angle,
                           cfg.side_angle, range_max)
        right = _sector_min(samples, -cfg.side_angle,
                            -cfg.front_half_angle, range_max)

        if self.turn_direction and front < cfg.clear_distance:
            return MotionDecision(0.0, self.turn_direction * cfg.turn_speed,
                                  'turning', front, left, right)
        if front < cfg.stop_distance:
            self.turn_direction = 1 if left >= right else -1
            return MotionDecision(0.0, self.turn_direction * cfg.turn_speed,
                                  'turning', front, left, right)
        self.turn_direction = 0
        return MotionDecision(cfg.forward_speed, 0.0, 'forward', front, left, right)
