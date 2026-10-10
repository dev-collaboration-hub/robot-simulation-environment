"""Deterministic virtual joint encoders and ray-based distance sensing (M11).

Sensor readings are snapshots, not aliases of privileged simulator state.
The only randomness comes from an owned, resettable seeded PRNG.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import cos, sin, sqrt
from random import Random
from .math3d import Vec2, finite
from .arm import ArmModel
from .collision import CircleObstacle, BoxObstacle


def ray_distance(origin: Vec2, heading: float, obstacles, maximum: float) -> float:
    maximum = finite(maximum, 'max_range')
    if maximum <= 0:
        raise ValueError('max_range must be positive')
    dx, dy = cos(finite(heading, 'heading')), sin(heading)
    nearest = maximum
    for shape in obstacles:
        if isinstance(shape, CircleObstacle):
            ox, oy = origin.x-shape.x, origin.y-shape.y
            b = ox*dx+oy*dy
            c = ox*ox+oy*oy-shape.radius*shape.radius
            discriminant = b*b-c
            if discriminant >= 0:
                root = sqrt(discriminant)
                if c <= 0:
                    nearest = 0.
                else:
                    for t in (-b-root, -b+root):
                        if 0 <= t <= nearest:
                            nearest = t
        elif isinstance(shape, BoxObstacle):
            tmin, tmax = 0., nearest
            for start, direction, low, high in (
                    (origin.x, dx, shape.min_x, shape.max_x),
                    (origin.y, dy, shape.min_y, shape.max_y)):
                if abs(direction) < 1e-12:
                    if not low <= start <= high:
                        tmin = float('inf')
                        break
                else:
                    a, b = (low-start)/direction, (high-start)/direction
                    tmin, tmax = max(tmin, min(a,b)), min(tmax, max(a,b))
            if tmax >= tmin and tmin <= nearest:
                nearest = max(0., tmin)
        else:
            raise TypeError('unsupported obstacle')
    return nearest


@dataclass(frozen=True)
class SensorReading:
    time: float
    joint_angles: tuple[float, ...]
    distance: float
    sequence: int


class VirtualSensorArray:
    def __init__(self, model: ArmModel, seed: int = 1,
                 interval: float = .05, encoder_stddev: float = 0.,
                 distance_stddev: float = 0., max_distance: float = 4.):
        self.model = model
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError('seed must be an integer')
        self.seed = seed
        self.interval = finite(interval, 'interval')
        self.encoder_stddev = finite(encoder_stddev, 'encoder_stddev')
        self.distance_stddev = finite(distance_stddev, 'distance_stddev')
        self.max_distance = finite(max_distance, 'max_distance')
        if self.interval <= 0 or self.encoder_stddev < 0 or self.distance_stddev < 0 or self.max_distance <= 0:
            raise ValueError('invalid sensor settings')
        self.reset()

    def reset(self) -> None:
        self._rng = Random(self.seed)
        self._ticks = 0
        self._clock = 0.
        self.latest: SensorReading | None = None

    def sample(self, dt: float, angles, obstacles=()) -> SensorReading | None:
        dt = finite(dt, 'dt')
        if not 0 < dt <= .25:
            raise ValueError('sensor timestep must lie in (0, .25]')
        valid = self.model.validate_angles(tuple(angles))
        self._clock += dt
        tick_count = int((self._clock+1e-12)/self.interval)
        if tick_count <= self._ticks:
            return self.latest
        # Sampling occurs at a well-defined virtual clock, independent of UI refresh.
        self._ticks = tick_count
        pose = self.model.forward(valid)
        measured = tuple(angle+self._rng.gauss(0, self.encoder_stddev) if self.encoder_stddev else angle
                         for angle in valid)
        observed = ray_distance(pose.tool, pose.orientations[-1], obstacles, self.max_distance)
        if self.distance_stddev:
            observed += self._rng.gauss(0, self.distance_stddev)
        observed = max(0., min(self.max_distance, observed))
        self.latest = SensorReading(round(tick_count*self.interval, 10), measured, observed, tick_count)
        return self.latest
