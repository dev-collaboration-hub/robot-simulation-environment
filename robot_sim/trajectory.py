"""Time-parameterized C1 continuous joint trajectories and playback.

Smoothstep zeroes endpoint velocities. Conservative T bounds peak joint speed
and acceleration analytically; acceleration changes abruptly at endpoints.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import sqrt
from .math3d import finite
from .arm import ArmModel

@dataclass(frozen=True)
class Trajectory:
    start: tuple[float, ...]
    end: tuple[float, ...]
    duration: float
    velocity_limits: tuple[float, ...]
    acceleration_limits: tuple[float, ...]

    def _u(self, t: float) -> float:
        t = finite(t, 'time')
        return min(1., max(0., t/self.duration))

    def sample(self, t: float) -> tuple[float, ...]:
        u = self._u(t)
        f = u*u*(3-2*u)
        return tuple(a+(b-a)*f for a,b in zip(self.start, self.end))

    def velocity(self, t: float) -> tuple[float, ...]:
        u = self._u(t)
        if t <= 0 or t >= self.duration:
            return tuple(0. for _ in self.start)
        return tuple((b-a)*6*u*(1-u)/self.duration for a,b in zip(self.start, self.end))

    def acceleration(self, t: float) -> tuple[float, ...]:
        u = self._u(t)
        if t <= 0 or t >= self.duration:
            return tuple(0. for _ in self.start)
        return tuple((b-a)*(6-12*u)/self.duration**2 for a,b in zip(self.start, self.end))


def plan_joint_path(model: ArmModel, start, end, accel_limit: float = 2.5,
                    velocity_scale: float = .7) -> Trajectory:
    a, b = model.validate_angles(tuple(start)), model.validate_angles(tuple(end))
    accel_limit = finite(accel_limit, 'accel limit')
    velocity_scale = finite(velocity_scale, 'velocity scale')
    if accel_limit <= 0 or not 0 < velocity_scale <= 1:
        raise ValueError('invalid velocity or acceleration bounds')
    speeds = tuple(j.max_speed*velocity_scale for j in model.joints)
    acceleration = tuple(accel_limit for _ in a)
    duration = max([.01] + [1.5*abs(q-p)/speed
                            for p,q,speed in zip(a,b,speeds)]
                        + [sqrt(6*abs(q-p)/accel)
                           for p,q,accel in zip(a,b,acceleration)])
    return Trajectory(a,b,duration,speeds,acceleration)


class TrajectoryPlayer:
    def __init__(self):
        self.path: Trajectory | None = None
        self.elapsed = 0.

    def load(self, path: Trajectory):
        self.path = path
        self.elapsed = 0.

    def cancel(self):
        self.path = None
        self.elapsed = 0.

    def step(self, dt: float):
        if self.path is None:
            return None
        dt = finite(dt, 'dt')
        if dt <= 0:
            raise ValueError('dt must be positive')
        self.elapsed = min(self.path.duration, self.elapsed+dt)
        return self.path.sample(self.elapsed)

    @property
    def finished(self):
        return self.path is None or self.elapsed >= self.path.duration
