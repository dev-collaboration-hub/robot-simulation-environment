"""Dependency-free 2D geometry (homogeneous transforms) for planar robot arms.

World convention: +x right, +y up, positive angles counterclockwise,
lengths in metres, angles in radians. Screen coordinates invert y.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import cos, sin, isfinite


def finite(value: float, name: str = 'value') -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def __post_init__(self) -> None:
        object.__setattr__(self, 'x', finite(self.x, 'x'))
        object.__setattr__(self, 'y', finite(self.y, 'y'))

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def scale(self, scalar: float) -> Vec2:
        scale = finite(scalar, 'scale')
        return Vec2(self.x * scale, self.y * scale)

    def distance_to(self, other: Vec2) -> float:
        from math import hypot
        return hypot(self.x-other.x, self.y-other.y)


@dataclass(frozen=True)
class Transform2:
    """SE(2) transform, rotation first then translation; no global state."""
    angle: float = 0.0
    offset: Vec2 = Vec2(0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, 'angle', finite(self.angle, 'angle'))
        if not isinstance(self.offset, Vec2):
            raise TypeError('offset must be Vec2')

    def rotate(self, point: Vec2) -> Vec2:
        c, s = cos(self.angle), sin(self.angle)
        return Vec2(c*point.x-s*point.y, s*point.x+c*point.y)

    def apply(self, point: Vec2) -> Vec2:
        return self.rotate(point) + self.offset

    def compose(self, child: Transform2) -> Transform2:
        """Apply child first, then self: self @ child."""
        return Transform2(self.angle + child.angle, self.apply(child.offset))

    def inverse(self) -> Transform2:
        reverse = Transform2(-self.angle)
        return Transform2(-self.angle, reverse.rotate(self.offset.scale(-1)))

    def matrix(self) -> tuple[tuple[float, float, float], ...]:
        c, s = cos(self.angle), sin(self.angle)
        return ((c, -s, self.offset.x), (s, c, self.offset.y), (0.0, 0.0, 1.0))
