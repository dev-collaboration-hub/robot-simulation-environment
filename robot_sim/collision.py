"""2D capsule-versus-circle/box collision and nonadjacent self-collision.

The arm has finite link radius; collision guard samples each joint transition to
reduce tunneling. This is a conservative stop, NOT contact-force resolution.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import hypot, ceil
from .arm import ArmModel
from .math3d import Vec2, finite


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float

    def __post_init__(self):
        for key in ('x', 'y', 'radius'):
            object.__setattr__(self, key, finite(getattr(self, key), key))
        if self.radius <= 0:
            raise ValueError('circle radius must be positive')


@dataclass(frozen=True)
class BoxObstacle:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def __post_init__(self):
        for key in ('min_x', 'min_y', 'max_x', 'max_y'):
            object.__setattr__(self, key, finite(getattr(self, key), key))
        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError('nonempty box expected')


def point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> float:
    dx, dy = b.x-a.x, b.y-a.y
    norm = dx*dx+dy*dy
    t = max(0., min(1., ((p.x-a.x)*dx+(p.y-a.y)*dy)/norm)) if norm else 0.
    return hypot(a.x+t*dx-p.x, a.y+t*dy-p.y)


def point_box_distance(p: Vec2, box: BoxObstacle) -> float:
    return hypot(max(box.min_x-p.x, 0., p.x-box.max_x),
                 max(box.min_y-p.y, 0., p.y-box.max_y))


def _orient(a, b, c):
    return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)


def segments_intersect(a, b, c, d) -> bool:
    eps = 1e-12
    ab_c, ab_d = _orient(a,b,c), _orient(a,b,d)
    cd_a, cd_b = _orient(c,d,a), _orient(c,d,b)
    if ((ab_c > eps and ab_d < -eps) or (ab_c < -eps and ab_d > eps)) and (
        (cd_a > eps and cd_b < -eps) or (cd_a < -eps and cd_b > eps)):
        return True
    # Endpoints, collinear and tangencies.
    for p, u, v in ((c,a,b),(d,a,b),(a,c,d),(b,c,d)):
        if abs(_orient(u,v,p)) <= eps and (
            min(u.x,v.x)-eps <= p.x <= max(u.x,v.x)+eps and
            min(u.y,v.y)-eps <= p.y <= max(u.y,v.y)+eps):
            return True
    return False


def segment_segment_distance(a, b, c, d) -> float:
    if segments_intersect(a,b,c,d):
        return 0.
    return min(point_segment_distance(a,c,d), point_segment_distance(b,c,d),
               point_segment_distance(c,a,b), point_segment_distance(d,a,b))


def segment_box_distance(a, b, box: BoxObstacle) -> float:
    corners = [Vec2(box.min_x,box.min_y), Vec2(box.max_x,box.min_y),
               Vec2(box.max_x,box.max_y), Vec2(box.min_x,box.max_y)]
    return min(point_box_distance(a,box), point_box_distance(b,box),
               *(segment_segment_distance(a,b,u,v) for u,v in zip(corners, corners[1:]+corners[:1])))


@dataclass(frozen=True)
class CollisionReport:
    collided: bool
    detail: str = ''


class CollisionWorld:
    def __init__(self, obstacles=(), link_radius=.045, check_self=True):
        self.obstacles = tuple(obstacles)
        self.link_radius = finite(link_radius, 'link radius')
        if self.link_radius <= 0:
            raise ValueError('link radius must be positive')
        if not all(isinstance(x, (CircleObstacle, BoxObstacle)) for x in self.obstacles):
            raise TypeError('unsupported obstacle type')
        self.check_self = bool(check_self)

    def inspect(self, model: ArmModel, angles) -> CollisionReport:
        pose = model.forward(angles)
        segments = list(zip(pose.positions, pose.positions[1:]))
        for i, (a, b) in enumerate(segments):
            for k, obstacle in enumerate(self.obstacles):
                if isinstance(obstacle, CircleObstacle):
                    distance = point_segment_distance(Vec2(obstacle.x, obstacle.y), a, b)
                    threshold = obstacle.radius+self.link_radius
                else:
                    distance = segment_box_distance(a, b, obstacle)
                    threshold = self.link_radius
                if distance <= threshold+1e-12:
                    return CollisionReport(True, f'link {i+1} vs obstacle {k+1}')
            if self.check_self:
                for j in range(i+2, len(segments)):
                    c,d = segments[j]
                    if segment_segment_distance(a,b,c,d) <= 2*self.link_radius+1e-12:
                        return CollisionReport(True, f'link {i+1} vs link {j+1}')
        return CollisionReport(False)

    def sweep(self, model: ArmModel, start, end) -> CollisionReport:
        a, b = model.validate_angles(tuple(start)), model.validate_angles(tuple(end))
        # Arc-length upper bound of the tip for each joint rotation.
        max_arc = sum(abs(x-y)*sum(j.length for j in model.joints[i:]) for i,(x,y) in enumerate(zip(a,b)))
        subdivisions = max(1, ceil(max_arc / max(self.link_radius*.4, .008)))
        if subdivisions > 1024:
            return CollisionReport(True, 'movement too large for bounded sweep')
        for k in range(subdivisions+1):
            u = k/subdivisions
            test = self.inspect(model, tuple(x+(y-x)*u for x,y in zip(a,b)))
            if test.collided:
                return CollisionReport(True, f'{test.detail} at sweep {k}/{subdivisions}')
        return CollisionReport(False)
