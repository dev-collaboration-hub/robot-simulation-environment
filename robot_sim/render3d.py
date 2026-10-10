"""CPU-only 3D camera and shaded box painter on a Tk canvas.

This is a scratch-built projection/wire geometry renderer, *not* a GPU engine.
The M1-M4 arm is planar (XY); visual solid links extend along Z.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import sin, cos, hypot, isfinite
from .math3d import finite, Vec2
from .arm import ArmPose

@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __post_init__(self):
        for k in ('x', 'y', 'z'):
            object.__setattr__(self, k, finite(getattr(self, k), k))

    def __add__(self, b):
        return Vec3(self.x+b.x, self.y+b.y, self.z+b.z)

    def __sub__(self, b):
        return Vec3(self.x-b.x, self.y-b.y, self.z-b.z)

    def dot(self, b):
        return self.x*b.x+self.y*b.y+self.z*b.z


@dataclass
class Camera:
    yaw: float = 0.60
    pitch: float = 0.30
    distance: float = 6.5
    zoom: float = 1.0
    pan_x: float = 0.0
    pan_y: float = 0.0
    perspective: bool = True

    def __post_init__(self):
        self.validate()

    def validate(self):
        for key in ('yaw', 'pitch', 'distance', 'zoom', 'pan_x', 'pan_y'):
            finite(getattr(self, key), key)
        if not -1.4 <= self.pitch <= 1.4 or not 1 <= self.distance <= 200 or not .1 <= self.zoom <= 6:
            raise ValueError('camera out of allowed bounds')

    def axes(self):
        cy, sy, cp, sp = cos(self.yaw), sin(self.yaw), cos(self.pitch), sin(self.pitch)
        right = Vec3(cy, 0, -sy)
        up = Vec3(-sy*sp, cp, -cy*sp)
        toward_camera = Vec3(sy*cp, sp, cy*cp)
        return right, up, toward_camera

    def project(self, point: Vec3, width: int, height: int):
        self.validate()
        right, up, toward = self.axes()
        depth = self.distance-point.dot(toward)
        if depth <= 0.05:
            return None
        focus = min(width, height)*.9*self.zoom
        scale = focus/depth if self.perspective else focus/self.distance
        return (width/2 + self.pan_x + point.dot(right)*scale,
                height/2 + self.pan_y - point.dot(up)*scale, depth)

    def orbit(self, dx: float, dy: float):
        self.yaw += finite(dx)*.01
        self.pitch = max(-1.4, min(1.4, self.pitch+finite(dy)*.01))

    def change_zoom(self, wheel_steps: float):
        self.zoom = max(.1, min(6., self.zoom*1.1**finite(wheel_steps)))


def link_box(start: Vec2, end: Vec2, radius: float = .06, thickness: float = .10):
    """Eight corners of a rectangular solid link centered on XY segment."""
    dx, dy = end.x-start.x, end.y-start.y
    length = hypot(dx, dy)
    if length < 1e-9:
        raise ValueError('link endpoints must differ')
    px, py = -dy/length*radius, dx/length*radius
    return tuple(Vec3(x+sign*px, y+sign*py, z) for z in (-thickness/2, thickness/2)
                 for x, y in ((start.x, start.y), (end.x, end.y))
                 for sign in (-1, 1))


FACES = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 2, 6, 4),
         (1, 5, 7, 3), (0, 4, 5, 1), (2, 3, 7, 6))


class SoftwareRenderer3D:
    """Draw perspective projected 3D geometry on a standard Tk Canvas."""
    def __init__(self, camera: Camera | None = None):
        self.camera = camera or Camera()

    def draw(self, canvas, pose: ArmPose, obstacles=()):
        width, height = max(500, canvas.winfo_width()), max(330, canvas.winfo_height())
        canvas.delete('scene3d')
        primitives = []

        def line(a: Vec3, b: Vec3, color, weight=1):
            p, q = self.camera.project(a, width, height), self.camera.project(b, width, height)
            if p and q:
                primitives.append(((p[2]+q[2])/2, 'line', (p, q, color, weight)))

        # XY vertical workspace and XZ ground plane grid.
        for n in range(-5, 6):
            line(Vec3(n*.5, -.20, -2.5), Vec3(n*.5, -.20, 2.5), '#263548')
            line(Vec3(-2.5, -.20, n*.5), Vec3(2.5, -.20, n*.5), '#263548')
        line(Vec3(0, 0, 0), Vec3(.6, 0, 0), '#eb6a74', 3)
        line(Vec3(0, 0, 0), Vec3(0, .6, 0), '#67ddab', 3)
        line(Vec3(0, 0, 0), Vec3(0, 0, .6), '#83afff', 3)

        for i, (a, b) in enumerate(zip(pose.positions, pose.positions[1:])):
            vertices = link_box(a, b)
            colors = ('#335879', '#2e9dcd', '#256b8a', '#57b9e6', '#1c5678', '#297ba2') if i % 2 == 0 else (
                '#795f3b', '#c08e43', '#87662a', '#ffd17d', '#73542d', '#b38a4d')
            for face_idx, face in enumerate(FACES):
                points = [self.camera.project(vertices[j], width, height) for j in face]
                if all(points):
                    primitives.append((sum(p[2] for p in points)/len(points), 'face',
                                       (points, colors[face_idx])))

        # Basic obstacle wire rings in XY, at Z=0.
        for obstacle in obstacles:
            from .collision import CircleObstacle, BoxObstacle
            if isinstance(obstacle, CircleObstacle):
                vertices = [Vec3(obstacle.x + obstacle.radius*cos(i*6.28318530718/32),
                                 obstacle.y + obstacle.radius*sin(i*6.28318530718/32), 0)
                            for i in range(33)]
            elif isinstance(obstacle, BoxObstacle):
                vertices = [Vec3(obstacle.min_x, obstacle.min_y, 0),
                            Vec3(obstacle.max_x, obstacle.min_y, 0),
                            Vec3(obstacle.max_x, obstacle.max_y, 0),
                            Vec3(obstacle.min_x, obstacle.max_y, 0),
                            Vec3(obstacle.min_x, obstacle.min_y, 0)]
            else:
                continue
            for p, q in zip(vertices, vertices[1:]):
                line(p, q, '#ff7584', 2)

        for _, kind, args in sorted(primitives, key=lambda item: -item[0]):
            if kind == 'line':
                a, b, color, weight = args
                canvas.create_line(a[0], a[1], b[0], b[1], fill=color,
                                   width=weight, tags='scene3d')
            else:
                pts, color = args
                coords = [v for p in pts for v in p[:2]]
                canvas.create_polygon(*coords, fill=color, outline='#14263b',
                                      width=1, tags='scene3d')
        for p in pose.positions:
            result = self.camera.project(Vec3(p.x, p.y, 0), width, height)
            if result:
                x, y, _ = result
                canvas.create_oval(x-4, y-4, x+4, y+4, fill='#e8f4ff',
                                   outline='#132333', tags='scene3d')
        canvas.create_text(12, 14, anchor='nw', text='3D CPU renderer | drag: orbit | right-drag: pan | wheel: zoom',
                           fill='#c3d4e9', tags='scene3d')
        return len(primitives)
