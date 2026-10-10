"""M12 simulated planar gripper: guarded grasp, attachment, follow, and release.

Objects follow the tool only through repeated simulation ticks, never through
a one-off teleport to the destination. World collision/bounds are enforced.
"""
from __future__ import annotations
from dataclasses import dataclass
from .math3d import Vec2, finite
from .world import World


@dataclass(frozen=True)
class GripResult:
    success: bool
    reason: str
    held: str | None


class Gripper:
    def __init__(self, world: World, tolerance: float = .10, force: float = 10.):
        self.world=world
        self.tolerance=finite(tolerance,'tolerance')
        self.force=finite(force,'force')
        if self.tolerance <= 0 or self.force <= 0:
            raise ValueError('gripper tolerance and force must be positive')
        self.reset()

    def reset(self):
        self.held: str | None = None
        self.opened=True
        self.offset=Vec2(0.,0.)
        self.last_reason='ready'

    def close(self, tool: Vec2) -> GripResult:
        if not isinstance(tool,Vec2):
            raise TypeError('tool must be Vec2')
        if self.held is not None:
            return GripResult(False,'already holding an object',self.held)
        candidates = sorted(
            ((o.position.distance_to(tool),o.name) for o in self.world.objects.values()),
            key=lambda item: (item[0],item[1]))
        if not candidates:
            return GripResult(False,'no objects in scene',None)
        distance, name = candidates[0]
        # The gripper is assumed to center its fingers on a nearby object.
        if distance > self.tolerance+self.world.get(name).radius:
            self.last_reason='object out of reach'
            return GripResult(False,self.last_reason,None)
        self.held=name
        self.offset=self.world.get(name).position-tool
        self.opened=False
        self.last_reason='grasped '+name
        return GripResult(True,self.last_reason,self.held)

    def follow(self, tool: Vec2) -> GripResult:
        if self.held is None:
            return GripResult(True,'nothing held',None)
        if not self.world.move(self.held,tool+self.offset,carried=True):
            self.last_reason='blocked object movement'
            return GripResult(False,self.last_reason,self.held)
        self.last_reason='carrying '+self.held
        return GripResult(True,self.last_reason,self.held)

    def open(self) -> GripResult:
        if self.held is None:
            self.opened=True
            return GripResult(False,'nothing to release',None)
        name=self.held
        # The latest verified followed position becomes the resting position.
        self.held=None
        self.offset=Vec2(0.,0.)
        self.opened=True
        self.last_reason='released '+name
        return GripResult(True,self.last_reason,None)
