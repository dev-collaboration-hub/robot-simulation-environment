"""Fixed-step deterministic joint motion simulator; renderer-independent."""
from __future__ import annotations
from dataclasses import dataclass
from .math3d import finite
from .arm import ArmModel, ArmPose


@dataclass(frozen=True)
class Snapshot:
    step: int
    time: float
    angles: tuple[float, ...]
    targets: tuple[float, ...]
    pose: ArmPose
    paused: bool


class Simulator:
    def __init__(self, model: ArmModel | None = None, dt: float = 1/60):
        self.model = model if model is not None else ArmModel.basic()
        self.dt = finite(dt, 'dt')
        if not 0 < self.dt <= 0.1:
            raise ValueError('dt must be greater than 0 and at most 0.1')
        self.reset()

    def reset(self) -> None:
        self.angles = self.model.initial_angles
        self.targets = self.angles
        self.steps = 0
        self.paused = True
        self._remainder = 0.0

    def command(self, angles: tuple[float, ...] | list[float]) -> None:
        self.targets = self.model.validate_angles(angles)

    def pause(self) -> None:
        self.paused = True
        self._remainder = 0.0

    def resume(self) -> None:
        self.paused = False
        self._remainder = 0.0

    def step_once(self) -> Snapshot:
        next_angles = []
        for old, target, joint in zip(self.angles, self.targets, self.model.joints):
            delta = target - old
            stride = joint.max_speed * self.dt
            if abs(delta) <= stride:
                next_angles.append(target)
            else:
                next_angles.append(old + (stride if delta > 0 else -stride))
        self.angles = self.model.validate_angles(tuple(next_angles))
        self.steps += 1
        return self.snapshot()

    def advance(self, elapsed: float) -> int:
        """Consume elapsed wall time with fixed-size steps, capped at 0.25s/frame."""
        elapsed = finite(elapsed, 'elapsed')
        if elapsed < 0:
            raise ValueError('elapsed must not be negative')
        if self.paused:
            return 0
        self._remainder += min(elapsed, 0.25)
        count = 0
        while self._remainder + 1e-12 >= self.dt:
            self.step_once()
            self._remainder -= self.dt
            count += 1
        return count

    def snapshot(self) -> Snapshot:
        return Snapshot(self.steps, self.steps*self.dt, self.angles,
                        self.targets, self.model.forward(self.angles), self.paused)
