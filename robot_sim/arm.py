"""Validated, hardware-agnostic planar revolute robot arm models."""
from __future__ import annotations
from dataclasses import dataclass
from math import pi
import json
from pathlib import Path
from typing import Any
from .math3d import Vec2, Transform2, finite


@dataclass(frozen=True)
class JointSpec:
    name: str
    length: float
    minimum: float = -pi
    maximum: float = pi
    initial: float = 0.0
    max_speed: float = pi

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('joint name must not be empty')
        for field in ('length', 'minimum', 'maximum', 'initial', 'max_speed'):
            object.__setattr__(self, field, finite(getattr(self, field), field))
        if self.length <= 0 or self.max_speed <= 0:
            raise ValueError('link length and speed must be positive')
        if self.minimum >= self.maximum:
            raise ValueError('minimum must be less than maximum')
        if not self.minimum <= self.initial <= self.maximum:
            raise ValueError('initial angle outside joint limits')


@dataclass(frozen=True)
class ArmPose:
    positions: tuple[Vec2, ...]       # base + endpoint of every link
    orientations: tuple[float, ...]  # cumulative link orientations

    @property
    def tool(self) -> Vec2:
        return self.positions[-1]


@dataclass(frozen=True)
class ArmModel:
    joints: tuple[JointSpec, ...]
    base: Vec2 = Vec2(0.0, 0.0)

    def __post_init__(self) -> None:
        joints = tuple(self.joints)
        if not 2 <= len(joints) <= 6:
            raise ValueError('planar arm must have 2 to 6 joints')
        if not all(isinstance(j, JointSpec) for j in joints):
            raise TypeError('joints must contain JointSpec values')
        if len({j.name for j in joints}) != len(joints):
            raise ValueError('joint names must be unique')
        if not isinstance(self.base, Vec2):
            raise TypeError('base must be Vec2')
        object.__setattr__(self, 'joints', joints)

    @classmethod
    def basic(cls) -> ArmModel:
        return cls((JointSpec('shoulder', 1.2), JointSpec('elbow', 0.9)))

    @classmethod
    def extended(cls) -> ArmModel:
        return cls(tuple(JointSpec(f'joint_{i+1}', 0.65, max_speed=1.4) for i in range(6)))

    @property
    def initial_angles(self) -> tuple[float, ...]:
        return tuple(j.initial for j in self.joints)

    @property
    def reach(self) -> float:
        return sum(j.length for j in self.joints)

    def validate_angles(self, angles: tuple[float, ...] | list[float]) -> tuple[float, ...]:
        if len(angles) != len(self.joints):
            raise ValueError(f'expected {len(self.joints)} joint angles')
        checked = tuple(finite(x, 'joint angle') for x in angles)
        for value, joint in zip(checked, self.joints):
            if not joint.minimum <= value <= joint.maximum:
                raise ValueError(f'{joint.name} angle outside limits')
        return checked

    def forward(self, angles: tuple[float, ...] | list[float]) -> ArmPose:
        checked = self.validate_angles(angles)
        frame = Transform2(offset=self.base)
        points = [self.base]
        headings = []
        for joint, angle in zip(self.joints, checked):
            frame = frame.compose(Transform2(angle))
            headings.append(frame.angle)
            end = frame.apply(Vec2(joint.length, 0.0))
            points.append(end)
            frame = Transform2(frame.angle, end)
        return ArmPose(tuple(points), tuple(headings))

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> ArmModel:
        if not isinstance(raw, dict):
            raise ValueError('model must be a JSON object')
        allowed = {'base', 'joints'}
        if set(raw)-allowed:
            raise ValueError(f'unknown model keys: {sorted(set(raw)-allowed)}')
        raw_base = raw.get('base', [0, 0])
        if not isinstance(raw_base, list) or len(raw_base) != 2:
            raise ValueError('base must be [x, y]')
        raw_joints = raw.get('joints')
        if not isinstance(raw_joints, list):
            raise ValueError('joints must be an array')
        permitted = {'name', 'length', 'minimum', 'maximum', 'initial', 'max_speed'}
        specs = []
        for entry in raw_joints:
            if not isinstance(entry, dict) or set(entry)-permitted:
                raise ValueError('invalid joint config keys')
            try:
                specs.append(JointSpec(**entry))
            except TypeError as exc:
                raise ValueError(f'invalid joint: {exc}') from exc
        return cls(tuple(specs), Vec2(*raw_base))

    @classmethod
    def from_json(cls, path: str | Path) -> ArmModel:
        with open(path, 'r', encoding='utf-8') as stream:
            return cls.from_dict(json.load(stream))
