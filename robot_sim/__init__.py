"""Offline scratch-built planar robotic arm simulator (M1-M4 foundation)."""
from .arm import ArmModel, ArmPose, JointSpec
from .engine import Simulator, Snapshot
from .math3d import Vec2, Transform2

__all__ = ['ArmModel', 'ArmPose', 'JointSpec', 'Simulator', 'Snapshot', 'Vec2', 'Transform2']
