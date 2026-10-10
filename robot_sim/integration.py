"""M14 optional boundary for externally authored motion planners.

The external repositories currently document architecture but expose no
verified Python API. This adapter requires an explicit, testable protocol; it
never guesses an import path or silently grants real-hardware access.
"""
from __future__ import annotations
import importlib
from .arm import ArmModel
from .trajectory import plan_joint_path, Trajectory


CONTRACT_VERSION = 1


class AdapterUnavailable(RuntimeError):
    pass


class MotionPlannerAdapter:
    def __init__(self, model: ArmModel, external_module: str | None = None):
        self.model = model
        self.external_module = external_module
        self.backend = 'local'
        self._planner = None
        if external_module is not None:
            if not external_module or not external_module.replace('_','').replace('.','').isalnum():
                raise ValueError('invalid module name')
            try:
                module = importlib.import_module(external_module)
            except ImportError as exc:
                raise AdapterUnavailable(f'{external_module} is not installed') from exc
            planner = getattr(module, 'plan_joint_trajectory_v1', None)
            if not callable(planner):
                raise AdapterUnavailable('module lacks callable plan_joint_trajectory_v1')
            self._planner = planner
            self.backend = external_module

    def plan(self, start, end) -> Trajectory:
        a=self.model.validate_angles(tuple(start))
        b=self.model.validate_angles(tuple(end))
        if self._planner is None:
            return plan_joint_path(self.model,a,b)
        payload = {
            'contract_version': CONTRACT_VERSION,
            'unit': 'radians',
            'joint_names': [j.name for j in self.model.joints],
            'start': list(a),
            'end': list(b),
            'max_speed': [j.max_speed for j in self.model.joints],
            'angle_limits': [[j.minimum,j.maximum] for j in self.model.joints],
        }
        result=self._planner(payload)
        if not isinstance(result,dict):
            raise ValueError('external planner must return a dictionary')
        if result.get('contract_version') != CONTRACT_VERSION:
            raise ValueError('unsupported external planner contract version')
        path=Trajectory(a,b,float(result['duration']),
                        tuple(float(v) for v in result['velocity_limits']),
                        tuple(float(v) for v in result['acceleration_limits']))
        if not 0 < path.duration <= 3600 or len(path.velocity_limits)!=len(a) or len(path.acceleration_limits)!=len(a):
            raise ValueError('external planner provided invalid dimensions/duration')
        for name, values in (('velocity',path.velocity_limits),('acceleration',path.acceleration_limits)):
            if not all(0 < x < 1e6 for x in values):
                raise ValueError(f'external {name} limits must be finite positive numbers')
        # Our smoothstep bound must fit the planner's declared limits.
        for i,(s,t) in enumerate(zip(a,b)):
            delta=abs(t-s)
            if 1.5*delta/path.duration > path.velocity_limits[i]+1e-9:
                raise ValueError('external plan would exceed speed limit')
            if 6*delta/(path.duration**2) > path.acceleration_limits[i]+1e-9:
                raise ValueError('external plan would exceed acceleration limit')
            if path.velocity_limits[i] > self.model.joints[i].max_speed+1e-9:
                raise ValueError('external plan exceeds robot joint max speed')
        return path
