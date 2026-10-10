"""Simulated joint motor bank: torque-saturated PID with anti-windup."""
from __future__ import annotations
from dataclasses import dataclass
from .math3d import finite
from .arm import ArmModel
from .physics import PlanarPhysics


@dataclass(frozen=True)
class MotorConfig:
    kp: float = 16.0
    ki: float = .8
    kd: float = 5.0
    torque_limit: float = 14.0

    def __post_init__(self):
        for key in ('kp','ki','kd','torque_limit'):
            value = finite(getattr(self, key), key)
            if value < 0 or (key == 'torque_limit' and value == 0):
                raise ValueError('nonnegative gains and positive torque limit required')


class VirtualMotors:
    def __init__(self, model: ArmModel, physics: PlanarPhysics | None = None, config: MotorConfig | None = None):
        self.model = model
        self.physics = physics if physics is not None else PlanarPhysics(model, gravity=0.)
        if self.physics.model != self.model:
            raise ValueError('motor model and physical model must match')
        self.config = config or MotorConfig()
        self.integral = [0.]*len(model.joints)
        self.target = model.initial_angles
        self.last_torques = (0.,)*len(model.joints)

    def command(self, target):
        self.target = self.model.validate_angles(tuple(target))

    def reset(self):
        self.physics.reset()
        self.integral = [0.]*len(self.model.joints)
        self.target = self.model.initial_angles
        self.last_torques = (0.,)*len(self.model.joints)

    def step(self, dt: float):
        dt = finite(dt, 'dt')
        if not 0 < dt <= .02:
            raise ValueError('motor dt outside supported range')
        torques = []
        c = self.config
        for i,(target,position,velocity) in enumerate(zip(self.target,self.physics.angles,self.physics.velocities)):
            error = target-position
            candidate_integral = max(-5., min(5., self.integral[i]+error*dt))
            raw = c.kp*error+c.ki*candidate_integral-c.kd*velocity
            bounded = max(-c.torque_limit, min(c.torque_limit, raw))
            # Freeze integral if saturated and error pushes further into saturation.
            if abs(raw) <= c.torque_limit or raw*error < 0:
                self.integral[i] = candidate_integral
            torques.append(bounded)
        self.last_torques = tuple(torques)
        return self.physics.step(self.last_torques,dt)
