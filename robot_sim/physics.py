"""Educational 2D joint-level gravity/damping dynamics; not coupled rigid-body physics."""
from __future__ import annotations
from dataclasses import dataclass
from math import cos
from .arm import ArmModel
from .math3d import finite

@dataclass(frozen=True)
class BodySpec:
    mass: float = 1.0
    damping: float = 1.3

    def __post_init__(self):
        if finite(self.mass, 'mass') <= 0 or finite(self.damping, 'damping') < 0:
            raise ValueError('mass must be positive; damping nonnegative')


class PlanarPhysics:
    """Each hinge is independently driven; gravity uses its link's absolute pose.

    Omits multibody inertia coupling, frictional contacts, stress, and reaction
    forces. Use only for qualitative simulation, not a hardware digital twin.
    """
    def __init__(self, model: ArmModel, bodies=None, gravity: float = 9.81):
        self.model = model
        self.bodies = tuple(bodies) if bodies is not None else tuple(BodySpec() for _ in model.joints)
        if len(self.bodies) != len(model.joints) or not all(isinstance(x, BodySpec) for x in self.bodies):
            raise ValueError('one valid BodySpec per joint is required')
        self.gravity = finite(gravity, 'gravity')
        if self.gravity < 0:
            raise ValueError('gravity cannot be negative')
        self.reset()

    def reset(self):
        self.angles = self.model.initial_angles
        self.velocities = (0.,)*len(self.angles)

    def step(self, torques, dt: float):
        dt = finite(dt, 'dt')
        if not 0 < dt <= .02:
            raise ValueError('physics dt must lie in (0, .02] seconds')
        if len(torques) != len(self.model.joints):
            raise ValueError('torque count mismatch')
        torques = tuple(finite(x, 'torque') for x in torques)
        headings = self.model.forward(self.angles).orientations
        new_angles, new_velocities = [], []
        for joint, body, angle, omega, torque, heading in zip(
                self.model.joints, self.bodies, self.angles, self.velocities, torques, headings):
            inertia = body.mass*joint.length**2/3
            gravity_torque = -body.mass*self.gravity*joint.length/2*cos(heading)
            alpha = (torque+gravity_torque-body.damping*omega)/inertia
            next_v = max(-joint.max_speed, min(joint.max_speed, omega+alpha*dt))
            next_q = angle+next_v*dt
            if next_q < joint.minimum:
                next_q, next_v = joint.minimum, 0.
            elif next_q > joint.maximum:
                next_q, next_v = joint.maximum, 0.
            new_angles.append(next_q)
            new_velocities.append(next_v)
        self.angles = self.model.validate_angles(tuple(new_angles))
        self.velocities = tuple(new_velocities)
        return self.angles
