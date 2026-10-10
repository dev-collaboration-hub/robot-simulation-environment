"""Analytic planar 2R IK and damped least squares 2–6R IK.

Forward geometry comes from ArmModel; no external numeric libraries.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import acos, atan2, cos, sin, hypot, pi
from .arm import ArmModel
from .math3d import Vec2, finite


@dataclass(frozen=True)
class IKResult:
    success: bool
    angles: tuple[float, ...]
    error: float
    iterations: int
    reason: str


def solve_ik(model: ArmModel, target: Vec2, seed=None, tolerance: float = 1e-3,
             max_iterations: int = 900) -> IKResult:
    if not isinstance(target, Vec2):
        raise TypeError('target must be Vec2')
    tolerance = finite(tolerance, 'tolerance')
    if tolerance <= 0 or not isinstance(max_iterations, int) or max_iterations < 1:
        raise ValueError('invalid tolerance / iterations')
    seed = model.validate_angles(tuple(seed) if seed is not None else model.initial_angles)
    length = model.base.distance_to(target)
    if length > model.reach+tolerance:
        return IKResult(False, seed, length-model.reach, 0, 'target beyond maximum reach')

    def score(angles):
        return model.forward(angles).tool.distance_to(target)

    if len(model.joints) == 2:
        a, b = (joint.length for joint in model.joints)
        x, y = target.x-model.base.x, target.y-model.base.y
        c2 = (x*x+y*y-a*a-b*b)/(2*a*b)
        if abs(c2) <= 1+1e-12:
            c2 = min(1, max(-1, c2))
            elbow = acos(c2)
            candidates = []
            for theta2 in (elbow, -elbow):
                theta1 = atan2(y, x)-atan2(b*sin(theta2), a+b*cos(theta2))
                # Equivalent ±2pi representation may fit limits better.
                for rotation in (-2*pi, 0, 2*pi):
                    pair = (theta1+rotation, theta2)
                    if all(j.minimum-1e-12 <= v <= j.maximum+1e-12 for j, v in zip(model.joints, pair)):
                        pair = tuple(max(j.minimum, min(j.maximum, v)) for j, v in zip(model.joints, pair))
                        candidates.append(pair)
            if candidates:
                answer = min(candidates, key=lambda q: (score(q), sum((q[i]-seed[i])**2 for i in range(2))))
                err = score(answer)
                return IKResult(err <= tolerance, answer, err, 1, 'analytic two-link solution')

    n = len(model.joints)
    limits = [(j.minimum, j.maximum) for j in model.joints]
    # Multiple deterministic seeds avoid stalls at straight-arm singularities.
    starts = [seed, model.initial_angles,
              tuple((j.minimum+j.maximum)/2 for j in model.joints),
              tuple(min(j.maximum, max(j.minimum, .45*(-1 if i % 2 else 1))) for i, j in enumerate(model.joints)),
              tuple(min(j.maximum, max(j.minimum, -.7 if i % 2 else .7)) for i, j in enumerate(model.joints))]
    best, best_error = seed, score(seed)
    total_iter = 0
    for initial in starts:
        values = list(initial)
        for iteration in range(max_iterations):
            total_iter += 1
            pose = model.forward(values)
            ex, ey = target.x-pose.tool.x, target.y-pose.tool.y
            error = hypot(ex, ey)
            if error < best_error:
                best, best_error = tuple(values), error
            if error <= tolerance:
                return IKResult(True, tuple(values), error, total_iter, 'damped least squares')
            sin_terms = [sin(a) for a in pose.orientations]
            cos_terms = [cos(a) for a in pose.orientations]
            jx, jy = [], []
            for i in range(n):
                jx.append(-sum(model.joints[k].length*sin_terms[k] for k in range(i, n)))
                jy.append(sum(model.joints[k].length*cos_terms[k] for k in range(i, n)))
            # u = inv(J J^T + lambda^2 I) * error; dq = J^T u
            damping = .06
            a = sum(z*z for z in jx)+damping*damping
            b = sum(x*y for x,y in zip(jx, jy))
            d = sum(z*z for z in jy)+damping*damping
            det = a*d-b*b
            ux, uy = (d*ex-b*ey)/det, (a*ey-b*ex)/det
            changes = [max(-.14, min(.14, x*ux+y*uy)) for x,y in zip(jx, jy)]
            # Small backtrack if a proposed update increases error near limits.
            factor = 1.0
            candidate = values
            for _ in range(7):
                proposal = [max(lo, min(hi, v+factor*delta)) for v, delta, (lo, hi) in zip(values, changes, limits)]
                if score(proposal) <= error+1e-12:
                    candidate = proposal
                    break
                factor *= .5
            if candidate == values:
                break
            values = candidate
    return IKResult(False, best, best_error, total_iter,
                    'no feasible joint-limited solution within tolerance')
