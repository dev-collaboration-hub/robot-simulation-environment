"""Deterministic integrated M5-M10 laboratory: IK -> planned path -> PID -> dynamics -> collision.

The arm itself remains *planar*. The software renderer supplies 3D views of
3D link solids. Physics is educational uncoupled joint gravity/torque.
"""
from __future__ import annotations
from dataclasses import dataclass
from .arm import ArmModel
from .math3d import Vec2, finite
from .kinematics import solve_ik, IKResult
from .integration import MotionPlannerAdapter
from .trajectory import TrajectoryPlayer, plan_joint_path
from .collision import CollisionWorld, CollisionReport
from .physics import PlanarPhysics
from .actuators import VirtualMotors


@dataclass(frozen=True)
class LabSnapshot:
    step: int
    time: float
    angles: tuple[float, ...]
    target: tuple[float, ...]
    error: float
    torques: tuple[float, ...]
    collision: str
    planning: bool
    paused: bool


class RobotLab:
    def __init__(self, model: ArmModel | None = None, world: CollisionWorld | None = None,
                 dt: float = .01, planner: MotionPlannerAdapter | None = None):
        self.model = model or ArmModel.basic()
        self.world = world or CollisionWorld()
        self.planner = planner or MotionPlannerAdapter(self.model)
        if self.planner.model != self.model:
            raise ValueError('planner model and simulator model must match')
        self.dt = finite(dt, 'dt')
        if not 0 < self.dt <= .02:
            raise ValueError('lab requires a fixed dt in (0, .02]')
        self.physics = PlanarPhysics(self.model,gravity=0.)
        self.motors = VirtualMotors(self.model, self.physics)
        self.player = TrajectoryPlayer()
        self.reset()

    def reset(self):
        self.motors.reset()
        self.player.cancel()
        self.step_index = 0
        self.paused = True
        self.collision = ''
        self._remainder = 0.

    def plan_target(self, target: Vec2) -> IKResult:
        result = solve_ik(self.model,target,seed=self.physics.angles)
        if not result.success:
            return result
        # Reject collisions at samples throughout the proposed trajectory.
        path = self.planner.plan(self.physics.angles, result.angles)
        previous = path.start
        count = max(2, int(path.duration/self.dt)+1)
        for k in range(1,count+1):
            current = path.sample(path.duration*k/count)
            report = self.world.sweep(self.model,previous,current)
            if report.collided:
                return IKResult(False, self.physics.angles, result.error, result.iterations,
                                'trajectory blocked: '+report.detail)
            previous = current
        self.player.load(path)
        self.collision = ''
        return result

    def command_joints(self, target):
        target = self.model.validate_angles(tuple(target))
        path = self.planner.plan(self.physics.angles, target)
        previous = path.start
        count = max(2,int(path.duration/self.dt)+1)
        for k in range(1,count+1):
            current = path.sample(path.duration*k/count)
            hit = self.world.sweep(self.model,previous,current)
            if hit.collided:
                raise ValueError('trajectory blocked: '+hit.detail)
            previous = current
        self.player.load(path)
        self.collision = ''

    def set_gravity(self, enabled: bool):
        self.physics.gravity = 9.81 if enabled else 0.

    def step_once(self):
        before = self.physics.angles
        wanted = self.player.step(self.dt)
        if wanted is not None:
            self.motors.command(wanted)
        prior_velocity = self.physics.velocities
        previous_integral = self.motors.integral[:]
        after = self.motors.step(self.dt)
        hit = self.world.sweep(self.model,before,after)
        if hit.collided:
            # Hard-stop before accepting an invalid trial. No contact dynamics.
            self.physics.angles = before
            self.physics.velocities = (0.,)*len(before)
            self.motors.integral = previous_integral
            self.motors.command(before)
            self.player.cancel()
            self.collision = hit.detail
            self.paused = True
        self.step_index += 1
        return self.snapshot()

    def resume(self):
        self.paused = False

    def pause(self):
        self.paused = True
        self._remainder = 0.

    def advance(self, elapsed: float):
        elapsed = finite(elapsed, 'elapsed')
        if elapsed < 0:
            raise ValueError('elapsed cannot be negative')
        if self.paused:
            return 0
        self._remainder += min(elapsed,.25)
        steps=0
        while self._remainder+1e-12 >= self.dt and not self.paused:
            self.step_once()
            self._remainder -= self.dt
            steps += 1
        return steps

    def snapshot(self):
        a, t = self.physics.angles, self.motors.target
        return LabSnapshot(self.step_index,self.step_index*self.dt,a,t,
                           sum((x-y)**2 for x,y in zip(a,t))**.5,
                           self.motors.last_torques,self.collision,
                           self.player.path is not None and not self.player.finished,self.paused)
