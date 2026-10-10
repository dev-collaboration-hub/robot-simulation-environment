"""M15 deterministic, offline robot environment with reset/step/observe/reward/done.

Action API (dict):
  {'type':'move_to', 'x':float, 'y':float}
  {'type':'set_joints', 'angles':[...]}
  {'type':'grip'} | {'type':'release'} | {'type':'noop'}

Every step advances exactly one simulation tick. All decisions use virtual
sensor readings and status rather than exposing hidden physics internals.
"""
from __future__ import annotations
from dataclasses import dataclass
from .math3d import Vec2
from .arm import ArmModel
from .world import World, Scene, example_scene
from .collision import CollisionWorld, CircleObstacle
from .advanced import RobotLab
from .gripper import Gripper
from .sensors import VirtualSensorArray


@dataclass(frozen=True)
class Goal:
    object_name: str
    destination: Vec2
    tolerance: float = .12

    def __post_init__(self):
        if not isinstance(self.object_name,str) or not self.object_name.strip():
            raise ValueError('goal requires an object name')
        if not isinstance(self.destination,Vec2):
            raise TypeError('goal destination must be Vec2')
        if not 0 < self.tolerance <= 1.:
            raise ValueError('invalid goal tolerance')


class RobotEnvironment:
    def __init__(self, model: ArmModel | None = None,
                 scene: Scene | None = None, goal: Goal | None = None,
                 max_steps: int = 2500, sensor_seed: int = 41):
        self.model=model or ArmModel.basic()
        self.scene=scene or example_scene('pick_and_place')
        self.world=World(self.scene)
        self.goal=goal or Goal('block',Vec2(.5,1.3))
        if self.goal.object_name not in self.world.objects:
            raise ValueError('goal object is not in scene')
        if not self.scene.contains(self.goal.destination,self.world.get(self.goal.object_name).radius):
            raise ValueError('goal outside world bounds')
        if not isinstance(max_steps,int) or isinstance(max_steps,bool) or max_steps<1:
            raise ValueError('max_steps must be positive integer')
        self.max_steps=max_steps
        self.sensor_seed=sensor_seed
        self.reset()

    def reset(self):
        self.world.reset()
        self.lab=RobotLab(self.model,CollisionWorld(self.scene.obstacles))
        self.gripper=Gripper(self.world)
        self.sensors=VirtualSensorArray(self.model,seed=self.sensor_seed,interval=.01)
        self.steps=0
        self.done=False
        self.success=False
        self._ever_grasped=False
        self.last_event='reset'
        self.sensors.sample(self.lab.dt,self.lab.physics.angles,self._visible_objects())
        return self.observe()

    def _visible_objects(self):
        # No privileged state: this list is provided to a simulated range sensor.
        other = (CircleObstacle(o.position.x,o.position.y,o.radius)
                 for o in self.world.objects.values() if o.name != self.gripper.held)
        return self.scene.obstacles+tuple(other)

    def observe(self):
        latest=self.sensors.latest
        if latest is None:
            raise RuntimeError('sensor array has not produced a reading')
        estimated_tool=self.model.forward(tuple(
            max(j.minimum,min(j.maximum,a))
            for j,a in zip(self.model.joints,latest.joint_angles))).tool
        status=self.lab.snapshot()
        return {
            'time':round(self.steps*self.lab.dt,8),
            'angles_sensor':tuple(round(a,8) for a in latest.joint_angles),
            'tool_estimate':(round(estimated_tool.x,6),round(estimated_tool.y,6)),
            'distance_sensor':round(latest.distance,6),
            'sensor_sequence':latest.sequence,
            'held':self.gripper.held,
            'object_positions':self.world.positions(),
            'planning':status.planning,
            'controller_error':round(status.error,7),
            'collision':status.collision,
            'last_event':self.last_event,
            'done':self.done,
        }

    def step(self, action=None):
        if self.done:
            raise RuntimeError('episode ended; call reset')
        action={'type':'noop'} if action is None else action
        if not isinstance(action,dict) or 'type' not in action:
            raise ValueError('action must be a dict with type')
        kind=action['type']
        info={'accepted':True,'message':''}
        if kind=='move_to':
            if set(action)!={'type','x','y'}:
                raise ValueError('move_to needs only x and y')
            result=self.lab.plan_target(Vec2(action['x'],action['y']))
            info={'accepted':result.success,'message':result.reason}
            self.last_event='planned XY move' if result.success else 'rejected IK: '+result.reason
        elif kind=='set_joints':
            if set(action)!={'type','angles'}:
                raise ValueError('set_joints needs only angles')
            try:
                self.lab.command_joints(action['angles'])
                self.last_event='joint plan accepted'
            except ValueError as exc:
                info={'accepted':False,'message':str(exc)}
                self.last_event='joint plan rejected: '+str(exc)
        elif kind=='grip':
            if set(action)!={'type'}:
                raise ValueError('grip has no additional properties')
            result=self.gripper.close(self.model.forward(self.lab.physics.angles).tool)
            info={'accepted':result.success,'message':result.reason}
            self.last_event=result.reason
            if result.success:self._ever_grasped=True
        elif kind=='release':
            if set(action)!={'type'}:
                raise ValueError('release has no additional properties')
            result=self.gripper.open()
            info={'accepted':result.success,'message':result.reason}
            self.last_event=result.reason
        elif kind=='noop':
            if set(action)!={'type'}:
                raise ValueError('noop has no additional properties')
        else:
            raise ValueError('unknown action type')

        previous=self.lab.physics.angles
        if not self.lab.paused:
            self.lab.step_once()
        elif self.lab.player.path is not None and (
                not self.lab.player.finished or self.lab.snapshot().error > .002):
            # Continue feedback settling after path time reaches its endpoint.
            # Otherwise a motor can freeze a few centimetres short of goal.
            self.lab.step_once()
        elif self.gripper.held is None:
            pass
        # Keep attached object physically at the current end-effector position.
        if self.gripper.held is not None:
            tool=self.model.forward(self.lab.physics.angles).tool
            follow=self.gripper.follow(tool)
            if not follow.success:
                self.lab.physics.angles=previous
                self.lab.physics.velocities=(0.,)*len(previous)
                self.lab.player.cancel()
                self.lab.pause()
                self.last_event='blocked carried-object movement'
                info={'accepted':False,'message':self.last_event}
        self.steps+=1
        self.world.ticks+=1
        self.sensors.sample(self.lab.dt,self.lab.physics.angles,self._visible_objects())
        obj=self.world.get(self.goal.object_name)
        self.success=(self._ever_grasped and self.gripper.held is None and
                      obj.position.distance_to(self.goal.destination) <= self.goal.tolerance)
        terminated=self.success or bool(self.lab.collision)
        truncated=self.steps>=self.max_steps and not terminated
        self.done=terminated or truncated
        # Small time cost, reward for successful grasp/release, large goal reward.
        reward= -0.001
        if kind=='grip' and info['accepted']:reward+=1.
        if self.success:reward+=10.
        if self.lab.collision:reward-=5.
        if not info['accepted']:reward-=.1
        info.update({'success':self.success,'terminated':terminated,'truncated':truncated})
        return self.observe(),reward,terminated,truncated,info


def scripted_pick_and_place(env: RobotEnvironment, limit: int = 2400):
    """Deterministic scripted policy for regression benchmarking, not AI training."""
    obs=env.reset()
    source=env.world.get(env.goal.object_name).position

    def move_to(point):
        nonlocal obs
        obs,_,terminated,truncated,info=env.step({'type':'move_to','x':point.x,'y':point.y})
        if not info['accepted'] or terminated or truncated:
            return False
        for _ in range(limit):
            obs,_,terminated,truncated,info=env.step()
            tool=env.model.forward(env.lab.physics.angles).tool
            if tool.distance_to(point)<=.03 and not env.lab.snapshot().planning:
                return True
            if terminated or truncated:
                return False
        return False

    if not move_to(source):
        return False, env.observe()
    obs,_,t,tr,info=env.step({'type':'grip'})
    if not info['accepted'] or t or tr:
        return False,obs
    if not move_to(env.goal.destination):
        return False,env.observe()
    obs,_,t,tr,info=env.step({'type':'release'})
    return env.success, obs


def evaluate_scripted(trials: int = 5):
    if not isinstance(trials,int) or isinstance(trials,bool) or not 1<=trials<=100:
        raise ValueError('trial count must lie in 1..100')
    results=[]
    for seed in range(trials):
        env=RobotEnvironment(sensor_seed=seed)
        ok,obs=scripted_pick_and_place(env)
        results.append({'trial':seed,'success':ok,'steps':env.steps,'last_event':obs['last_event']})
    return {'trials':trials,'successes':sum(x['success'] for x in results),
            'success_rate':sum(x['success'] for x in results)/trials,
            'results':results}
