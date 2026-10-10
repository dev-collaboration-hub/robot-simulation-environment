"""M11-M15 offline tests: no external packages or paid CI required."""
from __future__ import annotations
import json
from math import pi
from pathlib import Path
import tempfile
import types
import sys
import unittest
from robot_sim import ArmModel, Vec2
from robot_sim.collision import CircleObstacle, BoxObstacle
from robot_sim.sensors import VirtualSensorArray, ray_distance
from robot_sim.world import WorldObject, Scene, World, example_scene
from robot_sim.gripper import Gripper
from robot_sim.integration import MotionPlannerAdapter, AdapterUnavailable
from robot_sim.environment import RobotEnvironment, Goal, scripted_pick_and_place, evaluate_scripted
from robot_sim.advanced import RobotLab


class SensorTests(unittest.TestCase):
    def test_range_clear(self):
        self.assertEqual(ray_distance(Vec2(0,0),0,(),3),3)

    def test_circle_ray(self):
        self.assertAlmostEqual(ray_distance(Vec2(0,0),0,(CircleObstacle(1,0,.2),),3),.8)

    def test_box_ray(self):
        self.assertAlmostEqual(ray_distance(Vec2(0,0),0,(BoxObstacle(1,-.1,2,.1),),3),1)

    def test_inside_box(self):
        self.assertEqual(ray_distance(Vec2(1.5,0),0,(BoxObstacle(1,-.5,2,.5),),3),0)

    def test_outside_obstacle(self):
        self.assertEqual(ray_distance(Vec2(0,0),pi/2,(CircleObstacle(1,0,.1),),3),3)

    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):ray_distance(Vec2(0,0),float('nan'),(),1)

    def test_update_rate(self):
        s=VirtualSensorArray(ArmModel.basic(),interval=.05)
        self.assertIsNone(s.sample(.02,(0,0)))
        self.assertIsNotNone(s.sample(.03,(0,0)))
        self.assertEqual(s.sample(.01,(0,0)).sequence,1)

    def test_encoder_measures_angles(self):
        s=VirtualSensorArray(ArmModel.basic(),interval=.01)
        o=s.sample(.01,(.2,-.4))
        self.assertEqual(o.joint_angles,(.2,-.4))

    def test_repeatable_noise(self):
        s=VirtualSensorArray(ArmModel.basic(),seed=7,interval=.01,encoder_stddev=.02,distance_stddev=.04)
        a=[s.sample(.01,(.1,-.2)) for _ in range(4)]
        s.reset()
        b=[s.sample(.01,(.1,-.2)) for _ in range(4)]
        self.assertEqual(a,b)
        self.assertNotEqual(a[0].joint_angles,(.1,-.2))

    def test_reset_clears_state(self):
        s=VirtualSensorArray(ArmModel.basic(),interval=.01)
        s.sample(.01,(0,0))
        s.reset()
        self.assertIsNone(s.latest)

    def test_invalid_settings(self):
        for kwargs in ({'interval':0},{'encoder_stddev':-1},{'distance_stddev':-1},
                       {'max_distance':0},{'seed':True}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                VirtualSensorArray(ArmModel.basic(),**kwargs)


class WorldTests(unittest.TestCase):
    def test_example_scenes(self):
        self.assertEqual(len(example_scene('empty').objects),0)
        self.assertTrue(example_scene('obstacles').obstacles)
        self.assertEqual(len(example_scene('pick_and_place').objects),1)

    def test_json_roundtrip(self):
        scene=example_scene('obstacles')
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'scene.json'
            scene.save(path)
            loaded=Scene.load(path)
            self.assertEqual(scene,loaded)

    def test_movement_and_reset(self):
        world=World(example_scene())
        before=world.get('block').position
        self.assertTrue(world.move('block',Vec2(.5,1.1)))
        self.assertEqual(world.positions()['block'],(.5,1.1))
        world.reset()
        self.assertEqual(world.get('block').position,before)

    def test_collision_rejects_drop(self):
        scene=Scene('blocked',(WorldObject('block',Vec2(0,0)),),
                    (CircleObstacle(1,0,.3),))
        world=World(scene)
        self.assertFalse(world.move('block',Vec2(1,0)))
        self.assertEqual(world.get('block').position,Vec2(0,0))

    def test_boundary_guard(self):
        w=World(example_scene())
        self.assertFalse(w.move('block',Vec2(20,0)))

    def test_duplicate_names(self):
        with self.assertRaises(ValueError):
            Scene('bad',(WorldObject('x',Vec2(0,0)),WorldObject('x',Vec2(1,0))))

    def test_validation_rejects_unknown_fields(self):
        with self.assertRaises(ValueError):
            Scene.from_dict({'objects':[],'unexpected':1})

    def test_reject_bad_box(self):
        with self.assertRaises(ValueError):
            Scene.from_dict({'objects':[],'obstacles':[{'type':'box','min_x':1,'max_x':2}]})

    def test_move_objects_are_independent(self):
        sc=Scene('test',(WorldObject('a',Vec2(0,0)),WorldObject('b',Vec2(1,0))))
        w=World(sc)
        self.assertFalse(w.move('a',Vec2(1,0)))
        self.assertTrue(w.move('a',Vec2(.5,0)))

    def test_invalid_scene_name(self):
        with self.assertRaises(ValueError):Scene('',())


class GripTests(unittest.TestCase):
    def setUp(self):
        self.world=World(Scene('test',(WorldObject('box',Vec2(.05,0)),)))
        self.gripper=Gripper(self.world)

    def test_close_follow_release(self):
        before=self.world.get('box').position
        result=self.gripper.close(Vec2(0,0))
        self.assertTrue(result.success)
        self.assertEqual(self.world.get('box').position,before)
        self.assertTrue(self.gripper.follow(Vec2(.4,.5)).success)
        self.assertEqual(self.world.get('box').position,Vec2(.45,.5))
        self.assertTrue(self.gripper.open().success)
        self.assertEqual(self.world.get('box').position,Vec2(.45,.5))
        self.assertIsNone(self.gripper.held)

    def test_out_of_reach(self):
        self.assertFalse(self.gripper.close(Vec2(2,2)).success)
        self.assertIsNone(self.gripper.held)

    def test_double_grip_rejected(self):
        self.gripper.close(Vec2(0,0))
        self.assertFalse(self.gripper.close(Vec2(0,0)).success)

    def test_empty_open_rejected(self):
        self.assertFalse(self.gripper.open().success)

    def test_blocked_carried_move(self):
        self.gripper.close(Vec2(0,0))
        self.assertFalse(self.gripper.follow(Vec2(10,10)).success)
        self.assertEqual(self.world.get('box').position,Vec2(.05,0))

    def test_reset_clears_grip(self):
        self.gripper.close(Vec2(0,0))
        self.gripper.reset()
        self.assertIsNone(self.gripper.held)
        self.assertTrue(self.gripper.opened)


class AdapterTests(unittest.TestCase):
    def test_local_fallback(self):
        m=ArmModel.basic()
        a=MotionPlannerAdapter(m)
        p=a.plan((0,0),(.5,-.5))
        self.assertEqual(p.end,(.5,-.5))
        self.assertEqual(a.backend,'local')

    def test_missing_external_is_explicit(self):
        with self.assertRaises(AdapterUnavailable):
            MotionPlannerAdapter(ArmModel.basic(),'nonexistent_robot_motion_adapter_example')

    def test_external_fixture_contract(self):
        module=types.ModuleType('test_motion_fixture')
        def plan(payload):
            self.assertEqual(payload['contract_version'],1)
            self.assertEqual(payload['unit'],'radians')
            return {'contract_version':1,'duration':3.,
                    'velocity_limits':[1.,1.],
                    'acceleration_limits':[1.,1.]}
        module.plan_joint_trajectory_v1=plan
        sys.modules[module.__name__]=module
        try:
            a=MotionPlannerAdapter(ArmModel.basic(),module.__name__)
            self.assertEqual(a.plan((0,0),(.5,.2)).duration,3.)
        finally:
            del sys.modules[module.__name__]

    def test_adapter_is_used_by_real_lab(self):
        module=types.ModuleType('test_real_lab_motion_fixture')
        calls=[]
        def plan(payload):
            calls.append(payload)
            return {'contract_version':1,'duration':7.,
                    'velocity_limits':[1.,1.],
                    'acceleration_limits':[1.,1.]}
        module.plan_joint_trajectory_v1=plan
        sys.modules[module.__name__]=module
        try:
            model=ArmModel.basic()
            lab=RobotLab(model,planner=MotionPlannerAdapter(model,module.__name__))
            result=lab.plan_target(Vec2(1.2,.9))
            self.assertTrue(result.success)
            self.assertEqual(lab.player.path.duration,7.)
            self.assertEqual(len(calls),1)
            lab.step_once()
            self.assertEqual(lab.player.path.duration,7.)
        finally:
            del sys.modules[module.__name__]

    def test_external_overspeed_rejected(self):
        module=types.ModuleType('test_bad_motion_fixture')
        module.plan_joint_trajectory_v1=lambda payload: {
            'contract_version':1,'duration':.01,
            'velocity_limits':[1.,1.],'acceleration_limits':[1.,1.]}
        sys.modules[module.__name__]=module
        try:
            with self.assertRaises(ValueError):
                MotionPlannerAdapter(ArmModel.basic(),module.__name__).plan((0,0),(1,0))
        finally:
            del sys.modules[module.__name__]


class EnvironmentTests(unittest.TestCase):
    def test_initial_observation(self):
        env=RobotEnvironment()
        obs=env.reset()
        self.assertEqual(obs['held'],None)
        self.assertIn('distance_sensor',obs)
        self.assertEqual(obs['sensor_sequence'],1)

    def test_noop_is_deterministic(self):
        def run():
            env=RobotEnvironment(sensor_seed=123)
            return [env.step()[0] for _ in range(8)]
        self.assertEqual(run(),run())

    def test_end_episode_truncates(self):
        env=RobotEnvironment(max_steps=2)
        env.step()
        o,r,terminated,truncated,info=env.step()
        self.assertTrue(truncated)
        self.assertTrue(o['done'])
        with self.assertRaises(RuntimeError):env.step()

    def test_invalid_action(self):
        env=RobotEnvironment()
        with self.assertRaises(ValueError):env.step({'type':'wrong'})

    def test_unreachable_target_is_rejected(self):
        env=RobotEnvironment()
        _,_,_,_,info=env.step({'type':'move_to','x':9,'y':9})
        self.assertFalse(info['accepted'])

    def test_wrong_goal_is_rejected(self):
        with self.assertRaises(ValueError):
            RobotEnvironment(goal=Goal('missing',Vec2(1,1)))

    def test_episode_reset_replay(self):
        env=RobotEnvironment(sensor_seed=9)
        original=env.reset()
        for _ in range(4):env.step()
        self.assertEqual(env.reset(),original)

    def test_scripted_pick_and_place(self):
        env=RobotEnvironment()
        success,observation=scripted_pick_and_place(env)
        self.assertTrue(success,observation)
        self.assertLessEqual(
            env.world.get('block').position.distance_to(env.goal.destination),
            env.goal.tolerance)
        self.assertIsNone(env.gripper.held)

    def test_benchmark_reproducible(self):
        a=evaluate_scripted(2)
        b=evaluate_scripted(2)
        self.assertEqual(a,b)
        self.assertEqual(a['successes'],2)


if __name__=='__main__':
    unittest.main()
