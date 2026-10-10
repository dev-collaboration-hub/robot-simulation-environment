"""M5-M10 advanced simulation acceptance regressions."""
from __future__ import annotations
import math
import unittest
from robot_sim import ArmModel, JointSpec, Vec2
from robot_sim.render3d import Camera, Vec3, link_box
from robot_sim.kinematics import solve_ik
from robot_sim.trajectory import plan_joint_path, TrajectoryPlayer
from robot_sim.physics import BodySpec, PlanarPhysics
from robot_sim.collision import (CircleObstacle, BoxObstacle, CollisionWorld,
                                 point_segment_distance, segments_intersect,
                                 segment_box_distance, segment_segment_distance)
from robot_sim.actuators import MotorConfig, VirtualMotors
from robot_sim.advanced import RobotLab


class RendererMathTests(unittest.TestCase):
    def test_origin_projection(self):
        x,y,depth=Camera().project(Vec3(0,0,0),800,600)
        self.assertEqual((x,y,depth),(400,300,6.5))

    def test_projection_changes_with_yaw(self):
        camera=Camera()
        before=camera.project(Vec3(1,0,0),900,700)
        camera.orbit(50,0)
        self.assertNotEqual(before,camera.project(Vec3(1,0,0),900,700))

    def test_zoom(self):
        camera=Camera()
        before=camera.project(Vec3(1,0,0),900,700)
        camera.change_zoom(3)
        after=camera.project(Vec3(1,0,0),900,700)
        self.assertGreater(abs(after[0]-450),abs(before[0]-450))

    def test_camera_pitch_clamp(self):
        camera=Camera()
        camera.orbit(0,9999)
        self.assertEqual(camera.pitch,1.4)
        camera.orbit(0,-9999)
        self.assertEqual(camera.pitch,-1.4)

    def test_parallel_projection(self):
        camera=Camera(perspective=False)
        a=camera.project(Vec3(0,1,0),600,500)
        b=camera.project(Vec3(0,1,-1),600,500)
        self.assertIsNotNone(a)
        self.assertIsNotNone(b)

    def test_camera_invalid(self):
        for kwargs in ({'zoom':0},{'distance':0},{'pitch':9},{'yaw':float('nan')}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):Camera(**kwargs)

    def test_behind_camera(self):
        self.assertIsNone(Camera().project(Vec3(0,0,999),600,500))

    def test_link_geometry(self):
        corners=link_box(Vec2(0,0),Vec2(2,0),.1,.2)
        self.assertEqual(len(corners),8)
        self.assertEqual({round(v.z,5) for v in corners},{-.1,.1})

    def test_invalid_link(self):
        with self.assertRaises(ValueError):link_box(Vec2(0,0),Vec2(0,0))


class KinematicsTests(unittest.TestCase):
    def setUp(self):self.model=ArmModel.basic()

    def test_known_target(self):
        r=solve_ik(self.model,Vec2(1.2,.9))
        self.assertTrue(r.success)
        self.assertAlmostEqual(r.angles[0],0)
        self.assertAlmostEqual(r.angles[1],math.pi/2)

    def test_reachable_negative_y(self):
        r=solve_ik(self.model,Vec2(1.2,-.9))
        self.assertTrue(r.success)
        self.assertLessEqual(r.error,.001)

    def test_beyond_reach(self):
        r=solve_ik(self.model,Vec2(8,0))
        self.assertFalse(r.success)
        self.assertIn('reach',r.reason)

    def test_inner_workspace_unreachable(self):
        self.assertFalse(solve_ik(self.model,Vec2(0,0)).success)

    def test_singularity_near_target(self):
        r=solve_ik(self.model,Vec2(2.09,0.03))
        self.assertTrue(r.success)

    def test_limits_respected(self):
        m=ArmModel((JointSpec('a',1,minimum=-.2,maximum=.2),
                    JointSpec('b',1,minimum=-.2,maximum=.2)))
        r=solve_ik(m,Vec2(0,2))
        self.assertFalse(r.success)
        self.assertTrue(all(j.minimum<=v<=j.maximum for j,v in zip(m.joints,r.angles)))

    def test_six_joints(self):
        m=ArmModel.extended()
        target=m.forward((.2,-.4,.2,.3,-.2,.1)).tool
        r=solve_ik(m,target)
        self.assertTrue(r.success)
        self.assertLess(r.error,.001)

    def test_bad_tolerance(self):
        with self.assertRaises(ValueError):solve_ik(self.model,Vec2(1,1),tolerance=0)

    def test_bad_seed(self):
        with self.assertRaises(ValueError):solve_ik(self.model,Vec2(1,1),seed=(0,))


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.model=ArmModel.basic()
        self.path=plan_joint_path(self.model,(0,0),(.8,-.7))

    def test_endpoints(self):
        self.assertEqual(self.path.sample(-1),(0,0))
        self.assertEqual(self.path.sample(self.path.duration+1),(.8,-.7))

    def test_monotonic(self):
        p=[self.path.sample(self.path.duration*i/100)[0] for i in range(101)]
        self.assertEqual(p,sorted(p))

    def test_speed_acceleration_bounds(self):
        for i in range(101):
            t=self.path.duration*i/100
            for speed,bound in zip(self.path.velocity(t),self.path.velocity_limits):
                self.assertLessEqual(abs(speed),bound+1e-9)
            for accel,bound in zip(self.path.acceleration(t),self.path.acceleration_limits):
                self.assertLessEqual(abs(accel),bound+1e-9)

    def test_endpoint_velocity(self):
        self.assertEqual(self.path.velocity(0),(0.,0.))
        self.assertEqual(self.path.velocity(self.path.duration),(0.,0.))

    def test_replanning(self):
        player=TrajectoryPlayer()
        player.load(self.path)
        self.assertEqual(player.step(self.path.duration),self.path.end)
        self.assertTrue(player.finished)
        path2=plan_joint_path(self.model,(.8,-.7),(0,0))
        player.load(path2)
        self.assertFalse(player.finished)
        self.assertEqual(player.step(path2.duration),(0,0))
        player.cancel()
        self.assertIsNone(player.step(.01))

    def test_still_trajectory(self):
        p=plan_joint_path(self.model,(0,0),(0,0))
        self.assertGreater(p.duration,0)
        self.assertEqual(p.sample(1),(0,0))

    def test_bad_acceleration(self):
        with self.assertRaises(ValueError):plan_joint_path(self.model,(0,0),(1,0),accel_limit=0)


class PhysicsTests(unittest.TestCase):
    def setUp(self):self.model=ArmModel.basic()

    def test_zero_force_zero_gravity(self):
        p=PlanarPhysics(self.model,gravity=0.)
        self.assertEqual(p.step((0,0),.01),(0.,0.))

    def test_gravity_affects_horizontal_arm(self):
        p=PlanarPhysics(self.model,gravity=9.81)
        self.assertLess(p.step((0,0),.01)[0],0)

    def test_gravity_off_vs_on(self):
        a=PlanarPhysics(self.model,gravity=0.)
        b=PlanarPhysics(self.model,gravity=9.81)
        a.step((0,0),.01)
        b.step((0,0),.01)
        self.assertNotEqual(a.angles,b.angles)

    def test_torque_moves_joint(self):
        p=PlanarPhysics(self.model,gravity=0.)
        self.assertGreater(p.step((3,0),.01)[0],0)

    def test_reset(self):
        p=PlanarPhysics(self.model)
        p.step((0,0),.01)
        p.reset()
        self.assertEqual(p.angles,self.model.initial_angles)
        self.assertEqual(p.velocities,(0,0))

    def test_limits_hard_stop(self):
        m=ArmModel((JointSpec('j1',1,minimum=-.01,maximum=.01),JointSpec('j2',1)))
        p=PlanarPhysics(m,gravity=0.)
        for _ in range(10):p.step((100,0),.01)
        self.assertEqual(p.angles[0],.01)
        self.assertEqual(p.velocities[0],0)

    def test_validation(self):
        with self.assertRaises(ValueError):BodySpec(mass=0)
        with self.assertRaises(ValueError):PlanarPhysics(self.model,gravity=-1)
        with self.assertRaises(ValueError):PlanarPhysics(self.model,bodies=(BodySpec(),))
        with self.assertRaises(ValueError):PlanarPhysics(self.model).step((0,),.01)
        with self.assertRaises(ValueError):PlanarPhysics(self.model).step((0,0),.5)


class CollisionTests(unittest.TestCase):
    def setUp(self):self.model=ArmModel.basic()

    def test_point_segment_distance(self):
        self.assertEqual(point_segment_distance(Vec2(1,1),Vec2(0,0),Vec2(2,0)),1)

    def test_crossing_segments(self):
        self.assertTrue(segments_intersect(Vec2(0,0),Vec2(1,1),Vec2(0,1),Vec2(1,0)))

    def test_collinear_overlap(self):
        self.assertTrue(segments_intersect(Vec2(0,0),Vec2(2,0),Vec2(1,0),Vec2(3,0)))

    def test_segment_distance(self):
        self.assertAlmostEqual(segment_segment_distance(Vec2(0,0),Vec2(2,0),Vec2(0,1),Vec2(2,1)),1)

    def test_segment_box_distance(self):
        b=BoxObstacle(1,1,2,2)
        self.assertAlmostEqual(segment_box_distance(Vec2(0,0),Vec2(3,0),b),1)
        self.assertEqual(segment_box_distance(Vec2(0,1.5),Vec2(3,1.5),b),0)

    def test_circle_obstacle_hit(self):
        w=CollisionWorld((CircleObstacle(.7,.02,.05),))
        self.assertTrue(w.inspect(self.model,(0,0)).collided)

    def test_circle_clear(self):
        w=CollisionWorld((CircleObstacle(.7,1,.05),))
        self.assertFalse(w.inspect(self.model,(0,0)).collided)

    def test_box_collision(self):
        self.assertTrue(CollisionWorld((BoxObstacle(.3,-.1,.5,.1),)).inspect(self.model,(0,0)).collided)

    def test_sweep_catches_motion_obstacle(self):
        w=CollisionWorld((CircleObstacle(.6,.65,.06),))
        start=(0,0)
        end=(math.pi/2,0)
        self.assertFalse(w.inspect(self.model,start).collided)
        self.assertFalse(w.inspect(self.model,end).collided)
        self.assertTrue(w.sweep(self.model,start,end).collided)

    def test_no_tunnel_on_large_motion(self):
        w=CollisionWorld((CircleObstacle(.4,.6,.02),))
        self.assertTrue(w.sweep(self.model,(-1.2,0),(1.2,0)).collided)

    def test_self_collision(self):
        m=ArmModel((JointSpec(str(i),1) for i in range(4)))
        w=CollisionWorld()
        self.assertTrue(w.inspect(m,(0,math.pi/2,math.pi/2,math.pi/2)).collided)

    def test_collision_validation(self):
        with self.assertRaises(ValueError):CircleObstacle(0,0,0)
        with self.assertRaises(ValueError):BoxObstacle(1,0,0,1)
        with self.assertRaises(ValueError):CollisionWorld(link_radius=0)
        with self.assertRaises(TypeError):CollisionWorld((object(),))


class MotorsTests(unittest.TestCase):
    def setUp(self):self.model=ArmModel.basic()

    def test_motor_moves(self):
        motor=VirtualMotors(self.model)
        motor.command((.3,0))
        for _ in range(50):motor.step(.01)
        self.assertGreater(motor.physics.angles[0],.05)

    def test_motor_settles(self):
        motor=VirtualMotors(self.model)
        motor.command((.3,-.4))
        for _ in range(700):motor.step(.01)
        self.assertLess(abs(motor.physics.angles[0]-.3),.05)
        self.assertLess(abs(motor.physics.angles[1]+.4),.05)

    def test_torque_saturation(self):
        motor=VirtualMotors(self.model,config=MotorConfig(torque_limit=1))
        motor.command((2,2))
        motor.step(.01)
        self.assertTrue(all(abs(t)<=1 for t in motor.last_torques))

    def test_integral_bounded(self):
        motor=VirtualMotors(self.model,config=MotorConfig(ki=1000,torque_limit=.01))
        motor.command((2,2))
        for _ in range(100):motor.step(.01)
        self.assertTrue(all(abs(x)<=5 for x in motor.integral))

    def test_motor_reset(self):
        motor=VirtualMotors(self.model)
        motor.command((1,0))
        motor.step(.01)
        motor.reset()
        self.assertEqual(motor.physics.angles,(0,0))
        self.assertEqual(motor.target,(0,0))

    def test_config_validation(self):
        with self.assertRaises(ValueError):MotorConfig(torque_limit=0)
        with self.assertRaises(ValueError):MotorConfig(kp=-1)
        with self.assertRaises(ValueError):VirtualMotors(self.model).step(.1)


class IntegratedLabTests(unittest.TestCase):
    def test_target_plan_and_motion(self):
        lab=RobotLab()
        result=lab.plan_target(Vec2(1.2,.9))
        self.assertTrue(result.success)
        lab.resume()
        for _ in range(500):lab.step_once()
        self.assertLess(lab.model.forward(lab.physics.angles).tool.distance_to(Vec2(1.2,.9)),.07)
        self.assertFalse(lab.snapshot().planning)

    def test_blocked_path(self):
        lab=RobotLab(world=CollisionWorld((CircleObstacle(.7,.6,.14),)))
        result=lab.plan_target(Vec2(.1,1.9))
        self.assertFalse(result.success)
        self.assertIn('blocked',result.reason)

    def test_collision_stops_unsafe_motor_trial(self):
        lab=RobotLab(world=CollisionWorld((CircleObstacle(1.19,-.08,.04),)))
        lab.set_gravity(True)
        lab.resume()
        for _ in range(100):
            lab.step_once()
            if lab.collision:break
        self.assertTrue(lab.collision)
        self.assertTrue(lab.paused)

    def test_pause_step_reset(self):
        lab=RobotLab()
        self.assertTrue(lab.snapshot().paused)
        self.assertEqual(lab.advance(.1),0)
        lab.resume()
        self.assertEqual(lab.advance(.02),2)
        lab.pause()
        self.assertEqual(lab.advance(.05),0)
        lab.step_once()
        self.assertEqual(lab.snapshot().step,3)
        lab.reset()
        self.assertEqual(lab.snapshot().step,0)
        self.assertTrue(lab.snapshot().paused)

    def test_same_seed_determinism(self):
        def run():
            lab=RobotLab()
            lab.command_joints((.6,-.3))
            lab.resume()
            for _ in range(250):lab.step_once()
            return lab.snapshot()
        self.assertEqual(run(),run())

    def test_bad_command(self):
        with self.assertRaises(ValueError):RobotLab().command_joints((9,0))

    def test_gravity_toggle(self):
        lab=RobotLab()
        lab.set_gravity(True)
        self.assertEqual(lab.physics.gravity,9.81)
        lab.set_gravity(False)
        self.assertEqual(lab.physics.gravity,0)


if __name__=='__main__':unittest.main()
