"""M1-M4 automated checks: run python -m unittest discover -s tests -v."""
from __future__ import annotations
import json
from math import pi
from pathlib import Path
import unittest
from robot_sim import ArmModel, JointSpec, Simulator, Transform2, Vec2

ROOT = Path(__file__).resolve().parents[1]


class GeometryTests(unittest.TestCase):
    def test_vector_addition(self):
        self.assertEqual(Vec2(1, 2)+Vec2(3, -1), Vec2(4, 1))

    def test_vector_difference(self):
        self.assertEqual(Vec2(5, 2)-Vec2(2, -1), Vec2(3, 3))

    def test_vector_length(self):
        self.assertAlmostEqual(Vec2(0, 0).distance_to(Vec2(3, 4)), 5)

    def test_scale(self):
        self.assertEqual(Vec2(1, -2).scale(3), Vec2(3, -6))

    def test_rotation(self):
        p = Transform2(pi/2).apply(Vec2(2, 0))
        self.assertAlmostEqual(p.x, 0, places=12)
        self.assertAlmostEqual(p.y, 2, places=12)

    def test_translation(self):
        self.assertEqual(Transform2(offset=Vec2(2, -1)).apply(Vec2(1, 1)), Vec2(3, 0))

    def test_compose(self):
        a = Transform2(pi/2, Vec2(1, 2))
        b = Transform2(pi/2, Vec2(2, 0))
        p = a.compose(b).apply(Vec2(1, 0))
        expected = a.apply(b.apply(Vec2(1, 0)))
        self.assertAlmostEqual(p.x, expected.x)
        self.assertAlmostEqual(p.y, expected.y)

    def test_inverse(self):
        t = Transform2(pi/3, Vec2(2, 5))
        p = t.inverse().apply(t.apply(Vec2(-1, 2)))
        self.assertAlmostEqual(p.x, -1)
        self.assertAlmostEqual(p.y, 2)

    def test_matrix(self):
        m = Transform2(0, Vec2(3, 4)).matrix()
        self.assertEqual(m, ((1, 0, 3), (0, 1, 4), (0, 0, 1)))

    def test_reject_non_finite(self):
        for bad in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                Vec2(bad, 0)


class ArmTests(unittest.TestCase):
    def setUp(self):
        self.model = ArmModel.basic()

    def test_straight(self):
        p = self.model.forward((0, 0))
        self.assertAlmostEqual(p.tool.x, 2.1)
        self.assertAlmostEqual(p.tool.y, 0)
        self.assertEqual(len(p.positions), 3)

    def test_right_angle(self):
        p = self.model.forward((pi/2, 0))
        self.assertAlmostEqual(p.tool.x, 0, places=12)
        self.assertAlmostEqual(p.tool.y, 2.1)

    def test_elbow_bend(self):
        p = self.model.forward((0, pi/2))
        self.assertAlmostEqual(p.tool.x, 1.2)
        self.assertAlmostEqual(p.tool.y, 0.9)

    def test_negative_elbow(self):
        p = self.model.forward((0, -pi/2))
        self.assertAlmostEqual(p.tool.y, -0.9)

    def test_base_offset(self):
        model = ArmModel(self.model.joints, Vec2(1, -1))
        self.assertAlmostEqual(model.forward((0, 0)).tool.x, 3.1)
        self.assertAlmostEqual(model.forward((0, 0)).tool.y, -1)

    def test_reject_short_angles(self):
        with self.assertRaises(ValueError): self.model.forward((0,))

    def test_reject_over_joint_limit(self):
        with self.assertRaises(ValueError): self.model.forward((pi+0.1, 0))

    def test_reject_negative_length(self):
        with self.assertRaises(ValueError): JointSpec('bad', -1)

    def test_reject_zero_speed(self):
        with self.assertRaises(ValueError): JointSpec('bad', 1, max_speed=0)

    def test_reject_bad_limits(self):
        with self.assertRaises(ValueError): JointSpec('bad', 1, 2, 1)

    def test_reject_bad_initial(self):
        with self.assertRaises(ValueError): JointSpec('bad', 1, -1, 1, 2)

    def test_reject_duplicate_names(self):
        with self.assertRaises(ValueError): ArmModel((JointSpec('a', 1), JointSpec('a', 1)))

    def test_reject_unsupported_dof(self):
        for count in (1, 7):
            with self.subTest(count=count), self.assertRaises(ValueError):
                ArmModel(tuple(JointSpec(str(x), 1) for x in range(count)))

    def test_two_and_six_joint_configs(self):
        for name, count in [('two_joint.json', 2), ('six_joint.json', 6)]:
            with self.subTest(name=name):
                model = ArmModel.from_json(ROOT/'examples'/name)
                self.assertEqual(len(model.joints), count)
                self.assertEqual(len(model.forward(model.initial_angles).positions), count+1)

    def test_reject_bad_json_structure(self):
        for bad in ({}, {'joints': []}, {'joints': [{}, {}]}, {'joints': [], 'wrong': 1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ArmModel.from_dict(bad)

    def test_reject_invalid_angle_type(self):
        with self.assertRaises(ValueError): self.model.forward((0, True))


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        self.sim = Simulator(dt=0.01)

    def test_initial_paused(self):
        self.assertTrue(self.sim.paused)
        self.assertEqual(self.sim.steps, 0)

    def test_initial_pose(self):
        self.assertEqual(self.sim.snapshot().angles, (0, 0))

    def test_command_validates_limits(self):
        with self.assertRaises(ValueError): self.sim.command((5, 0))

    def test_advance_paused(self):
        self.assertEqual(self.sim.advance(0.2), 0)
        self.assertEqual(self.sim.steps, 0)

    def test_step_while_paused(self):
        self.sim.command((1, 0))
        state = self.sim.step_once()
        self.assertEqual(state.step, 1)
        self.assertTrue(state.paused)
        self.assertGreater(state.angles[0], 0)

    def test_elapsed_exact_ticks(self):
        self.sim.resume()
        self.assertEqual(self.sim.advance(0.05), 5)
        self.assertEqual(self.sim.steps, 5)

    def test_accumulator(self):
        self.sim.resume()
        self.assertEqual(self.sim.advance(0.006), 0)
        self.assertEqual(self.sim.advance(0.004), 1)

    def test_wall_time_cap(self):
        self.sim.resume()
        self.assertEqual(self.sim.advance(999), 25)

    def test_pause_resets_leftover_time(self):
        self.sim.resume()
        self.sim.advance(0.009)
        self.sim.pause()
        self.sim.resume()
        self.assertEqual(self.sim.advance(0.002), 0)

    def test_speed_limit(self):
        self.sim.command((2, 0))
        snap = self.sim.step_once()
        self.assertAlmostEqual(snap.angles[0], pi*.01)

    def test_target_arrival(self):
        self.sim.command((0.02, 0))
        self.assertAlmostEqual(self.sim.step_once().angles[0], 0.02)

    def test_time(self):
        self.sim.step_once()
        self.assertAlmostEqual(self.sim.snapshot().time, 0.01)

    def test_reset(self):
        self.sim.command((1, 0))
        self.sim.step_once()
        self.sim.reset()
        self.assertEqual((self.sim.steps, self.sim.angles, self.sim.targets), (0, (0, 0), (0, 0)))
        self.assertTrue(self.sim.paused)

    def test_repeatable_replay(self):
        def run():
            sim = Simulator(dt=.02)
            sim.command((.9, -.8))
            sim.resume()
            for interval in (.11, .03, .04, .07): sim.advance(interval)
            return sim.snapshot()
        self.assertEqual(run(), run())

    def test_sim_model_6_dof(self):
        sim = Simulator(ArmModel.extended())
        self.assertEqual(len(sim.snapshot().angles), 6)
        sim.command((.2,)*6)
        self.assertEqual(len(sim.step_once().pose.positions), 7)

    def test_invalid_dt(self):
        for dt in (0, -0.1, float('inf'), 0.3):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                Simulator(dt=dt)

    def test_invalid_elapsed(self):
        self.sim.resume()
        for elapsed in (-1, float('nan')):
            with self.subTest(elapsed=elapsed), self.assertRaises(ValueError):
                self.sim.advance(elapsed)


if __name__ == '__main__':
    unittest.main()
