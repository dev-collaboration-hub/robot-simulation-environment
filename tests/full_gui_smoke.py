"""M16 live Tk scene/sensor/gripper/agent GUI smoke (requires a real DISPLAY)."""
from __future__ import annotations
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import tkinter as tk
from robot_sim.environment import RobotEnvironment
from robot_sim.full_ui import FullWindow
from robot_sim.world import example_scene


def smoke(scene):
    root=tk.Tk()
    app=FullWindow(root,RobotEnvironment(scene=scene))
    try:
        root.update()
        assert app.canvas.find_withtag('scene3d'), '3D renderer did not draw'
        assert app.canvas.find_withtag('objects'), 'world object and target not drawn'
        before=app.env.observe()['sensor_sequence']
        app.x.set('1.4')
        app.y.set('.5')
        app.plan()
        assert app.env.lab.player.path is not None,'IK planning failed'
        for _ in range(8):
            app.step()
        after=app.env.observe()['sensor_sequence']
        assert after>before, 'sensor feedback not updating'
        app.zoom(2)
        app.reset()
        assert app.env.steps==0 and app.env.gripper.held is None
        root.update()
        print('PASS full GUI: live renderer, world, planning, sensor ticks, zoom, reset, close')
    finally:
        app.close()


if __name__=='__main__':
    smoke(example_scene())
