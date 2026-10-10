"""Real Tk 3D lab smoke; run under xvfb-run on headless Linux."""
from __future__ import annotations
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import tkinter as tk
from robot_sim.advanced_ui import AdvancedWindow
from robot_sim.advanced import RobotLab
from robot_sim.arm import ArmModel


def smoke(model,label):
    root=tk.Tk()
    app=AdvancedWindow(root,RobotLab(model))
    try:
        root.update()
        assert len(app.canvas.find_withtag('scene3d')) > 25
        initial=app.renderer.camera.project(__import__('robot_sim.render3d',fromlist=['Vec3']).Vec3(1,0,0),800,600)
        app.orbit(30,10)
        app.zoom(2)
        assert app.renderer.camera.project(__import__('robot_sim.render3d',fromlist=['Vec3']).Vec3(1,0,0),800,600) != initial
        app.target_x.set('1.2' if len(model.joints)==2 else '2.5')
        app.target_y.set('0.9')
        app.plan()
        root.update()
        assert 'Path planned' in app.message.get(),app.message.get()
        app.toggle()
        assert not app.lab.paused
        app.step()
        assert app.lab.step_index == 1
        app.toggle()
        app.gravity_var.set(True)
        app.set_gravity()
        assert app.lab.physics.gravity==9.81
        app.obstacle_var.set(True)
        app.set_obstacles()
        assert len(app.lab.world.obstacles)==2
        app.reset()
        assert app.lab.step_index==0 and app.lab.paused
        root.update()
        print('PASS',label,'3D GUI controls, planning, PID tick, gravity, obstacles, reset')
    finally:
        app.close()

if __name__=='__main__':
    smoke(ArmModel.basic(),'2-joint')
    smoke(ArmModel.extended(),'6-joint')
