"""Real Tk GUI interaction smoke test; requires desktop or DISPLAY (not Pillow).

Run: python tests/gui_smoke.py
In a headless Linux runner: xvfb-run -a python tests/gui_smoke.py
"""
from math import pi
import tkinter as tk
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_sim import ArmModel
from robot_sim.ui import ArmWindow


def check(model, label):
    root = tk.Tk()
    root.geometry('900x720+30+30')
    app = ArmWindow(root, model)
    try:
        root.update()
        assert len(app._sliders) == len(model.joints)
        assert len(app.canvas.find_withtag('arm')) >= len(model.joints) + 5
        app._vars[0].set(90.0)
        root.update()
        assert abs(app.sim.snapshot().angles[0] - pi/2) < 1e-8
        app.toggle()
        assert not app.sim.paused
        app.sim.command(tuple(.1 for _ in model.joints))
        app.step()
        assert app.sim.steps == 1
        app.toggle()
        app.reset()
        root.update()
        assert app.sim.steps == 0 and app.sim.paused
        assert app.sim.angles == model.initial_angles
        print(f'PASS {label}: launch, render, slider, play, step, pause, reset, close')
    finally:
        app.close()


if __name__ == '__main__':
    check(ArmModel.basic(), '2-joint')
    check(ArmModel.extended(), '6-joint')
    print('GUI smoke: 2/2 PASS')
