"""Built-in Tkinter renderer and controls for 2-6 DOF planar robot arms."""
from __future__ import annotations
from math import degrees, radians
from time import monotonic
import tkinter as tk
from tkinter import ttk
from .arm import ArmModel
from .engine import Simulator


class ArmWindow:
    def __init__(self, root: tk.Tk, model: ArmModel | None = None):
        self.root = root
        self.sim = Simulator(model)
        self.root.title('Robot Simulation Environment | M1-M4')
        self.root.minsize(680, 530)
        self.root.configure(bg='#0b1220')
        self._last_time = monotonic()
        self._sliders: list[ttk.Scale] = []
        self._vars: list[tk.DoubleVar] = []
        self._updating_controls = False
        self._closed = False
        self._after_id: str | None = None

        frame = ttk.Frame(root, padding=12)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text=f'Robotic Arm | {len(self.sim.model.joints)} joints',
                  font=('TkDefaultFont', 16, 'bold')).pack(anchor='w')
        self.canvas = tk.Canvas(frame, bg='#10192b', highlightthickness=0, height=360)
        self.canvas.pack(fill='both', expand=True, pady=(10, 8))
        self.controls = ttk.Frame(frame)
        self.controls.pack(fill='x')
        for i, joint in enumerate(self.sim.model.joints):
            row = ttk.Frame(self.controls)
            row.pack(fill='x')
            ttk.Label(row, text=joint.name, width=13).pack(side='left')
            angle = tk.DoubleVar(value=degrees(joint.initial))
            slider = ttk.Scale(row, from_=degrees(joint.minimum), to=degrees(joint.maximum),
                               variable=angle)
            slider.pack(side='left', fill='x', expand=True, padx=8)
            label = ttk.Label(row, width=8)
            label.pack(side='right')
            angle.trace_add('write', lambda *_args, v=angle, l=label, idx=i: (l.configure(text=f'{v.get():+.0f}°'), self._angle_changed(idx)))
            label.configure(text=f'{angle.get():+.0f}°')
            self._vars.append(angle)
            self._sliders.append(slider)
        toolbar = ttk.Frame(frame)
        toolbar.pack(fill='x', pady=8)
        self.toggle_button = ttk.Button(toolbar, text='Resume', command=self.toggle)
        self.toggle_button.pack(side='left', padx=(0, 6))
        ttk.Button(toolbar, text='Step', command=self.step).pack(side='left', padx=6)
        ttk.Button(toolbar, text='Reset', command=self.reset).pack(side='left', padx=6)
        self.status = ttk.Label(toolbar, text='Paused | step 0 | t=0.00s')
        self.status.pack(side='right')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.render()
        self._after_id = self.root.after(16, self._update)

    def _angle_changed(self, index: int) -> None:
        if self._updating_controls:
            return
        targets = list(self.sim.targets)
        targets[index] = radians(self._vars[index].get())
        joint = self.sim.model.joints[index]
        targets[index] = max(joint.minimum, min(joint.maximum, targets[index]))
        self.sim.command(targets)
        # Paused mode acts like a model-editor: sliders immediately change pose.
        if self.sim.paused:
            self.sim.angles = self.sim.targets
        self.render()

    def toggle(self) -> None:
        if self.sim.paused:
            self.sim.resume()
            self.toggle_button.configure(text='Pause')
        else:
            self.sim.pause()
            self.toggle_button.configure(text='Resume')
        self._last_time = monotonic()
        self.render()

    def step(self) -> None:
        self.sim.step_once()
        self.render()

    def reset(self) -> None:
        self.sim.reset()
        self._updating_controls = True
        try:
            for angle, joint in zip(self._vars, self.sim.model.joints):
                angle.set(degrees(joint.initial))
        finally:
            self._updating_controls = False
        self.toggle_button.configure(text='Resume')
        self._last_time = monotonic()
        self.render()

    def render(self) -> None:
        c = self.canvas
        c.delete('arm')
        snap = self.sim.snapshot()
        width = max(c.winfo_width(), 680)
        height = max(c.winfo_height(), 360)
        reach = self.sim.model.reach
        scale = min(width*.42, height*.38) / max(reach, 0.01)
        origin_x, origin_y = width/2, height/2
        def project(point):
            return origin_x + point.x*scale, origin_y - point.y*scale
        x0, y0 = project(self.sim.model.base)
        c.create_line(0, y0, width, y0, fill='#26354a', dash=(3, 5), tags='arm')
        c.create_line(x0, 0, x0, height, fill='#26354a', dash=(3, 5), tags='arm')
        points = [project(v) for v in snap.pose.positions]
        for i in range(len(points)-1):
            x1, y1 = points[i]
            x2, y2 = points[i+1]
            c.create_line(x1, y1, x2, y2, width=12, capstyle='round',
                          fill='#37b7e9' if i % 2 == 0 else '#ffc96a', tags='arm')
        for x, y in points[:-1]:
            c.create_oval(x-8, y-8, x+8, y+8, fill='#e7f2ff', outline='#163657', width=2, tags='arm')
        x, y = points[-1]
        c.create_oval(x-9, y-9, x+9, y+9, fill='#62dbab', outline='', tags='arm')
        c.create_text(12, 12, anchor='nw', fill='#dbeafe',
                      text=f'Tool: ({snap.pose.tool.x:+.3f}, {snap.pose.tool.y:+.3f}) m', tags='arm')
        c.create_text(width-12, 12, anchor='ne', fill='#8ca2bb',
                      text='X right · Y up · CCW positive', tags='arm')
        self.status.configure(text=f'{"Paused" if snap.paused else "Running"} | '
                                   f'step {snap.step} | t={snap.time:.2f}s')

    def _update(self) -> None:
        if self._closed:
            return
        now = monotonic()
        self.sim.advance(max(0.0, now-self._last_time))
        self._last_time = now
        self.render()
        self._after_id = self.root.after(16, self._update)

    def close(self) -> None:
        self._closed = True
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        self.root.destroy()
