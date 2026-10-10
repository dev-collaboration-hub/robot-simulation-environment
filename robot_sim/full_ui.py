"""Unified optional M11–M15 scene editor and AI-ready simulator window.

Renders real simulator state. Tk Canvas draws world markers over our CPU 3D
arm renderer. Uses no third-party packages, APIs, engines or GPU shaders.
"""
from __future__ import annotations
import tkinter as tk
from tkinter import ttk
from math import degrees
from time import monotonic
from .math3d import Vec2
from .environment import RobotEnvironment, scripted_pick_and_place
from .render3d import SoftwareRenderer3D, Vec3


class FullWindow:
    def __init__(self, root: tk.Tk, env: RobotEnvironment | None = None):
        self.root=root
        self.env=env or RobotEnvironment()
        self.renderer=SoftwareRenderer3D()
        root.title('Robot Simulation | M11-M15 Integrated Lab')
        root.geometry('1060x760')
        root.minsize(790,580)
        self.running=False
        self._closed=False
        self._job=None
        self._drag=None
        self._last=monotonic()
        container=ttk.Frame(root,padding=12)
        container.pack(fill='both',expand=True)
        ttk.Label(container,text='Integrated robotic arm lab: sensors | gripper | world | agent',
                  font=('TkDefaultFont',16,'bold')).pack(anchor='w')
        self.canvas=tk.Canvas(container,bg='#10192b',highlightthickness=0)
        self.canvas.pack(fill='both',expand=True,pady=(8,8))
        self.canvas.bind('<Configure>',lambda event:self.draw())
        self.canvas.bind('<ButtonPress-1>',lambda event:self._drag_start(event))
        self.canvas.bind('<B1-Motion>',lambda event:self._drag_move(event))
        self.canvas.bind('<MouseWheel>',lambda event:self.zoom(event.delta/120))
        self.canvas.bind('<Button-4>',lambda event:self.zoom(1))
        self.canvas.bind('<Button-5>',lambda event:self.zoom(-1))
        panel=ttk.Frame(container)
        panel.pack(fill='x')
        ttk.Label(panel,text='Target X').grid(row=0,column=0)
        self.x=tk.StringVar(value='1.4')
        ttk.Entry(panel,textvariable=self.x,width=9).grid(row=0,column=1)
        ttk.Label(panel,text='Target Y').grid(row=0,column=2)
        self.y=tk.StringVar(value='.5')
        ttk.Entry(panel,textvariable=self.y,width=9).grid(row=0,column=3)
        ttk.Button(panel,text='Plan XY',command=self.plan).grid(row=0,column=4,padx=5)
        self.run_button=ttk.Button(panel,text='Run',command=self.toggle)
        self.run_button.grid(row=0,column=5,padx=5)
        ttk.Button(panel,text='Step',command=self.step).grid(row=0,column=6,padx=5)
        ttk.Button(panel,text='Reset',command=self.reset).grid(row=0,column=7,padx=5)
        ttk.Button(panel,text='Grip',command=self.grip).grid(row=1,column=0,columnspan=2,sticky='ew',pady=8)
        ttk.Button(panel,text='Release',command=self.release).grid(row=1,column=2,columnspan=2,sticky='ew')
        ttk.Button(panel,text='Scripted demo',command=self.demo).grid(row=1,column=4,columnspan=2,sticky='ew')
        ttk.Button(panel,text='Zoom +',command=lambda:self.zoom(1)).grid(row=1,column=6)
        ttk.Button(panel,text='Zoom −',command=lambda:self.zoom(-1)).grid(row=1,column=7)
        self.note=tk.StringVar(value='Ready. Plan XY, run, grip a nearby block, move, then release.')
        self.readout=tk.StringVar()
        ttk.Label(container,textvariable=self.note,foreground='#2478a5').pack(anchor='w')
        ttk.Label(container,textvariable=self.readout).pack(anchor='w')
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.draw()
        self._job=self.root.after(16,self._tick)

    def _action(self, action):
        try:
            observation,reward,terminated,truncated,info=self.env.step(action)
            self.note.set(info.get('message') or observation['last_event'])
            if terminated or truncated:
                self.running=False
                self.run_button.configure(text='Run')
                self.note.set('Episode complete: '+('SUCCESS' if info['success'] else observation['last_event']))
        except (ValueError,RuntimeError,TypeError,KeyError) as error:
            self.note.set('Cannot run action: '+str(error))
        self.draw()

    def plan(self):
        try:
            self._action({'type':'move_to','x':float(self.x.get()),'y':float(self.y.get())})
        except ValueError as error:
            self.note.set('Invalid XY input: '+str(error))

    def grip(self):
        self._action({'type':'grip'})

    def release(self):
        self._action({'type':'release'})

    def step(self):
        self._action({'type':'noop'})

    def reset(self):
        self.env.reset()
        self.running=False
        self.run_button.configure(text='Run')
        self.note.set('Reset to exact original scene')
        self.draw()

    def demo(self):
        try:
            self.running=False
            self.run_button.configure(text='Run')
            ok,obs=scripted_pick_and_place(self.env)
            self.note.set(('Scripted pick-and-place: PASS' if ok else 'Scripted scenario: FAIL')
                          +f' | steps={self.env.steps}')
        except (ValueError,RuntimeError) as exc:
            self.note.set('Scenario error: '+str(exc))
        self.draw()

    def toggle(self):
        if self.env.done:
            self.note.set('Reset before starting a new episode')
            return
        self.running=not self.running
        self.run_button.configure(text='Pause' if self.running else 'Run')
        self._last=monotonic()

    def _drag_start(self,event):
        self._drag=(event.x,event.y)

    def _drag_move(self,event):
        if self._drag is None:return
        x,y=event.x,event.y
        self.renderer.camera.orbit(x-self._drag[0],y-self._drag[1])
        self._drag=(x,y)
        self.draw()

    def zoom(self,delta):
        self.renderer.camera.change_zoom(delta)
        self.draw()

    def draw(self):
        if self._closed:return
        snapshot=self.env.observe()
        pose=self.env.model.forward(self.env.lab.physics.angles)
        canvas=self.canvas
        self.renderer.draw(canvas,pose,self.env.scene.obstacles)
        canvas.delete('objects')
        w,h=max(canvas.winfo_width(),500),max(canvas.winfo_height(),330)
        for obj in self.env.world.objects.values():
            center=self.renderer.camera.project(Vec3(obj.position.x,obj.position.y,0.),w,h)
            if center is None:continue
            x,y,depth=center
            radius=max(5.,min(26.,obj.radius*min(w,h)*self.renderer.camera.zoom*.9/depth))
            holding=obj.name==self.env.gripper.held
            canvas.create_oval(x-radius,y-radius,x+radius,y+radius,
                               fill='#facc15' if holding else '#fb7185',
                               outline='#fff4b2',width=2,tags='objects')
            canvas.create_text(x,y-radius-8,text=obj.name,fill='#e2e8f0',tags='objects')
        # Goal target is separate from the object and does not move when carried.
        if self.env.goal is not None:
            goal=self.env.goal.destination
            p=self.renderer.camera.project(Vec3(goal.x,goal.y,0),w,h)
            if p:
                x,y,_=p
                canvas.create_line(x-9,y-9,x+9,y+9,fill='#36f5aa',width=2,tags='objects')
                canvas.create_line(x-9,y+9,x+9,y-9,fill='#36f5aa',width=2,tags='objects')
        self.readout.set(
            f'Step {self.env.steps} | tool≈{snapshot["tool_estimate"]} m | '
            f'distance sensor={snapshot["distance_sensor"]:.2f} m | '
            f'encoder={", ".join(f"{degrees(a):.1f}°" for a in snapshot["angles_sensor"])} | '
            f'held={snapshot["held"] or "none"} | '
            f'{"DONE" if snapshot["done"] else "active"}')

    def _tick(self):
        if self._closed:return
        now=monotonic()
        self._last=now
        if self.running and not self.env.done:
            try:
                self._action({'type':'noop'})
            except Exception as exc:
                self.running=False
                self.note.set('Simulation stopped: '+repr(exc))
        self.draw()
        self._job=self.root.after(16,self._tick)

    def close(self):
        self._closed=True
        self.running=False
        if self._job is not None:
            self.root.after_cancel(self._job)
            self._job=None
        self.root.destroy()
