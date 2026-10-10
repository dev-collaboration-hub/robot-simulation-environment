"""Interactive 3D Tk GUI for M5-M10 (CPU-only, no third-party packages)."""
from __future__ import annotations
import tkinter as tk
from tkinter import ttk
from time import monotonic
from math import degrees
from .advanced import RobotLab
from .collision import CollisionWorld, CircleObstacle, BoxObstacle
from .math3d import Vec2
from .render3d import SoftwareRenderer3D, Vec3


class AdvancedWindow:
    def __init__(self, root: tk.Tk, lab: RobotLab | None = None):
        self.root = root
        self.root.title('Robotic Arm | 3D Lab M5-M10')
        self.root.geometry('1050x760')
        self.root.minsize(800, 590)
        self.lab = lab or RobotLab()
        self.renderer = SoftwareRenderer3D()
        self._closed = False
        self._job = None
        self._last = monotonic()
        self._drag = None
        self._drag_button = None
        self._obstacles = (CircleObstacle(.55,.65,.13), BoxObstacle(-.8,.1,-.55,.55))
        body = ttk.Frame(root, padding=12)
        body.pack(fill='both',expand=True)
        ttk.Label(body,text='Scratch-built robotic arm | 3D simulation lab',
                  font=('TkDefaultFont', 16, 'bold')).pack(anchor='w')
        ttk.Label(body,text='Planar joints, 3D solid-link rendering | approximate gravity, PID and conservative collision stop',
                  foreground='#64748b').pack(anchor='w',pady=(0,8))
        self.canvas = tk.Canvas(body,bg='#10192b',highlightthickness=0)
        self.canvas.pack(fill='both',expand=True)
        self.canvas.bind('<ButtonPress-1>',lambda e:self._start_drag(e,'orbit'))
        self.canvas.bind('<B1-Motion>',self._drag_move)
        self.canvas.bind('<ButtonPress-3>',lambda e:self._start_drag(e,'pan'))
        self.canvas.bind('<B3-Motion>',self._drag_move)
        self.canvas.bind('<MouseWheel>',lambda e:self.zoom(e.delta/120))
        self.canvas.bind('<Button-4>',lambda e:self.zoom(1))
        self.canvas.bind('<Button-5>',lambda e:self.zoom(-1))
        self.canvas.bind('<Configure>',lambda e:self.draw())
        tools = ttk.Frame(body,padding=(0,10,0,0))
        tools.pack(fill='x')
        ttk.Label(tools,text='Target X (m)').grid(row=0,column=0,sticky='w')
        ttk.Label(tools,text='Target Y (m)').grid(row=0,column=2,sticky='w')
        self.target_x = tk.StringVar(value='1.25')
        self.target_y = tk.StringVar(value='0.85')
        ttk.Entry(tools,width=9,textvariable=self.target_x).grid(row=0,column=1,padx=5)
        ttk.Entry(tools,width=9,textvariable=self.target_y).grid(row=0,column=3,padx=5)
        ttk.Button(tools,text='IK + Plan',command=self.plan).grid(row=0,column=4,padx=6)
        self.run_button=ttk.Button(tools,text='Resume',command=self.toggle)
        self.run_button.grid(row=0,column=5,padx=5)
        ttk.Button(tools,text='Step',command=self.step).grid(row=0,column=6,padx=5)
        ttk.Button(tools,text='Reset',command=self.reset).grid(row=0,column=7,padx=5)
        self.gravity_var=tk.BooleanVar(value=False)
        ttk.Checkbutton(tools,text='Gravity',variable=self.gravity_var,command=self.set_gravity).grid(row=1,column=0,columnspan=2,sticky='w',pady=8)
        self.obstacle_var=tk.BooleanVar(value=False)
        ttk.Checkbutton(tools,text='Obstacles',variable=self.obstacle_var,command=self.set_obstacles).grid(row=1,column=2,columnspan=2,sticky='w')
        ttk.Button(tools,text='Orbit left',command=lambda:self.orbit(-15,0)).grid(row=1,column=4)
        ttk.Button(tools,text='Orbit right',command=lambda:self.orbit(15,0)).grid(row=1,column=5)
        ttk.Button(tools,text='Zoom +',command=lambda:self.zoom(1)).grid(row=1,column=6)
        ttk.Button(tools,text='Zoom −',command=lambda:self.zoom(-1)).grid(row=1,column=7)
        self.message=tk.StringVar(value='Ready — enter reachable XY target and press IK + Plan')
        self.status=tk.StringVar(value='')
        ttk.Label(body,textvariable=self.message,foreground='#1a73aa').pack(anchor='w')
        ttk.Label(body,textvariable=self.status).pack(anchor='w')
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.draw()
        self._job = root.after(16,self._tick)

    def plan(self):
        try:
            target=Vec2(float(self.target_x.get()), float(self.target_y.get()))
            result=self.lab.plan_target(target)
            self.message.set(('Path planned' if result.success else 'NOT planned')+
                             f' | IK error {result.error:.4f} m | {result.reason}')
        except (ValueError,TypeError) as exc:
            self.message.set(f'Invalid target: {exc}')
        self.draw()

    def toggle(self):
        if self.lab.paused:
            self.lab.resume()
            self.run_button.configure(text='Pause')
        else:
            self.lab.pause()
            self.run_button.configure(text='Resume')
        self._last=monotonic()
        self.draw()

    def step(self):
        self.lab.step_once()
        self._show_fault()
        self.draw()

    def reset(self):
        self.lab.reset()
        self.run_button.configure(text='Resume')
        self.message.set('Reset complete')
        self._last=monotonic()
        self.draw()

    def set_gravity(self):
        self.lab.set_gravity(self.gravity_var.get())
        self.draw()

    def set_obstacles(self):
        self.lab.world = CollisionWorld(self._obstacles if self.obstacle_var.get() else ())
        self.lab.player.cancel()  # queued path may no longer be safe
        self.draw()

    def orbit(self,dx,dy):
        self.renderer.camera.orbit(dx,dy)
        self.draw()

    def zoom(self,steps):
        self.renderer.camera.change_zoom(steps)
        self.draw()

    def _start_drag(self,event,mode):
        self._drag=(event.x,event.y)
        self._drag_button=mode

    def _drag_move(self,event):
        if self._drag is None:return
        dx,dy=event.x-self._drag[0],event.y-self._drag[1]
        self._drag=(event.x,event.y)
        if self._drag_button=='orbit':
            self.renderer.camera.orbit(dx,dy)
        else:
            self.renderer.camera.pan_x+=dx
            self.renderer.camera.pan_y+=dy
        self.draw()

    def _show_fault(self):
        if self.lab.collision:
            self.message.set('Collision safety stop: '+self.lab.collision)
            self.run_button.configure(text='Resume')

    def draw(self):
        if self._closed:return
        snap=self.lab.snapshot()
        pose=self.lab.model.forward(snap.angles)
        self.renderer.draw(self.canvas,pose,self.lab.world.obstacles)
        self.status.set(f'{"Paused" if snap.paused else "Running"} | t={snap.time:.2f}s | '
                        f'joints: {", ".join(f"{degrees(q):+.1f}°" for q in snap.angles)} | '
                        f'reference error={snap.error:.3f} rad | '
                        f'tool=({pose.tool.x:.2f}, {pose.tool.y:.2f}) m')

    def _tick(self):
        if self._closed:return
        now=monotonic()
        self.lab.advance(max(0,now-self._last))
        self._last=now
        self._show_fault()
        self.draw()
        self._job=self.root.after(16,self._tick)

    def close(self):
        self._closed=True
        if self._job is not None:
            self.root.after_cancel(self._job)
            self._job=None
        self.root.destroy()
