"""M16 machine-measured headless stress/CPU/memory benchmark.

Usage: python scripts/benchmark.py [--steps 2000]
Reports measurements from *the machine that actually runs it*, not estimates.
"""
from __future__ import annotations
import argparse
import json
import os
import platform
import sys
from pathlib import Path
from time import perf_counter, process_time
import tracemalloc

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_sim.advanced import RobotLab
from robot_sim.math3d import Vec2


def benchmark(steps=2000):
    if isinstance(steps,bool) or not isinstance(steps,int) or steps < 100 or steps > 100000:
        raise ValueError('steps must be an integer from 100 to 100000')
    lab=RobotLab()
    result=lab.plan_target(Vec2(1.2,.9))
    if not result.success:
        raise RuntimeError('reference target path did not plan')
    tracemalloc.start()
    wall_start=perf_counter()
    cpu_start=process_time()
    for _ in range(steps):
        lab.step_once()
        state=lab.snapshot()
        if state.collision:
            raise RuntimeError('collision in empty benchmark scene')
    wall=max(perf_counter()-wall_start,1e-10)
    cpu=max(process_time()-cpu_start,0)
    current,peak=tracemalloc.get_traced_memory()
    tracemalloc.stop()
    end=lab.snapshot()
    return {
        'system':platform.platform(),
        'python':sys.version.split()[0],
        'logical_cpus':os.cpu_count(),
        'steps':steps,
        'simulation_seconds':round(steps*lab.dt,6),
        'wall_seconds':round(wall,6),
        'process_cpu_seconds':round(cpu,6),
        'steps_per_wall_second':round(steps/wall,2),
        'realtime_factor':round(steps*lab.dt/wall,3),
        'tracemalloc_peak_bytes':peak,
        'tracemalloc_final_bytes':current,
        'final_joint_angles':end.angles,
        'final_tool_position':(lab.model.forward(end.angles).tool.x,
                               lab.model.forward(end.angles).tool.y),
        'limitations':'headless CPU simulation loop; not GUI FPS, not physics accuracy or hardware safety'
    }


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--steps',type=int,default=2000)
    args=parser.parse_args()
    print(json.dumps(benchmark(args.steps),indent=2,sort_keys=True))
