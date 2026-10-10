"""Launch the 2D, advanced 3D, or integrated virtual robotic-arm simulator."""
from __future__ import annotations
import argparse
import json
from .arm import ArmModel


def main() -> None:
    parser=argparse.ArgumentParser(description='Offline CPU robotic-arm simulation')
    parser.add_argument('--model',help='arm JSON config (2–6 planar revolute joints)')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--advanced',action='store_true',help='M5–M10 3D movement/physics lab')
    mode.add_argument('--full',action='store_true',help='M11–M15 integrated sensors/gripper/world/AI lab')
    mode.add_argument('--evaluate',action='store_true',help='headless scripted scenario evaluation')
    parser.add_argument('--scene',help='scene JSON (for --full)')
    parser.add_argument('--trials',type=int,default=5,help='evaluation trials (1..100)')
    args=parser.parse_args()
    model=ArmModel.from_json(args.model) if args.model else ArmModel.basic()
    if args.evaluate:
        if args.scene or args.model:
            parser.error('--evaluate uses the fixed default reference model and scene')
        from .environment import evaluate_scripted
        result=evaluate_scripted(args.trials)
        print(json.dumps(result,indent=2,sort_keys=True))
        if result['successes'] < result['trials']:
            raise SystemExit(1)
        return
    if args.scene and not args.full:
        parser.error('--scene requires --full')
    import tkinter as tk
    root=tk.Tk()
    if args.full:
        from .world import Scene
        from .environment import RobotEnvironment
        from .full_ui import FullWindow
        scene=Scene.load(args.scene) if args.scene else None
        FullWindow(root,RobotEnvironment(model=model,scene=scene))
    elif args.advanced:
        from .advanced import RobotLab
        from .advanced_ui import AdvancedWindow
        AdvancedWindow(root,RobotLab(model))
    else:
        from .ui import ArmWindow
        ArmWindow(root,model)
    root.mainloop()


if __name__=='__main__':
    main()
