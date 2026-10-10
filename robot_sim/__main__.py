"""Run with: python -m robot_sim [--model examples/two_joint.json]."""
from __future__ import annotations
import argparse
import tkinter as tk
from .arm import ArmModel
from .ui import ArmWindow


def main() -> None:
    parser = argparse.ArgumentParser(description='Scratch-built offline robotic arm simulator')
    parser.add_argument('--model', help='JSON robot model file (2–6 joints)')
    args = parser.parse_args()
    model = ArmModel.from_json(args.model) if args.model else ArmModel.basic()
    root = tk.Tk()
    ArmWindow(root, model)
    root.mainloop()


if __name__ == '__main__':
    main()
