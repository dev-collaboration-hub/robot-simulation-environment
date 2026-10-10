# Robot Simulation Environment

## Overview

Robot Simulation Environment is a robotics project focused on creating virtual environments where robotic systems can be designed, tested, and improved before deploying on real hardware.

The goal of this repository is to safely test robot movement, sensors, control algorithms, and AI models through simulation.

---

# Objectives

- Build virtual robot testing environments
- Reduce hardware testing risks
- Test algorithms before real deployment
- Create simulation tools for robotics research

---

# Core Modules


## 1. Robot Model System

Virtual representation of robots.

Features:

- Robot structure modeling
- Joint configuration
- Physical properties
- Movement constraints


---

## 2. Physics Simulation

Simulates real-world behavior.

Features:

- Gravity
- Collision detection
- Forces
- Motion dynamics


---

## 3. Virtual Sensors

Simulated sensor environment.

Features:

- Camera simulation
- Distance sensors
- IMU simulation
- Encoder feedback


---

## 4. Control Testing

Test robot controllers safely.

Features:

- Motor command testing
- Motion validation
- PID tuning
- Error analysis


---

## 5. AI Training Environment

Train intelligent robots.

Features:

- Reinforcement Learning environments
- Reward systems
- Autonomous task learning


---

# Architecture


Robot Algorithm

        |

Simulation Engine

        |

Virtual Robot

        |

Physics System

        |

Performance Feedback


---

# Future Applications

- Robotic Arm Simulation
- Autonomous Robots
- AI Robot Training
- Digital Twin Systems
- Reinforcement Learning


---

# Roadmap


## Phase 1: Basic Simulation

- [ ] Robot Model Loader
- [ ] Joint Simulation
- [ ] Movement Testing


## Phase 2: Physics Layer

- [ ] Collision System
- [ ] Sensor Simulation
- [ ] Environment Interaction


## Phase 3: AI Simulation

- [ ] RL Training Environment
- [ ] Autonomous Testing
- [ ] Digital Twin System


---

# Vision

To create a safe virtual robotics laboratory where intelligent machines can learn before entering the real world.


## Runnable robotic-arm simulator (M1–M4)

A dependency-free, offline, CPU-only **planar 2D robotic arm** foundation has been implemented on `main`. It is **not yet the later 3D or physics simulator**. Python 3.10+ with Tkinter is required.

**Windows:** clone/download this repository and double-click `launch_robot_simulator.bat`.

**From a terminal, from the repository folder:**

```sh
python -m robot_sim
python -m robot_sim --model examples/six_joint.json
python -m unittest discover -s tests -v
python tests/gui_smoke.py
```

The GUI supports joint sliders, live end-effector coordinates, pause/resume, step and reset. It accepts 2–6 planar revolute joints configured in JSON. The mathematics, simulation loop and robot model use only the Python standard library; no pip install, GPU or paid API is required.

**Verification (2026-10-10):** 43 unit tests passed, and both 2- and 6-joint GUI smoke tests passed using a real Tk window on a Linux virtual display. These are local tests; native Windows confirmation and user acceptance are still pending. See [M1–M4 verification](evidence/M01-M04/verification.md). Do not mark the milestone checkboxes complete until the agreed verification gates pass.

## Scratch-built robotic arm: code milestones

The implementation plan, code tasks, real-run acceptance criteria, and required verification evidence are in [Code Milestones](docs/CODE_MILESTONES.md). All milestones remain unverified until implemented, tested, run, and reviewed.
