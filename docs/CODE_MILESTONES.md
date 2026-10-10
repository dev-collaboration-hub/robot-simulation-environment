# Code Milestones — Scratch-Built Robotic Arm Simulation

> Repository: `dev-collaboration-hub/robot-simulation-environment`  
> Target branch: `main`  
> Status: **M1–M4 implemented and locally tested; native Windows + user verification pending. No milestone is marked fully complete.**  
> Scope: Build a usable robotic-arm simulator from scratch, progressing from a 2-joint on-screen arm to a 3D, physics-aware, testable environment.

## Ground rules

1. **Do real implementation work in this repository's `main` branch** unless the user explicitly changes that instruction. This file is a plan, not evidence of code completion.
2. **Never tick a milestone just because a plan, README, placeholder, mock, or test file exists.** Completion requires implementation + passing relevant automated tests + an actual end-to-end run + evidence reviewed by both the user and ChatGPT.
3. Keep each milestone **unticked `[ ]`** until that full gate passes. When blocked, write `BLOCKED` with the precise reason; do not claim success.
4. Prefer **Python standard library only** for the initial scratch-built engine, geometry, math, basic physics, tests and Tkinter UI; CPU-only and offline on Windows. Do **not** make PyBullet, MuJoCo, Gazebo, ROS or a GPU a hidden dependency. Reconsider dependencies only with an explicit documented justification/approval.
5. Scratch-built means writing our **own simulation logic** (time stepping, robot model, kinematics, collision approximation, control, sensors, and basic rendering). It does **not** mean rewriting Python, operating-system graphics drivers or the standard library.
6. Use deterministic tests (fixed seeds and fixed simulation timestep), shared coordinate conventions, explicit joint limits, and safe finite-value validation. Simulation results must not be presented as certified real-hardware behavior.
7. Before adding another repository, demonstrate why a reusable component cannot be cleanly implemented in this repo or consumed from an existing repository.

## Planned code layout (create when implementing, not as empty placeholders)

```text
robot_sim/
  __main__.py             # launch: python -m robot_sim
  app.py                  # application entry and controls
  math3d.py               # vectors, matrices, transforms
  arm.py                  # joints, links, end-effector, limits
  engine.py               # deterministic simulation step
  kinematics.py           # FK and IK
  render2d.py             # initial Tkinter canvas renderer
  render3d.py             # CPU 3D projection and drawing
  trajectory.py           # speed-limited paths
  physics.py              # gravity, forces, basic dynamics
  collision.py            # geometry overlap and contacts
  actuators.py            # simulated motors and PID
  sensors.py              # virtual measurements
  gripper.py              # grasp and release
  world.py                # objects, obstacles, scenes
  scenarios.py            # repeatable tasks
  integration.py          # adapters for external robot modules
tests/                    # unittest: no paid CI required
examples/                 # runnable scenarios
evidence/                 # actual verification per milestone
```

The layout can evolve only when a code-level need is demonstrated. Do not build all modules upfront.

## Milestones and acceptance gates

### M1 — First usable 2-joint arm (2D)
- [ ] Implement a basic two-link arm with configurable lengths and a grounded base.
- [ ] Render it on screen using Tkinter Canvas; provide two joint-angle controls, Reset, and a visible end-effector.
- [ ] Accept user input safely and update the arm immediately without flicker or crashes.
- [ ] Test zero/90-degree poses, link lengths, angle boundaries, and reset.
- **End-to-end gate:** Launch `python -m robot_sim`, move both joints in a real window, reset, and capture a screenshot plus test log.

### M2 — Scratch-built coordinate math and forward kinematics
- [ ] Implement vector, matrix, rotation, and transform utilities with a documented unit/axis convention.
- [ ] Compute every joint position and end-effector pose from angles and link lengths.
- [ ] Use these calculations in the running M1 demo, not in a disconnected example.
- [ ] Test reference poses, composition of transforms, precision tolerance, and invalid inputs.
- **Gate:** On-screen coordinates and independently calculated test cases agree.

### M3 — Deterministic simulation loop
- [ ] Add pause, step-once, resume, reset, and simulation clock with fixed timestep.
- [ ] Separate model updates from rendering and ensure consistent behavior under varying screen refresh rates.
- [ ] Log state snapshots; re-running identical inputs reproduces the same output.
- **Gate:** Repeat one command sequence twice, compare state snapshots, and test pause/step/reset.

### M4 — Reusable robotic arm model
- [ ] Support configurable revolute joints, links, joint limits, and 2–6 degrees of freedom.
- [ ] Validate arm configuration and provide at least two model examples.
- [ ] Keep hardware-independent simulated joint states clearly separate from real motor drivers.
- **Gate:** Two distinct arm configurations load and move without changing engine code.

### M5 — CPU-only 3D visualization
- [ ] Implement 3D coordinates, camera, perspective/orthographic projection and world axes.
- [ ] Draw robot links/joints with a basic software renderer; support rotate, zoom, and pan.
- [ ] Handle depth ordering/visibility sufficiently for a simple arm scene; document known rendering limitations.
- **Gate:** A 3D arm can be viewed from multiple camera angles and tracked during motion without a GPU or rendering framework.

### M6 — Inverse kinematics
- [ ] Provide a reachable-target solver for the two-link arm and a numerical method for supported multi-joint configurations.
- [ ] Respect joint limits; report unreachable targets rather than silently returning a wrong pose.
- [ ] Add a GUI/API target-position command.
- **Gate:** Known reachable targets converge within a stated tolerance; unreachable and singularity-near cases are tested.

### M7 — Trajectory planning
- [ ] Generate time-parameterized joint paths with speed/acceleration limits.
- [ ] Support start, stop, cancel and replan; keep motion inside joint limits.
- [ ] Show target trajectory versus actual simulated joint positions.
- **Gate:** Recorded paths meet stated bounds without discontinuous jumps.

### M8 — Basic physics and gravity
- [ ] Add link mass/inertia approximations, gravity, torque/force inputs, and numerical integration.
- [ ] Define and test the limits of the physics model; start with a simple stable approximation rather than claim industrial-grade rigid-body simulation.
- [ ] Compare gravity-on and gravity-off behavior under known configurations.
- **Gate:** Repeatable physical-state tests pass and no unexplained divergence appears in bounded test scenes.

### M9 — Collision detection and constraints
- [ ] Detect arm versus obstacle, base and supported self-collisions using documented simple collision shapes.
- [ ] Handle overlaps/contacts with basic constraints or a safe stop and meaningful diagnostics.
- [ ] Test contact, no-contact, edge contact, and near-boundary cases.
- **Gate:** Real on-screen collision scenarios match expected test results; collision cannot silently pass through an obstacle in supported conditions.

### M10 — Virtual actuators and feedback controller
- [ ] Build simulated joint motors with position/velocity commands and bounded outputs.
- [ ] Implement PID or comparable feedback control and basic saturation/fault feedback.
- [ ] Compare target and measured joint states in logs/UI.
- **Gate:** Controlled movement settles within defined tolerance without exceeding stated motor/joint limits.

### M11 — Virtual sensor system
- [ ] Simulate joint encoders and at least one environment-oriented sensor (e.g., distance).
- [ ] Add optional deterministic noise, update rates, and sensor reset.
- [ ] Route sensor outputs through a clean interface rather than giving higher layers hidden access to true state.
- **Gate:** Sensor readings match expected ground truth within defined error bounds and repeat with the same seed.

### M12 — Gripper and pick-and-place
- [ ] Implement open/close gripper state, supported grasp detection, attachment and release.
- [ ] Allow a simple block to be picked up, moved and placed without teleporting or cloning the object.
- [ ] Detect failed grasps and blocked placement.
- **Gate:** A complete pickup-and-placement replay succeeds; failures are visible and testable.

### M13 — Scene and environment management
- [ ] Add ground plane, blocks, obstacles, and load/save of scenes with validated JSON formats.
- [ ] Add a small scene library: free movement, target reaching, obstructed target, pick-and-place.
- [ ] Support restoring exact initial conditions for regression testing.
- **Gate:** Each saved scene reloads and produces the same initial robot/object state.

### M14 — Robot control integration
- [ ] Define a documented simulation API for commands, joint states, sensor reads, errors and events.
- [ ] Integrate with `dev-collaboration-hub/robot-motion-control` and, where appropriate, `robotic-arm-control-system` through an adapter; avoid copying whole repositories or importing hardware-only dependencies.
- [ ] Keep standalone execution functional when those external repositories are unavailable.
- **Gate:** A tested command-to-motion-to-feedback sequence works both standalone and through at least one actual external adapter.

### M15 — Repeatable scenarios and AI-ready interface
- [ ] Expose environment reset/step/observe/action/reward/done semantics for later planning and learning.
- [ ] Create deterministic scripted evaluation scenarios; keep a safe manual override and enforce joint/collision limits.
- [ ] Benchmark task success rate on fixed scenarios without claiming that training or autonomy is complete.
- **Gate:** A scripted agent completes a documented scenario; replay and results are independently reproducible.

### M16 — Full integration, performance, and final release gate
- [ ] Run unit, integration, GUI smoke, collision, controller, and scenario regression tests.
- [ ] Measure CPU, memory, and frame rate on the actual target PC; record hardware, Python version, scene, and run duration. Record actual values rather than invent a performance claim.
- [ ] Verify offline launch, safe reset, clean exit, no paid APIs, and documentation reflecting implemented behavior.
- [ ] Make a consolidated acceptance report with known limits and any unverified features.
- **Gate:** Both the user and ChatGPT review actual test + real runtime evidence. Mark overall project complete only if all required gates pass.

## Verification evidence (required for **every** milestone)

Store real outputs under `evidence/Mxx/<run-id>/` (do not fabricate or write these files in advance):

- `commit.txt`: exact tested commit SHA, branch, and any uncommitted changes.
- `commands.txt`: exact commands used and environment information.
- `tests.txt`: named tests, pass/fail/skip counts, failures, and logs.
- `runtime.txt` or `runtime.json`: actual observable behavior, steps, expected vs observed.
- `screenshots/` or a short recording for visual milestones; do not substitute a generated mockup.
- `review.md`: user check, ChatGPT check, outstanding defects, and final gate decision.

Expected basic commands **after M1 code exists**:

```powershell
python -m unittest discover -s tests -v
python -m robot_sim
```

For nonvisual milestones, use real scenario traces/logs instead of unnecessary screenshots.

## Progress

| Milestone | State | Evidence |
| --- | --- | --- |
| M1 2-joint 2D demo | CODE + LINUX GUI PASS / USER CHECK PENDING | [Verification](../evidence/M01-M04/verification.md) |
| M2 Coordinate math + FK | CODE + UNIT TEST PASS / USER CHECK PENDING | [Verification](../evidence/M01-M04/verification.md) |
| M3 Deterministic loop | CODE + UNIT TEST PASS / USER CHECK PENDING | [Verification](../evidence/M01-M04/verification.md) |
| M4 Arm model | CODE + LINUX GUI PASS / USER CHECK PENDING | [Verification](../evidence/M01-M04/verification.md) |
| M5 3D renderer | NOT STARTED / UNVERIFIED | — |
| M6 IK | NOT STARTED / UNVERIFIED | — |
| M7 Trajectories | NOT STARTED / UNVERIFIED | — |
| M8 Physics | NOT STARTED / UNVERIFIED | — |
| M9 Collision | NOT STARTED / UNVERIFIED | — |
| M10 Actuators | NOT STARTED / UNVERIFIED | — |
| M11 Sensors | NOT STARTED / UNVERIFIED | — |
| M12 Gripper | NOT STARTED / UNVERIFIED | — |
| M13 Scenes | NOT STARTED / UNVERIFIED | — |
| M14 External integration | NOT STARTED / UNVERIFIED | — |
| M15 Scenarios + AI-ready API | NOT STARTED / UNVERIFIED | — |
| M16 Final acceptance | NOT STARTED / UNVERIFIED | — |

**Next acceptance task: Native Windows and user verification for M1–M4. Next planned code milestone: M5.** Documentation alone never changes a milestone status.
