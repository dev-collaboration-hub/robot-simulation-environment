# M5–M10: Scratch-built advanced simulation verification

**Date:** 2026-10-10  
**Branch:** main  
**Decision:** **Code implemented and tested on a local Linux virtual desktop. NOT yet accepted as COMPLETE on the user's Windows PC.** No milestones are ticked.

## Features actually implemented

| Milestone | Code / observed behavior | Limitations |
| --- | --- | --- |
| M5 3D renderer | Vec3, camera yaw/pitch, zoom/pan, perspective/orthographic projection, shaded solid link cuboids, XY joints projected into a 3D workspace and ground grid | **Joint kinematics still planar (XY)**; this is not arbitrary six-axis spatial IK; painter's algorithm is approximate, not depth-buffer GPU rendering |
| M6 IK | Analytic 2R target solving with elbow alternatives; numerical damped least-squares for 2–6 planar joints; explicit unreachable and joint-limit reporting | Numerical IK does not guarantee every feasible pose will be found |
| M7 trajectory | Deterministic C1 smoothstep paths; analytically bounded peak speed and acceleration; start/stop/cancel/replan and target-vs-actual tracking | Not jerk-limited; acceleration changes at segment endpoints |
| M8 basic physics | Torque, approximate gravity, mass, inertia, damping, bounded semi-implicit integration and joint limit stops | Independent joint dynamics; **no full rigid-body coupling/contact forces** |
| M9 collision | Planar link capsules vs circle/axis-aligned box obstacles; nonadjacent self-collision; swept interpolation; conservative stop with diagnostic | No rigid-body contact response or full spatial mesh collision |
| M10 virtual motors | Independent bounded PID motor torque, anti-windup, speed limits, position feedback, reset | Qualitative virtual motor model, not certified hardware controller |

The integrated **RobotLab** connects: target XY -> IK -> path preflight -> sampled trajectory -> PID torques -> approximate dynamics -> collision sweep -> GUI state. M5-M10 are available through the real Tk desktop GUI.

## Reproducible commands

From the repository root, Python 3.10+ with Tkinter installed; no pip, GPU or paid API needed:

\`\`\`sh
python -m unittest discover -s tests -v
python -m robot_sim --advanced
python -m robot_sim --advanced --model examples/six_joint.json
python tests/advanced_gui_smoke.py
\`\`\`

On **Windows**, double-click \`launch_advanced_simulator.bat\`. For headless Linux with Xvfb installed:

\`\`\`sh
xvfb-run -a python tests/advanced_gui_smoke.py
\`\`\`

## Actual results observed in this session

- Environment: Linux, Python 3.13.5, Tk 8.6 with Xvfb (a real Tk desktop window in a virtual display).
- Total offline tests: **100 / 100 passed** (43 original M1–M4 + 57 advanced M5–M10), \`Ran 100 tests in 0.165s; OK\`.
- Real GUI smoke: **2 / 2 PASS**, with 2-joint and 6-joint windows; each verified drawing, orbit, zoom, XY target planning, PID tick, pause, gravity toggle, obstacle toggle, reset, close.
- Actual \`python -m robot_sim --advanced\` graphical entrypoint: process still running after 1s, no stderr; terminated intentionally for automated smoke validation.
- Sample physical controller: target XY (1.2, 0.9) m; after 500 fixed steps at 0.01s, resulting tool position approximately (1.1760, 0.8997) m (position error ~0.0240 m).
- A 1280x900 **real GUI screenshot** from Xvfb was captured during local validation. File stored in the assistant's execution workspace as \`evidence/M05-M10/advanced_3d_gui.png\`, **not present in this GitHub repository**. Do not confuse this with Windows validation.
- No paid GitHub CI workflow was requested or started.

## Hash validation of committed code

Each GitHub \`main\` blob was fetched after upload and verified to match the exact tested local file:

| Path | Git blob SHA |
| --- | --- |
| robot_sim/render3d.py | d945b580d83e95f115ec116856c412bb3789c0f1 |
| robot_sim/kinematics.py | 87aa26d42d61364eecfb23ec0a2818f4af998215 |
| robot_sim/trajectory.py | e833ebff526a0168cbb066537741bebb66175c72 |
| robot_sim/physics.py | be326260120bbb564c58d123aeda4c2784fc9928 |
| robot_sim/collision.py | 2e49dd5f1e7a20a386bfbae1e8c5c0f18bf756af |
| robot_sim/actuators.py | cc30db72d53ba2f5de0d78b2147dc323e1421f07 |
| robot_sim/advanced.py | 5ba6ef076330c0e722c53ff402fffca5c2018fc4 |
| robot_sim/advanced_ui.py | ff532ea89d06dc434518327bef054605d129458c |
| robot_sim/__main__.py | 4cc5f6d557a09cfdf9f7fe7cb4f253710f3de8d8 |
| tests/test_advanced.py | 4a1ff39b85bddfd6fd65039bc9b863eade854205 |
| tests/advanced_gui_smoke.py | 41c50427a944fcb7f28ad7ca3773b5442e9302f3 |

## Still required before any completion tick

1. On the user's real Windows desktop, launch \`launch_advanced_simulator.bat\` and verify live visuals, dragging/orbit, zoom and clean exit.
2. Set target X=1.2 and Y=0.9, click IK + Plan, Resume, and confirm the arm visibly moves toward target.
3. Switch on gravity and obstacles. Confirm failure feedback and safe motion stop in a blocked scenario, without silent penetration.
4. Run the unit and GUI smoke scripts on Windows. Save named pass/fail logs and a real screenshot/recording, including Python version, tested commit and machine details.
5. User and assistant both review this evidence and confirm each of M5–M10 against its specific acceptance gate.
6. Physics/3D limitations remain in scope: do not claim fully spatial rigid-body dynamics, 3D IK or industrial safety.

**Verification state:** M5–M10 code implemented, Linux tests PASS, native Windows/human acceptance PENDING.
