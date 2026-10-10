# Motion Integration Contract V1 (M14)

**Status:** Specified and supported by the simulator, but NOT yet certified against real `dev-collaboration-hub/robot-motion-control` or `dev-collaboration-hub/robotic-arm-control-system` code.

This is an *optional in-process Python interface*. It is deliberately separate from real motor drivers or network control. No hardware commands are allowed through this contract.

## External module interface

An external independently implemented module exports:

```python
def plan_joint_trajectory_v1(request: dict) -> dict:
    # request fields:
    # contract_version: 1
    # unit: "radians"
    # joint_names: list[str] (2 to 6)
    # start, end: list[float], length = joint_count
    # max_speed: list[float] (rad/s)
    # angle_limits: [[lower, upper], ...] (rad)
    # result fields:
    # contract_version: 1
    # duration: positive float (seconds)
    # velocity_limits: list[positive float] (rad/s)
    # acceleration_limits: list[positive float] (rad/s^2)
    ...
```

Example temporary fixture below is **a contract example, not evidence that any external repo has a working implementation**:

```python
def plan_joint_trajectory_v1(request):
    if request["contract_version"] != 1 or request["unit"] != "radians":
        raise ValueError("contract mismatch")
    # Fixed duration is only for this small deterministic sample.
    return {
        "contract_version": 1,
        "duration": 7.0,
        "velocity_limits": [1.0 for _ in request["joint_names"]],
        "acceleration_limits": [1.0 for _ in request["joint_names"]],
    }
```

The caller verifies the returned plan's dimensions, finite positive bounds, velocity and acceleration limits, and robot-specific maximum velocity, then constructs a C1 smoothstep trajectory. It rejects invalid or unsupported results; there is no silent fallback when an explicitly configured external adapter fails.

## Simulator wiring

```python
from robot_sim.arm import ArmModel
from robot_sim.advanced import RobotLab
from robot_sim.integration import MotionPlannerAdapter

model=ArmModel.basic()
adapter=MotionPlannerAdapter(model,external_module="name_of_installed_partner_module")
lab=RobotLab(model,planner=adapter)
lab.command_joints((.2,-.1))
lab.step_once()
```

No package installation or paid service is needed for **standalone local operation**:

```python
lab=RobotLab(ArmModel.basic())
```

## Required for true completion

1. Partner repository must independently expose the V1 function with tests and supported import/install instructions.
2. Clone/install that code locally; do not copy it into the simulator repo merely to claim integration.
3. Execute a real imported end-to-end scenario against the partner module, including an invalid plan rejection test.
4. Record both exact commits, test commands, output, success/failure and user review.
5. Test real-hardware integration separately if it is ever introduced. This simulation contract never proves physical safety.

Until that happens, **M14 is BLOCKED** even though adapter fixtures can test the interface shape.
