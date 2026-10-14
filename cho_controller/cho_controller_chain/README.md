# cho_controller_chain

Prototype of chained control on Humble's ros2_control (controller_manager 2.54): one
robot-independent joint impedance law that exports per-joint reference interfaces, and two
reference generators in front of it that serve the `JointSpace` and `TaskSpace` actions. It runs
beside the existing controllers on the Franka MuJoCo torque bringup and no bringup spawns it.

## Plugins

| Plugin | Base class | Command interfaces | State interfaces | Serves |
| --- | --- | --- | --- | --- |
| `cho_controller_chain/JointImpedanceLaw` | `ChainableControllerInterface` | `<joint>/effort` | `<joint>/position`, `<joint>/velocity` | exports `<law>/<joint>/position`, `/velocity`, `/acceleration`; topic `~/controller_state` |
| `cho_controller_chain/JointSpaceGenerator` | `ControllerInterface` (`ReferenceGenerator`) | the law's reference interfaces | `<joint>/position`, `<joint>/velocity` | action `~/joint_space` |
| `cho_controller_chain/TaskSpaceGenerator` | `ControllerInterface` (`ReferenceGenerator`) | the law's reference interfaces | `<joint>/position`, `<joint>/velocity` | action `~/task_space` |

`<law>` is the law's node name. The generators claim the same interfaces, so one is active at a
time. The actions follow [cho_interfaces/CONTRACT.md](../../cho_interfaces/CONTRACT.md).

## The law

```
tau = M(q) (a_ref + kp .* (q_ref - q) + kd .* (v_ref - dq)) + C(q, dq) dq + g(q)
```

- The model is `robot_description` (the parameter, else robot_state_publisher's) with every joint
  not in `joints` locked at its neutral position. `rotor_inertia` is added to `M(q)`
  (Pinocchio `Model::armature`); the URDF carries none.
- `g(q)` is left out when `compensate_gravity` is false. `tau` is clipped per joint to
  `effort_limits`; a non-finite `tau` writes 0.
- Not chained (no generator active): it holds the pose measured at its activation, `v = a = 0`.
- Chained: the references a generator wrote in the same cycle (preceding controllers update
  first); a set with a non-finite value is replaced by the last finite one.
- `~/controller_state` (`control_msgs/JointTrajectoryControllerState`), every cycle: reference,
  feedback, error, and `output.effort = tau`.

## The generators

- On its first update after activation a generator takes the law's current reference as its hold
  when every position in it is finite, else the measured `q`; `v = a = 0`. Swapping generators
  therefore does not step the reference.
- With no goal it holds its last reference (`v = a = 0`).
- `JointSpaceGenerator`: a Ruckig trajectory from the current reference, limited by
  `joint_limits.<joint>.*`; writes the sample's position, velocity and acceleration.
- `TaskSpaceGenerator`: an SE3 trajectory starting at `FK(q_ref)`, limited by `cartesian_limits.*`.
  Each cycle one damped-least-squares step from `q_ref` toward the sample (local Jacobian,
  `ik_damping`; `ik_gain * dt` clamped to 0.5), each `|dq_i| <= max_joint_velocity * dt`, kept
  inside the URDF limits. Writes `q_ref`, `v_ref = dq / dt` and `a_ref = 0`.
- Success and timeout are judged on the measured state, as for every controller.
- `dt` is `ReferenceGenerator::nominal_period()`.

## Parameters

Declared with `generate_parameter_library` (`src/*_parameters.yaml`). Every parameter has a
default; configure fails on the conditions given.

| Controller | Parameter | Default | Unit, rule |
| --- | --- | --- | --- |
| law | `robot_description` | `""` | empty: robot_state_publisher's; still empty: fails |
| law | `joints` | `[]` | non-empty, in the model's kinematic order, each 1-DOF |
| law | `kp` | `[]` | 1/s^2, one per joint, finite, >= 0 |
| law | `kd` | `[]` | 1/s, one per joint, finite, >= 0 |
| law | `effort_limits` | `[]` | N m; empty: the URDF's; else one per joint, > 0 |
| law | `compensate_gravity` | `true` | |
| law | `rotor_inertia` | `[]` | kg m^2; empty: none; else one per joint, finite, >= 0 |
| both generators | `robot_description`, `joints` | `""`, `[]` | as the law's |
| both generators | `law_name` | `chain_impedance_law` | whose reference interfaces are claimed |
| TaskSpace | `ee_frame` | `""` | a frame of the model; the relative goal frame |
| TaskSpace | `ik_damping` | `0.01` | > 0 |
| TaskSpace | `ik_gain` | `50.0` | 1/s, > 0 |
| TaskSpace | `max_joint_velocity` | `2.0` | rad/s, > 0 |

The motion limits (`joint_limits.<joint>.*`, `cartesian_limits.*`) are declared by
`cho_controller_common`'s loaders, as for the other controllers.

## Running it on the Franka MuJoCo bringup

`cho_bringup_franka/config/mujoco/controllers.yaml` declares `chain_impedance_law`,
`chain_joint_space_generator` and `chain_task_space_generator` with their parameters. The bringup
hands the robot's motion limits only to the controllers it spawns, so write them to a file, load
the three inactive, then hand the arm over in one STRICT switch.

```bash
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller

python3 -c "import yaml; from cho_robot_config import motion_limit_parameters; \
yaml.safe_dump({'/**': {'ros__parameters': motion_limit_parameters('franka')}}, open('/tmp/chain_limits.yaml', 'w'))"
ros2 run controller_manager spawner chain_impedance_law chain_joint_space_generator chain_task_space_generator \
  --inactive -p /tmp/chain_limits.yaml

# the law and a generator in, the running arm controller out
ros2 control switch_controllers --strict --activate chain_impedance_law chain_joint_space_generator \
  --deactivate joint_space_impedance_controller
# swap generators: the law stays chained and is not restarted
ros2 control switch_controllers --strict --activate chain_task_space_generator \
  --deactivate chain_joint_space_generator
```

Goals go to `/chain_joint_space_generator/joint_space` and `/chain_task_space_generator/task_space`
(absolute: `base` or `fr3_link0`; relative: `fr3_hand_tcp`, which needs the hand, the default
`load_gripper:=true`).

- A generator swap is one STRICT request. Deactivating the last generator alone, or a BEST_EFFORT
  swap whose activation fails, takes the law out of chained mode: Humble restarts it and it holds
  the pose measured then. Entering chained mode restarts it the same way.
- The robot registry does not name these instances, so a task tree's exclusive switch does not take
  them down. Hand the arm back (a STRICT switch activating an arm controller and deactivating the law
  and the active generator) before running a task.

## Tests

`colcon test --packages-select cho_controller_chain`: `test_joint_impedance_torque` (the law's
math against Pinocchio), `test_chain_switch` (chained-mode entry, exit and a generator swap on the
`cho_controller_base` controller_manager harness) and `test_chain_generators` (the two actions).
