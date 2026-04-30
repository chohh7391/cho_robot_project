# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# simulation
cd ~/ros2_ws && colcon build --symlink-install

# real robot (Release required for real-time performance)
cd ~/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

source ~/ros2_ws/install/setup.bash
```

If an initial build fails due to missing symbols, repeat `source install/setup.bash && colcon build --symlink-install` until it succeeds (packages build in dependency order).

qpOASES (submodule) must be installed system-wide before first build:
```bash
cd extern/qpOASES && mkdir build && cd build && cmake .. && sudo make install
```

## Run

```bash
# Real robot (set robot_ip in cho_franka_bringup/config/real/franka.config.yaml first)
ros2 launch cho_franka_bringup bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_impedance_controller

# Gazebo simulation
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py control_mode:=position controller_name:=ik_controller

# MuJoCo simulation
ros2 launch cho_franka_bringup bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=operational_space_controller

# VLA mode (vla_controller overrides controller_name)
ros2 launch cho_franka_bringup bringup_real_robot.launch.py vla:=true control_mode:=torque

# Behavior tree task manager
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_and_place  # or forge

# Test clients
python3 cho_task_manager/python/action_client.py          # generic action client
python3 cho_task_manager/python/vla_action_client.py      # VLA client
python3 cho_task_manager/python/vla_action_with_csv.py --csv_path <PATH> --hz <HZ> --chunk_size <N>
```

Add `--ros-args -p use_sim_time:=true` to client scripts when running against simulation.

## Logging

```bash
ros2 bag record /log/ee_pose   # records desired, reference, and current EE pose
python3 cho_task_manager/python/plot_pose_log.py --path <DB3_PATH>
```

## Troubleshooting

- Gazebo black screen: `export IGN_IP=127.0.0.1`
- Two-PC setup via Tailscale: set `ROS_DISCOVERY_SERVER=<PC2_IP>:11811` on both PCs, run `fastdds discovery --server-id 0 -l <PC2_IP> -p 11811` on PC2 first.

---

## Architecture

### Package layout

| Package | Language | Role |
|---|---|---|
| `cho_interfaces` | IDL | ROS2 action/msg definitions |
| `cho_controller/cho_controller_common` | C++ | Robot kinematics, QP math, HQP solver, trajectories |
| `cho_controller/cho_controller_franka` | C++ | Franka-specific controllers + action servers (pluginlib) |
| `cho_controller/utils/cho_trajectory_smoother` | C++ | Trajectory smoother utility |
| `cho_robots_bringup/cho_franka_bringup` | Python | Launch files for real/gazebo/mujoco |
| `cho_robots_description/cho_franka_description` | URDF/xacro | Robot URDF and meshes |
| `cho_task_manager` | Python | py_trees behavior tree task manager |
| `extern/franka_ros2` | C++ (submodule) | Franka hardware interface |
| `extern/mujoco_ros2_control` | C++ (submodule) | MuJoCo ros2_control plugin |
| `extern/qpOASES` | C++ (submodule) | QP solver (system-installed) |

### Controller inheritance hierarchy

```
controller_interface::ControllerInterface   (ros2_control)
  └── FrankaBaseController                  (base_controller.hpp)
        ├── GravityCompensationController
        ├── JointSpaceImpedanceController
        ├── TaskSpaceImpedanceController
        ├── OperationalSpaceController
        ├── JointSpaceQPController
        ├── TaskSpaceQPController
        ├── IKController
        └── VLAController
```

`FrankaBaseController` handles all shared setup: joint state reading from `state_interfaces_`, Pinocchio forward kinematics (`compute_all_terms()`), mass matrix, Jacobian, NLE computation, hand gravity compensation, torque clipping, and `/log/ee_pose` publishing. Every subclass calls `FrankaBaseController::on_init/on_configure/on_activate/update` and then adds its own control law.

Controllers are registered as pluginlib plugins via `cho_controller_franka.xml`.

### Action server pattern

Each controller that accepts goals embeds a `BaseActionServer<ActionT>` subclass. The action server runs inside the controller process; `controller->update()` calls `action_server_->compute(time, state_)` each control cycle when a goal is active.

Action interfaces defined in `cho_interfaces/action/`:
- `TaskSpace.action` — target EE pose + duration + relative flag
- `JointSpace.action` — target joint configuration
- `Gripper.action` — gripper open/close
- `VisionLanguageAction.action` — model name + inference frequency (for VLA)

### VLA controller

`VLAController` accepts either `effort` (torque/task-space impedance) or `position` (IK/pseudo-inverse) as `control_mode`. When the VLA action server is not running, it holds the initial EE pose. The `vla_action_with_csv.py` client replays pre-recorded action chunks; `vla_action_client.py` streams goals live.

### Bringup flow

All three launch files (`bringup_real_robot.launch.py`, `bringup_gazebo_robot.launch.py`, `bringup_mujoco_robot.launch.py`) share the same `OpaqueFunction` pattern:

1. Read `franka.config.yaml` for namespace, robot_type, robot_ip, etc.
2. Merge `payload.yaml` (end-effector mass/CoM/inertia) with dynamic params into a temp YAML.
3. Spawn the appropriate subset of controllers (position or torque set) via `controller_manager spawner`, passing `-p <temp.yaml>`.
4. Always spawn `gripper_controller` and `ee_state_broadcaster`.
5. Active controller determined by `controller_name` (or `vla_controller` when `vla:=true`); others spawned `--inactive`.

`bringup_type` (`real`/`gazebo`/`mujoco`) is injected into every controller's `ros__parameters`. Controllers use this to switch behavior (e.g., gripper interface only available in sim, gravity direction differs in Gazebo).

### cho_controller_common internals

Provides robot-agnostic primitives used by QP controllers:
- `RobotWrapper` — thin Pinocchio wrapper for URDF loading and kinematic/dynamic queries
- `tasks/` — `TaskSE3Equality`, `TaskJointPosture`, `TaskContactForce`, `TaskMotion` (QP task definitions)
- `formulation/` — `InverseDynamicsFormulationAcc` (whole-body QP formulation)
- `solver/` — HQP solvers backed by eiquadprog and qpOASES
- `trajectory/` — `TrajectorySE3`, `TrajectoryEuclidian` (minimum-jerk/cubic interpolation)

### Task manager

`cho_task_manager` uses `py_trees`/`py_trees_ros` behavior trees. Add new tasks by creating a module under `cho_task_manager/tasks/` that exports a `create_<name>_tree()` function, then register it in `task_manager_node.py`.

### Key config files

| File | Purpose |
|---|---|
| `cho_franka_bringup/config/real/franka.config.yaml` | Robot IP, namespace, joint rate |
| `cho_franka_bringup/config/real/controllers.yaml` | Controller parameters (gains, etc.) |
| `cho_franka_bringup/config/mujoco/controllers.yaml` | Same for MuJoCo |
| `cho_franka_bringup/config/payload.yaml` | End-effector payload (mass, CoM, inertia) |
