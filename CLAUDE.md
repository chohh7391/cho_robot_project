# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test

Run from workspace root (`~/ros2_ws`):

```bash
colcon build --symlink-install
source install/setup.bash

# Scoped build during iteration
colcon build --packages-select cho_controller_franka

# Tests
colcon test --packages-select cho_task_manager cho_description_franka
colcon test-result --verbose
```

Python linting enforces max line length 120 (flake8), relaxed pep257. Test files must be named `test_*.py`.

## Launch

```bash
# Real robot (set IP in cho_bringup_franka/config/real/franka.config.yaml first)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller

# Simulation
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position controller_name:=task_space_ik_controller
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_qp_controller

# VLA mode (requires control_mode set in controller config)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque vla:=true

# Behavior tree task
ros2 launch cho_task_manager run_task_manager.launch.py task:=<task_name>

# Gazebo black screen fix
export IGN_IP=127.0.0.1
```

## Architecture

### Package Map

```
cho_controller/
  cho_controller_common/     # Shared C++ math: Pinocchio FK/IK/dynamics, Eigen utilities
  cho_controller_franka/     # 12 ros2_control plugin controllers + action servers
  utils/cho_trajectory_smoother/  # Time-optimal trajectory generation

cho_interfaces/              # ROS2 msgs (ActionChunk, PoseLog, JointLog) and actions (JointSpace, TaskSpace, Gripper, VLA)

cho_description/cho_description_franka/
  robots/                    # Xacro entry points per robot variant
  urdf/                      # Generated URDFs (fr3, fr3_with_ft_sensor, etc.)
  xml/                       # MuJoCo scene XMLs
  config/                    # Payload YAML

cho_bringup/cho_bringup_franka/
  launch/                    # bringup_real/gazebo/mujoco_robot.launch.py
  config/{real,gazebo,mujoco}/controllers.yaml  # Per-environment controller gains
  config/real/franka.config.yaml               # Robot IP, gripper, FT sensor flags

cho_task_manager/
  cho_task_manager/
    behaviors/               # py_trees leaf nodes: action/, service/
    tasks/                   # Behavior trees, split per robot: franka/ (pick_and_place, forge), ur/ (pick_and_place, multi_move); __init__.py dispatches by robot_type via build_task_tree()
    task_manager_node.py     # ROS2 node that runs the selected tree
    utils/controller_names.py  # Loads config/robots/<robot>.yaml (single source of truth for controller roles)
  config/robots/             # Per-robot controller role config (franka.yaml, ur5e.yaml)
  python/                    # Standalone scripts: action_client, vla_action_client, plot_*

extern/
  franka_ros2/               # Official Franka ROS2 driver (do not edit)
  mujoco_ros2_control/       # MuJoCo hardware interface (do not edit)
  qpOASES/                   # QP solver used by QP controllers
```

### Controller Plugin Architecture

Controllers are `ros2_control` plugins registered in `cho_controller_franka.xml`. Each inherits from `BaseController` (Pinocchio robot model, gravity compensation, realtime state) and overrides `update()`.

Key controllers:
- `task_space_qp_controller` — operational-space QP with contact-aware force control
- `task_space_impedance_controller` — impedance control in Cartesian space
- `joint_space_qp_controller` — joint-level QP controller
- `vla_controller` — receives `ActionChunk` from VLA inference and streams joint/task commands
- `ee_state_broadcaster` — publishes `/ee_state/pose` and `/ee_state/twist` (Cartesian state used by Python tasks)
- `joint_trajectory_controller` — executes trajectories with joint logging to `/log/joint_states`

Action servers (`src/servers/`) wrap controllers to expose `cho_interfaces` action goals over ROS2.

### Task Manager / Behavior Tree Flow

`task_manager_node.py` instantiates a py_trees tree from `tasks/<task>.py`. Leaf behaviors in `behaviors/action/` (JointSpace, TaskSpace, Gripper) send goals to the controller action servers. `behaviors/service/` handles controller switching via `controller_manager`. The VLA flow adds `vla_controller` activation and a `VLACompletionWaiterBehavior`.

### Frame Conventions

- EE pose published on `/ee_state/pose` is **fr3_hand_tcp** frame (tip of gripper).
- FT sensor is mounted between `fr3_link8` and `fr3_hand`. Transform from FT sensor local frame to TCP frame: `R_tcp_ft = Rx(π) × Rz(-π/4)` (derived from URDF joints `flange_to_ft_sensor` and `fr3_hand_joint`).
- When using real FT sensor data: rotate from FT local frame to world using `q_ft_to_world = quat_mul(ee_quat, q_ft_to_tcp)`.

### Multi-PC Setup

Use FastDDS discovery server on PC2 and set `ROS_DISCOVERY_SERVER=<PC2_IP>:11811` on both machines before launching.

## Naming Conventions

- C++: namespaces match package names, controller classes in `snake_case` matching plugin names.
- ROS link/joint/topic/controller names are stable — configs and tests depend on them. Do not rename without updating all YAMLs and launch files.
- Launch args and xacro properties: `snake_case`.
- Python: `setup.cfg` enforces flake8 max-line 120.
