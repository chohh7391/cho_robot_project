# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
It holds the rules an agent works by and a map of where each fact is written down: a code fact in the
comment beside that code, the client contract in [cho_interfaces/CONTRACT.md](cho_interfaces/CONTRACT.md),
a package's internals in its README.md or DESIGN.md, operating procedures in [docs/](docs/). Change a fact
there, not here. How the project is meant to be used (the core and its project repositories, the
steps from a change to the robot, running the real FR3): [docs/usage_guidelines.md](docs/usage_guidelines.md).

## Build & Test

Run from workspace root (`~/fire_ws`):

```bash
colcon build --symlink-install
source install/setup.bash

# Scoped build during iteration
colcon build --packages-select cho_controller_franka

# Tests
colcon test --packages-select cho_task_manager cho_description_franka
colcon test-result --verbose
```

Dependencies, simulators and the full build: [docs/installation.md](docs/installation.md). `git pull` does not
check out a submodule added since the clone; `install_dependencies.bash` or `git submodule update --init
--recursive` does ([installation.md, ROS Dependencies](docs/installation.md#ros-dependencies)).

**Lint.** flake8 at 120 columns in every Python package, pycodestyle + pyflakes only, relaxed pep257; test files
are named `test_*.py`. Each package pins the optional flake8 plugins' rules off
(`extend-ignore = A,B90,C4,C81,CNL,D,I,Q`) in its `setup.cfg` or its `.flake8`, because ROS 2 machines and CI
have those plugins installed. A new Python package copies the pin, as in
[cho_robot_config/setup.cfg](cho_robot_config/setup.cfg) (ament_python, `test/test_flake8.py`) or
[cho_bringup_fr5/.flake8](cho_bringup/cho_bringup_fr5/.flake8) (ament_cmake, `ament_flake8`).

**Launch goldens.** Every bringup launch file is compared, per argument set in
`cho_bringup_common/test/launch_golden/cases.yaml`, against committed expected output. A launch change meant to
change behaviour fails it until the expected files are regenerated and the diff reviewed; a new launch file or
bringup needs cases. `test_golden_registry_names.py` checks the spawned controllers against the registry. Procedure: [test_launch_golden.py](cho_bringup/cho_bringup_common/test/test_launch_golden.py);
what is printed: [launch_golden.py](cho_bringup/cho_bringup_common/cho_bringup_common/launch_golden.py).

**CI.** [ci.yml](.github/workflows/ci.yml) builds and tests every package in its checkout (of the submodules only
`extern/openarm_can`) except `NOT_BUILT_PACKAGES`, each listed there with the reason; a new package needs no entry.
A package in the checkout that is not built goes in both `NOT_BUILT_PACKAGES` and `ROSDEP_SKIP_KEYS`; a key not in
the checkout at all goes in `ROSDEP_SKIP_KEYS` alone. A run that tests nothing fails. Why, and the
`rosdep --simulate` check for a change: ci.yml's header comment. Style and headers: [lint.yml](.github/workflows/lint.yml).

## Launch

`ros2 launch cho_bringup_<robot> bringup_<backend>_robot.launch.py`; `--show-args` lists the arguments. Every
robot and backend, the selection arguments and which controller exists where:
[docs/controllers_and_bringup.md](docs/controllers_and_bringup.md) ([Recipes](docs/controllers_and_bringup.md#recipes)).

```bash
# Franka, real (robot IP in cho_bringup_franka/config/real/franka.config.yaml)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller
# Franka, MuJoCo
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_qp_controller
# OpenArm, MuJoCo, legacy controllers (mujoco_mit_prototype:=true selects the MIT controllers)
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller
# OpenArm, real hardware: MIT controllers only, commissioning stage
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py controller_name:=task_space_impedance_mit_controller can_interface:=can0
# Behavior tree task
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=<robot> task:=<task_name> use_sim_time:=true
```

- OpenArm runs in MuJoCo, Isaac Sim and on real hardware. The real bringup takes the controllers in
  `launch_utils.REAL_MIT_DIRECT_CONTROLLERS` and is being commissioned:
  [docs/openarm_real_bringup.md](docs/openarm_real_bringup.md).
- Isaac Sim needs the robot's USD built once, and its own `ROS_DOMAIN_ID` when anything else on the machine
  simulates: [Recipes](docs/controllers_and_bringup.md#recipes).
- Tasks, including `task:=trajectory_replay`: [docs/tasks.md](docs/tasks.md). Two PCs: [docs/multi_pc.md](docs/multi_pc.md).

## Package Map

What each package is, and where its own documentation is. `extern/` holds the pinned vendor sources
([extern/README.md](extern/README.md)); they are never edited.

main is the core platform. A research project's code lives in its own repository, an overlay workspace on a pinned
core tag ([templates/overlay_project/README.md](templates/overlay_project/README.md)), not in these packages: what a
project needs from the core goes in as a general extension point, and code a second project needs moves into the core.

**cho_controller/**
- `cho_controller_common` — Pinocchio robot model, FK/IK/dynamics, Eigen math, TSID-derived tasks and QP solvers,
  Ruckig point-to-point trajectories. Built Release `-O3`
  ([CMakeLists.txt](cho_controller/cho_controller_common/CMakeLists.txt)).
- `cho_controller_base` — robot-independent: `GoalPhaseActionServer` and the `JointSpaceServer` / `TaskSpaceServer`
  every arm serves ([goal_phase_action_server.hpp](cho_controller/cho_controller_base/include/cho_controller_base/goal_phase_action_server.hpp)),
  DLS IK, and the held-command ledger `libcho_controller_base_ledger.so`
  ([held_command.hpp](cho_controller/cho_controller_base/include/cho_controller_base/held_command.hpp)). Every arm
  base lets go through `release_on_deactivate()` (velocity commands to 0, position commands recorded) and seeds a
  position controller with `live_held_command()`; Franka's position and velocity controllers also hand a moving
  command to the controller switched in with them (`live_held_motion()`), which brings it to rest.
  `testing/controller_manager_harness.hpp` steps a controller_manager on mock hardware (each arm's `test_reactivation`).
- `cho_controller_franka` — FR3: 13 plugins ([cho_controller_franka.xml](cho_controller/cho_controller_franka/cho_controller_franka.xml)),
  `ActionChunkController` and the Franka hand's `GripperController` among them
  ([instances](docs/controllers_and_bringup.md#franka-fr3--cho_controller_franka)).
- `cho_controller_fr5` — FAIRINO FR5: `JointSpacePositionController` and `TaskSpaceIKController` on `FR5BaseController`
  ([instances](docs/controllers_and_bringup.md#fairino-fr5--cho_controller_fr5)).
- `cho_controller_ur` — UR5e: `JointSpacePositionController` and `TaskSpaceIKController` on `URBaseController`
  ([instances](docs/controllers_and_bringup.md#ur5e--cho_controller_ur)).
- `cho_controller_gripper` — one `GripperController` for the UR, FR5 and OpenArm grippers: the `~/gripper` action and
  a `~/width_command` topic, in metres of opening
  ([gripper_controller.hpp](cho_controller/cho_controller_gripper/include/cho_controller_gripper/gripper_controller.hpp)).
- `cho_controller_openarm_mit` — every OpenArm arm controller plugin, two families: `cho_controller::openarm` (joint-space
  controllers, `ee_state_broadcaster`) and `cho_controller_openarm_mit` (the MIT drive-protocol producers)
  ([DESIGN.md, Package layout](cho_controller/cho_controller_openarm_mit/DESIGN.md#package-layout)).
- `cho_controller_chain` — chained-control prototype: a robot-independent chainable joint impedance law and the
  JointSpace / TaskSpace reference generators in front of it; Franka MuJoCo torque, loaded by hand
  ([README.md](cho_controller/cho_controller_chain/README.md)).
- `utils/cho_action_chunk` — the action-chunk pipeline, free of ROS, hosted by Franka's `action_chunk_controller` (through
  its `ActionChunkServer`) and OpenArm's `ActionChunkController` ([DESIGN.md](cho_controller/utils/cho_action_chunk/DESIGN.md)).
  Both hosts use `cho_action_chunk::Session` (chunk ingest, the per-cycle reference, the goal's lifetime, the telemetry
  values) and keep only their ROS adapter and control law ([DESIGN.md, Session](cho_controller/utils/cho_action_chunk/DESIGN.md#session)).

**cho_hardware/** — the OpenArm MIT `ros2_control` side ([protocol](docs/openarm_mit_contract_v1.md);
the three backends: [Consumers](docs/openarm_mit_contract_v1.md#consumers))
- `cho_openarm_mit_core` — the MIT tuple, session/lease/SAFE primitives, `SwitchGate`, safety-profile loading, the
  MuJoCo backend's tuple `Limiter`; no plugin ([DESIGN.md](cho_hardware/cho_openarm_mit_core/DESIGN.md)).
- `cho_hardware_openarm_mit_real` — `OpenArmMitRealSystem`, SocketCAN through the vendor `openarm_can`, one component
  per arm, the hand with `hand:=true` ([README.md](cho_hardware/cho_hardware_openarm_mit_real/README.md)).
- `cho_hardware_openarm_mit_mujoco` — `MitMujocoSystem` over `mujoco_ros2_control`: the per-arm safety limiter and
  drive-protocol consumer the MuJoCo MIT bringup runs.
- `cho_hardware_openarm_mit_test` — `FakeMitSystem`, a test-only fixture for the MIT contract.

**cho_interfaces/** — messages `ActionChunk`, `ChunkStreamTelemetry`, `PoseLog`, `GripperState`, `ObjectVisibility`,
`ObjectVisibilityArray`, `CameraVisibility`; actions `JointSpace`, `TaskSpace`, `Gripper`, `ChunkStream`.
What a client may rely on: [CONTRACT.md](cho_interfaces/CONTRACT.md).

**cho_description/**
- `cho_description_franka` — FR3 xacro entry points (`robots/`), generated URDFs (`urdf/fr3`, `urdf/fr3_with_ft_sensor`),
  MuJoCo XML, Isaac USD (generated, [usd/README.md](cho_description/cho_description_franka/usd/README.md)).
- `cho_description_fr5` — FR5 URDF and MuJoCo scenes from the vendor bundle, with a hardware-switching `ros2_control` block.
- `cho_description_ur` — UR description, upstream fork ([README.md](cho_description/cho_description_ur/README.md)).
- `cho_description_openarm` — OpenArm v1.0, vendored fork: one xacro entry point for every backend
  (`robots/openarm_v10/`), MuJoCo scenes per control mode, the MIT safety profiles
  (`config/mit_safety_profiles_v1.yaml`), Isaac USD ([usd/README.md](cho_description/cho_description_openarm/usd/README.md));
  `scripts/sync_mjcf_inertials.py` keeps the MJCF's inertials equal to the URDF's.

**cho_bringup/**
- `cho_bringup_common` — ament_python helpers every bringup imports (runtime parameter files, spawners, the Isaac
  start-up and gate order, search paths, `load_package_utils()`) and the launch golden test. Robot-specific names
  and rules stay in each robot's `utils/launch_utils.py`, installed with `PATTERN ... EXCLUDE`, never
  `FILES_MATCHING` (under `--symlink-install` that copies them, and the copy goes stale).
- `cho_bringup_franka`, `cho_bringup_ur`, `cho_bringup_fr5`, `cho_bringup_openarm` — `launch/bringup_<backend>_robot`
  (and `_moveit`) files, `config/<backend>/controllers*.yaml`, `config/isaac/robot_profile*.json` (the robot's Isaac
  physics); Franka also `config/payload.yaml` (the hand; `payload_ft_sensor.yaml` with the FT sensor,
  `load_ft_sensor`) and `config/real/franka.config.yaml` (robot IP, gripper).
  Which backends each has: [launch matrix](docs/controllers_and_bringup.md#bringup-launch-matrix).

**cho_simulation/cho_simulation_isaac/** — the Isaac runner and URDF-to-USD converter (`isaac/`, run under
`isaacsim/python.sh`, not ROS) and the ROS-side `scripts/isaac_command_gate.py`, `scripts/isaac_ft_sensor.py`.

**cho_moveit/** — `cho_moveit_common` (the MoveIt action bridge) and one planning configuration per robot
([README.md](cho_moveit/README.md)); the bridge's contract: [CONTRACT.md](cho_interfaces/CONTRACT.md#the-moveit-bridge).

**cho_robot_config/** — the per-robot registry, `config/<robot>.yaml`; its schema is the docstring of
[registry.py](cho_robot_config/cho_robot_config/registry.py), which also says how a project adds its own arm
controllers (`CHO_ROBOT_CONFIG_ADDITIONS`). Also `controller_action_name()`,
`task_goal_frame()`, `motion_limit_parameters()`.

**cho_task_manager/** — the py_trees task manager: `behaviors/`, `subtrees/`, `tasks/<robot>/` (registered in
`tasks/core.py`), `testing.py` (a task's goal checks), recordings in `config/replay/<robot>/`
([README.md](cho_task_manager/config/replay/README.md)). How a package registers a task and hands it `task_params:=`:
[README.md](cho_task_manager/README.md). Reference: [docs/tasks.md](docs/tasks.md).

**cho_control_tools/** — interactive action clients, action-chunk tools (`action_chunk_client`, `action_chunk_mit_probe`) and bag
plotters ([docs/action_clients.md](docs/action_clients.md)).

**cho_sensor/** (grouping directory)
- `bota_ft_sensor` — Bota FT launch over the Bota driver in `extern/` ([README.md](cho_sensor/bota_ft_sensor/README.md)).
- `cho_realsense` — D435 over the stock `rs_launch.py`; no detection, no camera-to-robot transform
  ([README.md](cho_sensor/cho_realsense/README.md)).
- `cho_oak` — OAK-D Pro W over the stock depthai launch ([README.md](cho_sensor/cho_oak/README.md)).
- `cho_camera_calibration` — target, procedure, per-camera `camera_info` files and `cameras.yaml`
  ([README.md](cho_sensor/cho_camera_calibration/README.md)).

**cho_perception/cho_object_pose/** — AprilTag `tag_<id>` TF and decode quality to a gated `PoseStamped` in the
robot's base frame, multi-camera fusion, `/perception/object_visibility`
([README.md](cho_perception/cho_object_pose/README.md)); operating it: [docs/apriltag_perception.md](docs/apriltag_perception.md).

**tools/** — not a package: `build_openarm_vendor.sh`; `ci/stub_ament_packages.sh` (the Franka stand-ins for the
golden test in CI) and `ci/check_copyright_headers.py`. **dds/** — the Fast DDS modes of
[docs/multi_pc.md](docs/multi_pc.md). **templates/overlay_project/** — not built (`COLCON_IGNORE`): a project
repository's starting point, one package registering one task and the `.repos` pin of the core
([README.md](templates/overlay_project/README.md)).

## Controllers

- Each controller package registers its plugins in its own XML (`cho_controller_<robot>.xml`,
  `cho_controller_gripper.xml`, OpenArm's `controller_plugins.xml`). Which instance runs on which backend:
  [controllers_and_bringup.md](docs/controllers_and_bringup.md#controllers-per-robot); the config's
  `controller_manager` section is the authority.
- Arm controllers derive from their robot's base class (`FrankaBaseController`, `FR5BaseController`,
  `URBaseController`, `OpenArmBaseController`: the Pinocchio model, realtime state, the `~/controller_state` and
  `~/ee_state` publishers) and override `update()`. The OpenArm MIT producers derive from `DirectControllerBase` or
  are the FollowJointTrajectory pair ([DESIGN.md](cho_controller/cho_controller_openarm_mit/DESIGN.md)).
- What every controller serves and a client may rely on (action names, goals, frames, `duration_sec` as a minimum,
  outcomes, state topics): [CONTRACT.md](cho_interfaces/CONTRACT.md). Python builds action names with
  `controller_action_name()` and goal frames with `task_goal_frame()`, never by hand.
- The JointSpace/TaskSpace servers are `cho_controller_base`'s; a robot's `servers/` code is an adapter, and the
  control thread never calls `rclcpp_action`. They time a goal from the Ruckig trajectory's `getDuration()`, never
  from the goal's `duration_sec`
  ([trajectory_euclidian.hpp](cho_controller/cho_controller_common/include/cho_controller_common/trajectory/trajectory_euclidian.hpp)).
- Which chunks OpenArm's `ActionChunkController` accepts, and the parameters it requires at configure time:
  its class comment ([action_chunk_controller.hpp](cho_controller/cho_controller_openarm_mit/include/cho_controller_openarm_mit/action_chunk_controller.hpp)).
- New or reworked controller parameters are declared with `generate_parameter_library` (the pattern:
  [task_space_ik_controller_parameters.yaml](cho_controller/cho_controller_fr5/src/task_space_ik_controller_parameters.yaml)):
  per-parameter ranges in the YAML, checks that relate parameters in code. Under Humble `ros2 param describe` shows
  no constraints (the controller_manager declares the overrides first), but every set is still validated. The other
  controllers declare by hand; convert them when they are next reworked.

## Control period

Code that advances its own trajectory clock or integrates an open-loop reference takes the period from its base
class's `nominal_period(period)` (`FrankaBaseController`, `FR5BaseController`, `OpenArmBaseController`,
`cho_controller_chain::ReferenceGenerator`): never
`1 / get_update_rate()` (0 unless the config sets a per-controller rate; only Franka's real one does), a literal `dt`, or the raw `period`
(jitters, and is 0 on a cycle with no new state). Audit a new controller for all three forms (`get_update_rate`,
a literal such as `dt = 0.001`, `period.seconds()`); a search for one misses the others. The reasons are in the
`nominal_period()` comment of [cho_controller_franka/src/base_controller.cpp](cho_controller/cho_controller_franka/src/base_controller.cpp).

Exceptions, with no `nominal_period()`:
- `cho_controller_gripper` uses the measured period capped at 0.1 s; a zero or non-finite period advances nothing
  (`bounded_dt()`, [gripper_controller.cpp](cho_controller/cho_controller_gripper/src/gripper_controller.cpp)).
- OpenArm MIT: `DirectControllerBase` and `TaskSpaceImpedanceController` add the measured period to their clock and
  timers only when it is in (0, 0.1 s); their feed-forward slew and idle-release blend take `bounded_dt()`, the
  gripper's rule (capped at 0.1 s, 0 for a zero or non-finite period); the FollowJointTrajectory pair samples by
  update time
  ([DESIGN.md, Control time](cho_controller/cho_controller_openarm_mit/DESIGN.md#control-time)).

A command write that a divide or an unbounded solve can reach has an `allFinite()` guard before it, like
`clip_torque()`: zero on a velocity interface, hold the previous command on a position interface.

No package compiles with `-Ofast`: it folds `isfinite()` / `isnan()` to constants
([cho_action_chunk/DESIGN.md, Finite math](cho_controller/utils/cho_action_chunk/DESIGN.md#finite-math)).

## Task trees

[docs/tasks.md](docs/tasks.md) is the reference. When writing a tree:
- Robot facts come from the registry (`robot_config` roles, poses, holds): the action leaves and
  `ChunkStreamCompletionWaiterBehavior` have no default controller, an exclusive switch without `robot_config` raises, and
  `ControllerNames` is Franka's names only.
- The standard root is `guarded_mission()` (a failure ends on the hold controller); shared fragments are in
  `cho_task_manager/subtrees/`.
- Waits use `behaviors/wait.WaitBehavior` (node clock), never `py_trees.timers.Timer`; node-clock deadlines go
  through `utils/clock.py`.
- A blackboard key that may be unwritten is read with `utils/blackboard.read_if_set()`.
- A task is registered, never listed: an entry point in the `cho_task_manager.tasks` group names its
  `{robot_type: builder}` mapping (this package's are in `tasks/core.py`). Names are global and a collision refuses
  to start. A new task takes its own parameters from `task_params:=` (`robot_config['task_params']`), not from a new
  node parameter, and its tests use `cho_task_manager.testing`
  ([README.md](cho_task_manager/README.md)).

## Frames

- TaskSpace goal frames, `ee_name`, and what `/ee_state/pose` carries:
  [CONTRACT.md, Frames](cho_interfaces/CONTRACT.md#frames) and [State topics](cho_interfaces/CONTRACT.md#state-topics).
- `ActionChunk.relative_frame` (`base` is the model's root frame, not the arm's `arm_base_link`):
  [ActionChunk.msg](cho_interfaces/msg/ActionChunk.msg).
- The FR3 FT sensor's mounting and its rotation to the TCP and world frames:
  [bota_ft_sensor README, Frames](cho_sensor/bota_ft_sensor/README.md#frames).

## Multi-PC Setup

Two PCs over Tailscale and a Fast DDS discovery server on PC1; every terminal sources `dds/dds_mode.sh`:
[docs/multi_pc.md](docs/multi_pc.md).

## Naming Conventions

- C++ classes are PascalCase (`TaskSpaceQPController`); source files are snake_case.
- Plugin names are `<package>/<Class>` (`cho_controller_franka/TaskSpaceQPController`), registered in the package's
  plugin XML. Controller instance names in the YAML configs and launch files are snake_case
  (`task_space_qp_controller`).
- Namespaces: `cho_controller::franka`, `::fr5`, `::ur`, `::openarm` (OpenArm's joint-space family) and `::common`
  (`cho_controller_common`); everywhere else the package name: the MIT family in `cho_controller_openarm_mit`,
  `cho_controller_gripper`, `cho_controller_base`, `cho_action_chunk` and the `cho_hardware/` packages.
- ROS link/joint/topic/controller names are stable — configs and tests depend on them. Do not rename without
  updating all YAMLs and launch files.
- Launch args and xacro properties: `snake_case`.
- Python: flake8 at 120 columns (see Build & Test).

## File Headers

Every C++, Python and CMake source outside `extern/` starts with a copyright and license header that the Lint
workflow checks; how to add one, and the form for code adapted from upstream: [NOTICE](NOTICE) ("File headers").
Never edit `extern/`: build on a vendor's package by adding it there as a submodule and including or referencing it.

## Build Alias
we set alias related to build below.
you can use this alias when you need to build some packages or entire packages.
alias cbr='MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install'
alias cbp='MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select'

Do not raise these limits. Prefer scoping the package set (`--packages-select`, `--packages-above`)
to keep a wide rebuild short instead of adding parallelism.
