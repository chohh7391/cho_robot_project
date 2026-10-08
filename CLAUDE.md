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
The rules are pycodestyle + pyflakes only: every package pins the optional flake8
plugins' rules off (`extend-ignore = A,B90,C4,C81,CNL,D,I,Q` in its `setup.cfg`, or
in a `.flake8` that its CMakeLists.txt hands to `ament_cmake_flake8`), because a ROS 2
machine has `python3-flake8-{docstrings,quotes,import-order,...}` installed and the
code was never written to them. CI installs those plugins too, so a package without
the pin fails there exactly as it does locally. A new Python package copies the pin.

Every bringup launch file is covered by a golden test: cho_bringup_common's
`test/test_launch_golden.py` evaluates each one for the argument sets in
`test/launch_golden/cases.yaml` (walking OpaqueFunctions and event handlers with
a real LaunchContext, starting nothing, Isaac and hardware stubbed) and compares
against `test/launch_golden/expected/`. A launch change that is meant to change
behaviour fails it until the expected files are regenerated and the diff reviewed:

```bash
colcon build --symlink-install --packages-select <changed bringup packages>
source install/setup.bash
cd src/cho_robot_project/cho_bringup/cho_bringup_common
CHO_LAUNCH_GOLDEN_UPDATE=1 python3 -m pytest test/test_launch_golden.py
git diff test/launch_golden/
# One case by hand: python3 -m cho_bringup_common.launch_golden <pkg> <file> [name:=value ...]
```

A new launch file or bringup package needs cases there, or the test fails. The
walk follows `cho_bringup_*` and `cho_moveit_*` includes (MoveIt's scene gate and
RViz condition are part of a bringup's output), prints each node's parameters,
the joints' origins and limits, every xacro run (Command and `xacro.process_file`)
and spawner `-p` files in the order given (Humble: the last one wins). CI evaluates
all four bringups: their hardware and Franka source-only dependencies are skipped
(`ROSDEP_SKIP_KEYS` in ci.yml) and the two Franka packages a launch looks up by path
are empty stand-ins (`tools/ci/stub_ament_packages.sh`). Expected output never
depends on where the workspace is installed: paths become tokens before anything is
formatted.

CI (`.github/workflows/ci.yml`) derives its rosdep paths from BUILD_PACKAGES, so
every workspace package a listed package depends on, of any dependency type, must
be listed too or named in ROSDEP_SKIP_KEYS - `rosdep --ignore-src` reports any other
workspace name as an unknown key. Check with `rosdep install --simulate --rosdistro
humble --ignore-src --from-paths $(colcon list --paths-only --packages-select
<BUILD_PACKAGES>) --skip-keys "<ROSDEP_SKIP_KEYS>"` in a shell that has sourced only
/opt/ros/humble.

Submodules: `git pull` does not check out a submodule added since the clone
(`extern/bota_driver_ros2_example` was). `install_dependencies.bash` checks out the
missing ones (only those) before running rosdep; by hand, `git submodule update
--init --recursive`.

## Launch

```bash
# Real robot (set IP in cho_bringup_franka/config/real/franka.config.yaml first)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller

# Simulation
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position controller_name:=task_space_ik_controller
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_qp_controller
# Isaac Sim (build the USD once first: cho_description_franka/usd/README.md)
ros2 launch cho_bringup_franka bringup_isaac_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller
ros2 launch cho_bringup_ur bringup_isaac_robot.launch.py controller_name:=task_space_ik_controller
# OpenArm: simulation only. bimanual:=true for the two-arm torso; physics_engine
# is physx (default) or newton - both verified for torque/position/velocity.
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller
ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller physics_engine:=newton bimanual:=true
# Isaac: give it its own ROS_DOMAIN_ID if anything else on the machine simulates -
# two /clock publishers make sim time jump backwards and every controller misbehaves.
# If the requested controller fails to activate, the command gate stays closed and
# Isaac holds the home pose with one ERROR line to show for it; for headless or
# unattended runs pass shutdown_on_gate_failure:=true to end the launch instead.

# VLA mode (requires control_mode set in controller config)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque vla:=true
# OpenArm VLA: the MIT prototype path only. Verified in MuJoCo; the real bringup
# deliberately does not offer it (see launch_utils.REAL_MIT_DIRECT_CONTROLLERS).
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque mit_controller_name:=vla_mit_controller
# End-to-end check against a running sim: streams chunks like a bridge would and
# asserts what the arm and the telemetry actually did.
ros2 run cho_control_tools vla_mit_probe

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
  cho_controller_base/       # Robot-independent, header-only except one small
                             # library: the RT-safe action server (GoalPhase), the
                             # JointSpace/TaskSpace servers every arm serves, DLS IK
                             # and held-command helpers. Robots keep only thin
                             # adapters (state fields, defaults).
                             # libcho_controller_base_ledger.so is the process-wide
                             # record behind live_held_command(): every base records
                             # what it leaves in on_deactivate (release_held_command),
                             # and a position controller seeds from the command
                             # interface only while that is still a live hold (the
                             # joint has not moved more than 1 urad since); a value
                             # from a controller outside this repo is judged by the
                             # 0.05 rad band. ur_robot_driver resets position commands
                             # to the measurement on every position-mode start.
                             # testing/controller_manager_harness.hpp is test support:
                             # a controller_manager on mock hardware that a test steps
                             # one period at a time (each arm's test_reactivation).
  cho_controller_franka/     # 12 ros2_control plugin controllers + action servers
  utils/cho_vla_core/        # Robot-independent VLA action-chunk pipeline: chunk
                             # validation, observation-time splicing, reference
                             # sampling/limiting, stream watchdog, gripper edge
                             # detection. NO ROS (rclcpp is not a dependency) and
                             # no clock of its own - time is a plain double on the
                             # host's control clock, so the whole pipeline is
                             # gtest-able without a controller_manager fixture.
                             # Consumed by cho_controller_franka's VLAActionServer
                             # and cho_controller_openarm_mit's VlaController.
                             # See its DESIGN.md.

cho_interfaces/              # ROS2 msgs (ActionChunk, VlaTelemetry, PoseLog) and actions (JointSpace, TaskSpace, Gripper, VLA)

cho_description/cho_description_franka/
  robots/                    # Xacro entry points per robot variant
  urdf/                      # Generated URDFs (fr3, fr3_with_ft_sensor, etc.)
  xml/                       # MuJoCo scene XMLs
  usd/                       # Isaac Sim USD assets (generated, gitignored; see usd/README.md)
  config/                    # Payload YAML

cho_bringup/cho_bringup_franka/
  launch/                    # bringup_real/gazebo/mujoco/isaac_robot.launch.py
  config/{real,gazebo,mujoco,isaac}/controllers.yaml  # Per-environment controller gains
  config/real/franka.config.yaml               # Robot IP, gripper, FT sensor flags

cho_bringup/cho_bringup_common/  # ament_python; what every bringup shares, imported as
                                 # `from cho_bringup_common import ...`: the runtime
                                 # param file (write_runtime_param_file + cleanup),
                                 # spawners (create_controller_spawners), the Isaac
                                 # start-up/gate order, search-path prepending, and
                                 # load_package_utils() for a robot's own
                                 # lib/<pkg>/utils/launch_utils.py. Robot-specific
                                 # names and rules stay in those robot utils.
                                 # Those utils/ dirs are installed with PATTERN
                                 # EXCLUDEs, never FILES_MATCHING, which makes
                                 # --symlink-install copy them (and go stale).

cho_description/cho_description_openarm/   # enactic OpenArm v1.0, vendored fork
  robots/openarm_v10/        # ONE xacro entry point for real/gazebo/mujoco/isaac/mock
  xml/openarm_v10{,_bimanual}/   # MuJoCo scenes, one per control_mode
  usd/                       # Isaac USD (generated, gitignored; see usd/README.md)
  scripts/sync_mjcf_inertials.py  # keeps the MJCF's inertials/axes equal to the URDF

cho_controller/cho_controller_openarm_mit/  # every OpenArm controller plugin
  # Two families in one package, and the namespaces say which is which.
  #
  # namespace cho_controller::openarm - ee_state_broadcaster + joint_space
  # impedance/position/velocity, merged in from the former
  # cho_controller_openarm. Dynamic-size Eigen and name-based Pinocchio
  # indexing, so one class serves both the single arm and either arm of the
  # bimanual torso. ee_state_broadcaster is NOT optional for the MIT path:
  # every MIT config spawns it, the MIT controllers have no broadcaster of
  # their own, and the Inverse3 teleop bridge takes its observation from the
  # /ee_state/<side>/pose it publishes.
  #
  # namespace cho_controller_openarm_mit - the MIT drive-protocol controllers:
  # joint position/impedance, task-space impedance, VLA, and the FJT pair.
  #
  # Plugin lookup names all carry the cho_controller_openarm_mit/ prefix; the
  # four merged ones were renamed from cho_controller_openarm/ when the
  # packages joined, and every config moved with them.

cho_bringup/cho_bringup_openarm/          # mujoco + isaac only (no real hardware yet)
  config/{mujoco,isaac}/controllers{,_bimanual}.yaml
  config/isaac/robot_profile{,_bimanual}.json  # OpenArm-owned Isaac physics
  utils/launch_utils.py      # per_arm() prefixes controller names on a bimanual build

cho_simulation/cho_simulation_isaac/   # Shared by every robot's Isaac bringup
  isaac/run_isaac_sim.py     # Isaac standalone runner (runs under isaacsim/python.sh, NOT ROS)
  isaac/convert_urdf_to_usd.py  # URDF -> USD, also under isaacsim/python.sh
  scripts/                   # ROS-side helpers: isaac_command_gate.py, isaac_ft_sensor.py

cho_bringup/cho_bringup_<robot>/config/isaac/
  robot_profile.json         # Robot-owned Isaac physics: joints, home, armature, drive gains

cho_task_manager/
  cho_task_manager/
    behaviors/               # py_trees leaf nodes: action/, service/
    tasks/                   # Behavior trees, split per robot: franka/ (pick_and_place, forge), ur/ (pick_and_place, multi_move); __init__.py dispatches by robot_type via build_task_tree()
    utils/occlusion.py       # ROS-free: when a recovery sweep is worth doing, when
                             # it worked, and the sweep-table schema. Paired with
                             # behaviors/action/occlusion_sweep.py, the leaf that
                             # conducts it -- a LEAF and not something outside the
                             # tree, because only a leaf can read the visibility
                             # topic and judge its own success.
    config/sweep/            # Where the wrist camera goes to look, per bench. A
                             # boustrophedon raster at a fixed tool tilt, swept far
                             # to near and repeated lower, stopping at the first
                             # viewpoint whose decode clears min_decision_margin.
                             # SIDEWAYS and not just downwards: the spacing is for
                             # PARALLAX, not coverage (one viewpoint already covers
                             # the area at this FOV), because a different line of
                             # sight is the only thing that helps when something is
                             # in the way. <bench>.raster.yaml is the spec a person
                             # edits; <bench>.yaml is SOLVED from it by
                             # scripts/solve_sweep_raster.py and is not hand-edited
                             # -- every waypoint has to stay in one IK branch.
                             # The task latches the pose BEFORE returning: a
                             # recovered pose expires with the aggregation window.
    task_manager_node.py     # ROS2 node that runs the selected tree
    utils/controller_names.py  # Compatibility view of cho_robot_config/config/<robot>.yaml

tools/                       # Not a package. build_openarm_vendor.sh; ci/: the
                             # Franka stand-ins for the launch golden test
                             # (stub_ament_packages.sh) and the CMakeLists.txt
                             # header check (check_copyright_headers.py).

cho_robot_config/
  config/*.yaml              # Per-robot registry: controller/action roles,
                             # controllers.additional_arm (arm controllers with
                             # no role, which an exclusive switch must still take
                             # down), poses.home (operator presets) and
                             # poses.task_home (where task trees home; a home
                             # selector or a vector, read by task_home_pose()).

cho_control_tools/
  cho_control_tools/         # Interactive clients, VLA tools, and bag plotters

cho_sensor/                  # Sensor stacks; grouping directory, not a package
  bota_ft_sensor/            # Bota FT driver launch for the arms here. Uses Bota's own
                             # bota_driver (extern/bota_driver_ros2) and, for its default
                             # config, bota_driver_example (extern/bota_driver_ros2_example)
                             # instead of copies; only what differs lives here.
  hansung_scale/             # Hansung HS-AA RS232 scale driver + its msgs.
                             # SELF-CONTAINED: no cho_* dependencies, meant to be
                             # usable as a standalone module. Do not entangle it
                             # with cho_interfaces or the robot verticals.
  cho_realsense/             # D435 only. Includes the stock rs_launch.py and hands
                             # it our config_file, bota-style. No detection, and
                             # deliberately NO camera->robot transform.
                             # Two identical D435s are connected: set serial_no.
  cho_oak/                   # OAK-D Pro W, over the stock depthai camera.launch.py.
                             # RGBSTEREO so the GLOBAL-SHUTTER mono pair is published
                             # (the default RGBD gives depth instead), and the IR dot
                             # projector off or its pattern breaks tag decoding.
                             # Mono comes out unrectified -- detector needs rectify.
  cho_camera_calibration/    # Checkerboard target, procedure, and per-camera
                             # camera_info files + cameras.yaml (model, both serial
                             # numbers, firmware). Intrinsics belong to a stream
                             # PROFILE, not a camera. NOTE: realsense2_camera has no
                             # camera_info_url and no set_camera_info, so a D435
                             # calibration cannot be fed back without a relay node
                             # that does not exist yet; depthai and v4l2_camera both
                             # accept one.

cho_perception/              # Perception that knows a robot; grouping directory
  cho_object_pose/           # tag_<id> TF + decode quality -> a gated PoseStamped in
                             # the robot's base frame, which PoseTargetBehavior
                             # latches. geometry.py is ROS-free and holds everything
                             # worth testing; node.py is the tf2 adapter. See its
                             # README, and docs/apriltag_perception.md.
                             #
                             # cameras.yaml carries a per-camera `priority`. Equal
                             # priorities are FUSED (median); a higher one REPLACES
                             # the lower ones' samples for the objects it can see --
                             # a 200 mm close-up and a 1 m wide view are not two
                             # measurements of one quantity. The override is decided
                             # by what is in the aggregation window, so it clears
                             # itself after window_sec with no lifetime of its own,
                             # and min_cameras is NOT applied to an object while it
                             # is in force (requiring consensus and declaring one
                             # camera authoritative are contradictory). visibility.py
                             # holds that rule, ROS-free.
                             #
                             # /perception/object_visibility
                             # (cho_interfaces/ObjectVisibilityArray) publishes what
                             # every camera can see of every object -- the reasons
                             # the node always logged, in a form a behaviour tree can
                             # branch on, with each camera's decision_margin and
                             # edge_px. That topic is the trigger for the FR5
                             # occlusion recovery.

extern/
  franka_ros2/               # Official Franka ROS2 driver (do not edit)
  mujoco_ros2_control/       # MuJoCo hardware interface (do not edit)
```

### Controller Plugin Architecture

Controllers are `ros2_control` plugins registered in `cho_controller_franka.xml`. Each inherits from `BaseController` (Pinocchio robot model, gravity compensation, realtime state) and overrides `update()`.

Key controllers:
- `task_space_qp_controller` — operational-space QP with contact-aware force control
- `task_space_impedance_controller` — impedance control in Cartesian space
- `joint_space_qp_controller` — joint-level QP controller
- `vla_controller` — receives `ActionChunk` from VLA inference and streams joint/task commands.
  The chunk semantics live in `cho_vla_core`; this controller keeps only the three
  control laws (effort / position / velocity) and its `VLAActionServer` is a thin
  ROS adapter over that core. OpenArm's equivalent is
  `cho_controller_openarm_mit/VlaController`, which derives from
  `TaskSpaceImpedanceController` and overrides `write_task_target()` alone, so
  both action spaces run drive-side impedance and the MIT session/ACK/lease/SAFE
  protocol is inherited unchanged. Two invariants there that the base class does
  NOT enforce and the VLA path makes mandatory at configure time:
  `max_reference_offset` (the only bound on `kp*(q_des - q)`, which the drive
  applies downstream of `torque_limit`) and `stream_timeout_sec > 0`.
  `max_task_wrench` is inert under `drive_side_impedance` but is still validated
  as six positive values, so every controller config derived from that base needs
  it.
- `ee_state_broadcaster` — publishes `/ee_state/pose` and `/ee_state/twist` (Cartesian state used by Python tasks)

Each arm controller publishes its state on **per-controller namespaced topics** (`BaseController`):
`/<controller>/controller_state` (`control_msgs/JointTrajectoryControllerState`, reference=desired / feedback=current)
and `/<controller>/ee_state` (`cho_interfaces/PoseLog`, Cartesian). These replaced the old global `/log/joint_pos` and `/log/ee_pose`. Plot via `ros2 run cho_control_tools plot_joint_pos_log --topic <t>` / `ros2 run cho_control_tools plot_pose_log --topic <t>`.

New or reworked controller parameters are declared with `generate_parameter_library`
(the pattern: `cho_controller_fr5/src/task_space_ik_controller_parameters.yaml`):
per-parameter ranges go in the YAML, checks that relate parameters stay in code. Under
Humble the controller_manager declares parameters from the YAML overrides before the
listener does, so `ros2 param describe` shows no constraints, but every set is still
validated (an out-of-range `ros2 param set` is refused). The other controllers still
declare by hand; convert them when they are next reworked.

Action servers (`src/servers/`) wrap controllers to expose `cho_interfaces` action goals over ROS2.
What every controller serves and a client may rely on is written down in
`cho_interfaces/CONTRACT.md`: actions are relative to the controller's node
(`/<controller>/joint_space`, `/task_space`, `/gripper`, `/vla`; the old
`/controller_action_server/<controller>` namespace is gone), goals take
`duration_sec`, JointSpace goals may name their joints, and a
TaskSpace goal is a `PoseStamped` that must be in the model's root frame (absolute) or
the EE frame (relative) -- controllers never transform, they reject. Python builds
every name through `controller_action_name(controller, kind)`, never by hand: the rule
lives in `cho_robot_config` (which validates `actions.preferences` against it),
`cho_task_manager/utils/controller_names.py` wraps it, and
`cho_control_tools/action_names.py` is the operator clients' registry-free copy. The
MoveIt bridge serves `~/joint_space` / `~/task_space` too, under the node name
`cho_robot_config.moveit_bridge_node()` gives (`<robot>[_<profile>]_moveit_action_bridge`),
and refuses to start under any other. Those names are absolute, so a ROS namespace is
NOT transparent (CONTRACT.md, Names).

Clients name and stamp their goals from the registry: JointSpace targets carry the
profile's `model.joints` (`arm_joint_names()`; per-arm prefixed on a bimanual build),
and TaskSpace goals `cho_robot_config.task_goal_frame(config, relative)` --
`model.absolute_goal_frame` / `model.relative_goal_frame`, `''` where undeclared.
`model.arm_base_link` is NOT the absolute goal frame everywhere (OpenArm bimanual arms'
`link0` is off the torso root; the trees there say `world`).
`cho_task_manager/test/test_goal_frames.py` expands every bringup's description with
its xacro mappings and proves each absolute goal frame is a `root_frames()` member. The
MoveIt bridge keeps `duration_sec` a minimum: plan-only, then the plan is slowed
uniformly to `duration_sec` (never sped up) and executed via `ExecuteTrajectory`; its
planning budget is its own `planning_time_sec`. Humble's move_group (2.5.x) accepts an
`ExecuteTrajectory` cancel and ignores it, and answers it only after the trajectory has
ended, so the bridge cancels by publishing `"stop"` on move_group's
`trajectory_execution_event` (repeated until terminal; CANCELED or ABORTED/PREEMPTED
count as a cancel). It refuses goals while nothing subscribes to that topic, and
publishes stop on SIGINT/SIGTERM with an execution in flight. Verified in FR5 MuJoCo:
the arm stops within 0.25 s.

The JointSpace and TaskSpace servers themselves are shared: `cho_controller_base`'s
`JointSpaceServer` / `TaskSpaceServer` over `GoalPhaseActionServer`, with each robot
supplying only an adapter (which state field is the measured joints, where a goal
starts, success-tolerance defaults). The control thread never touches `rclcpp_action`:
`compute()` only stores atomics, and a 5 ms non-RT timer makes the terminal calls and
publishes progress feedback. Every base controller owns a `ControllerActivity` that
its `on_activate`/`on_deactivate` update and its servers are attached to
(`attach_activity`): goals are REJECTED while the controller is inactive, and a goal
is ABORTED, with the reason in the result's `message`, when its controller is
deactivated -- it used to stay active with no result and block every later goal.

Their point-to-point motion is `cho_controller_common`'s `TrajectoryEuclidianRuckig` /
`TrajectorySE3Ruckig`: Ruckig's fastest motion within the robot's limits, slowed
uniformly to the goal's `duration_sec`. The duration is therefore a minimum, and a goal
faster than the limits allow takes longer; the servers time success and timeout from
`trajectory_->getDuration()`, never from the goal's `duration_sec`. The limits are the
robot's MoveIt files, not a controllers.yaml: `joint_limits.yaml` (ros2_control's
`joint_limits.<joint>.*` schema) and, if present, `pilz_cartesian_limits.yaml`, which
each bringup's runtime-params helper merges in through
`cho_robot_config.motion_limit_parameters()`. A joint with no `max_acceleration`
leaves its goals exactly as long as requested; the controller logs which applies.

**Controllers that advance their own trajectory clock or integrate an open-loop
reference** (`joint_space_position`, `joint_space_velocity`, `task_space_velocity`,
`task_space_ik`, `vla_controller` in its position/velocity modes) must take the
per-cycle period from `nominal_period(period)`, never from `1 / get_update_rate()`,
never from a hardcoded constant, and never from the raw measured period.
`get_update_rate()` returns 0 whenever a controller inherits the controller_manager's
rate, which is every controller here — no config sets a per-controller `update_rate`.
The old `: 0.001` fallback was silently correct only at a 1 kHz controller_manager
(MuJoCo); at Isaac's 250 Hz it stretched every goal 4x, which looks like a weak drive
rather than a clock bug. Fixed in both `FrankaBaseController` and
`OpenArmBaseController`.

Two variants of the same bug hid from the `get_update_rate()` sweep and were fixed
separately (2026-09-08), so grep for all three forms when auditing a new controller:

- a **literal** `const double dt = 0.001` (`task_space_ik_controller`), which no
  search for `get_update_rate` finds;
- the **raw measured period** (`vla_controller`'s position/velocity reference
  integrator). Besides the jitter this feeds into the commanded rate, `period` is
  exactly 0 on any cycle that saw no new state — which the Isaac config documents as
  routine when `update_rate` and the `/clock` rate disagree — so
  `(q_ref - q_ref_prev) / period` produced NaN, and `std::clamp` propagates NaN
  rather than sanitizing it.

Because of that last point, every command write that can be reached by a divide or an
unbounded solve now carries an `allFinite()` guard before it, in the style of
`clip_torque()`: zero on a velocity interface, hold the previous command on a position
interface.

### Task Manager / Behavior Tree Flow

`task_manager_node.py` instantiates a py_trees tree from `tasks/<task>.py`. Leaf behaviors in `behaviors/action/` (JointSpace, TaskSpace, Gripper) send goals to the controller action servers. `behaviors/service/` handles controller switching via `controller_manager`. The VLA flow adds `vla_controller` activation and a `VLACompletionWaiterBehavior`.

Shared tree fragments live in `cho_task_manager/subtrees/`, not copied per task:

- `home_subtree()` — the switch → go-home → open-gripper block four trees used to
  carry their own copy of. It takes `robot_config`, which is what makes the
  exclusive switch derive its deactivate list from the robot's own registry entry,
  and opens that robot's own `robot_config['gripper']`. The pose comes from
  `home_joint_state(robot_config)`, i.e. the registry's `poses.task_home`.
- `guarded_mission()` — the standard root, `OneShot -> Selector(mission, safe abort)`.
  A failing leaf used to propagate straight to the root and shut the node down with
  the last-driven controller still active. Now the abort branch runs first: it
  switches to the hold controller and verifies with `list_controllers` that the
  switch took, because the exclusive switch path is BEST_EFFORT and activating a
  controller the bringup never loaded leaves `result.ok` true. `Inverter(FailureIsSuccess(...))`
  keeps the root reporting FAILURE, so a successful abort is never mistaken for a
  successful mission.
- `tare_ft_children()` — FT tare plus its settle wait, spliced ahead of the home
  block by the contact-rich forge tasks.

Robot facts are never written into the task manager. The action leaves, the sweeps
and `VLACompletionWaiterBehavior` have no default controller (a tree passes the
robot config's role), an exclusive switch without `robot_config` raises, and
`ControllerNames` is Franka's names only, kept for the Franka trees and pinned to
`franka.yaml` by `test_controller_names`. Waits use `behaviors/wait.WaitBehavior`
(node clock, so sim time under `use_sim_time`), never `py_trees.timers.Timer`,
which counts wall time.

Motion targets can come from the blackboard instead of being fixed at tree-build
time. `TaskSpaceActionBehavior(target_pose_key=...)` / `JointSpaceActionBehavior(target_joints_key=...)`
resolve the target in `initialise()`, which py_trees calls immediately before the
goal is sent; exactly one of literal / key is required. `behaviors/topic/pose_target.py`
(`PoseTargetBehavior`) is the producer side — it latches a `PoseStamped` off a
topic into a key. Namespaces live in `utils/blackboard.py` (`/task` for motion
targets, `/mit_tuning` for the tuning task's measurements) so producer and consumer
cannot drift. Nothing transforms frames, so `PoseTargetBehavior.required_frame`
has no default and a pose from another frame is rejected rather than driven to.

Note for any blackboard read: py_trees raises `KeyError` for a registered but
unwritten key, and `getattr(board, key, None)` does **not** absorb it — that
KeyError escapes `update()` and takes the node down. Use
`utils/blackboard.read_if_set()`.

`BaseActionBehavior.terminate()` cancels a goal that may still be running for EVERY
status, not only INVALID: one already accepted at once, one still awaiting acceptance
when it is accepted (a sweep can fail on the tick after it sent a waypoint). The leaf
records the cancels it still owes (`cancels_outstanding()`), and
`task_manager_node.shutdown_task_manager()` spins, at most 1 s, until none is left --
also when the tree ended SUCCESS/FAILURE -- with Ctrl-C/SIGTERM held from before
`tree.stop()` to the end of that flush (an interrupt that lands before the handlers
are in place restarts the stop-and-flush).
Every deadline a behaviour takes on the node clock -- goal timeouts, waits, sweep
ceilings, dwells, latch windows, outage clocks -- goes through `utils/clock.py`: none
is set while the clock reads 0 (sim time before the first /clock), so it starts from
the first real reading instead of being missed the moment /clock arrives.

`guarded_mission(..., monitor=...)` adds a watchdog branch beside the mission via
`subtrees/watched_mission()`: `Parallel(SuccessOnSelected([mission]))` with
`behaviors/topic/safety_monitor.py`'s `SafetyMonitorBehavior`. It must be a
Parallel, not a decorator — py_trees invalidates the sibling branch on a trip, so
the running action leaf gets `terminate(INVALID)` and `BaseActionBehavior` cancels
its goal there; a decorator returning FAILURE would leave the goal running. The
guards are FT wrench magnitude, joint-limit proximity, and two Jacobian indices —
`sqrt(det(J Jᵀ))` (Yoshikawa; **not** `det(J)`, which does not exist for the 7-DOF
arms) and `sigma_min(J)`, both from the `LOCAL_WORLD_ALIGNED` Jacobian, never
`WORLD`. Every guard is off unless its threshold is given, staleness counts as a
trip (the arming window waits only for a first sample of each input and for the
description; an input seen and then quiet trips at once), and thresholds are commissioning values: `report_period_sec` logs the
measured numbers to set them from. These are supervisory at the 100 ms tick rate,
not a replacement for the 1 kHz `clip_torque()` / `clip_position()` guards.

Which controller can hold the arm is `control_mode`-dependent — the description
exports one command interface per joint, so the position hold is not loaded in a
torque bringup. `cho_robot_config` carries `controllers.hold_by_control_mode` per
robot and per profile (a bimanual profile must restate it: `controllers` is merged
key-by-key, so it would otherwise inherit unprefixed names `per_arm()` never
spawns). Each task declares the mode it is written for; `control_mode:=` on the
launch overrides it, and an undeclared mode raises at tree-build time.

### The `-Ofast` / `isfinite()` trap

`-Ofast` implies `-ffinite-math-only`, which folds `std::isfinite()` / `isnan()`
to constants. Measured on g++ 11.4: an `-Ofast` build reports a NaN-carrying
vector as all-finite, with no warning. **No package here uses `-Ofast`, and none
should**: `cho_controller_common` used it until 2026-10-04, where it also allowed
the dense QP solver's `objective == infinity` infeasibility test to be compiled
away. It is now Release (`-O3`), and the task-space QP measured the same in
MuJoCo (about 10 us per solve either way).

`cho_vla_core` still pins `-fno-finite-math-only` explicitly, which beats `-Ofast`
regardless of flag order (also measured), and `test_finite_math_guard` fails the
build if that flag is ever removed. `cho_controller_common`'s
`test_point_to_point` fails if the library is built with finite-math (Ruckig's
templates rely on `isnan()`/`isinf()`).

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

## File Headers

Every C++, Python and CMake source outside `extern/` starts with a copyright and
license header that `ament_copyright` recognises, `setup.py` and `CMakeLists.txt`
included; the Lint workflow checks the whole tree. `ament_copyright` never selects a
`CMakeLists.txt` (and would comment one with `//`), so `tools/ci/check_copyright_headers.py`
checks those with its parser. NOTICE ("File headers") is the rule:

- Written here: `ament_copyright --add-missing "Hyunho Cho" apache2 <files>` adds it;
  for a CMakeLists.txt, `tools/ci/check_copyright_headers.py --add-missing "Hyunho Cho" apache2 <path>`.
- Adapted from upstream: the upstream copyright line(s), then `Copyright <year> Hyunho Cho`,
  the upstream license in full, and a `Derived from ... see NOTICE` line (the TSID
  files in `cho_controller_common` are the pattern). Add a NOTICE entry.
- A package whose files carry more than one license lists each in `package.xml`.
- Never edit `extern/`. To build on a vendor's package, add it there as a submodule
  and include or reference it (as `bota_ft_sensor` does with Bota's example) rather
  than copying its files.

## Build Alias
we set alias related to build below.
you can use this alias when you need to build some packages or entire packages.
alias cbr='MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install'
alias cbp='MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select'

Do not raise these limits. This machine has 6 GB of RAM, and 4 workers at
`-j4` puts up to 16 concurrent `g++` processes on the Cho controller
packages, which exhausts memory and locks the machine up hard enough to
need a reboot. The `-l2` load limit matters as much as the job count.
Prefer scoping the package set (`--packages-select`, `--packages-above`)
to keep a wide rebuild short instead of adding parallelism.
