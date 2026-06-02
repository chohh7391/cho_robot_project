# UR5e Integration Work Plan

## Goal

Add UR5e support to this workspace while keeping the existing Franka stack stable.
The integration should reuse official Universal Robots packages where possible,
and add Cho-specific wrappers only where this project needs custom launch,
controllers, action servers, or task-manager routing.

This plan assumes two implementers:

- Agent A: this Codex session.
- Agent B: the second agent/user-assigned worker.

The main split should minimize merge conflicts:

- Agent A owns description, bringup, external package integration, and launch wiring.
- Agent B owns `cho_controller_ur`, action-server reuse, and task-manager robot-type routing.

## Design Decisions

1. `cho_ur_description` will be based on
   `UniversalRobots/Universal_Robots_ROS2_Description`, renamed for local customization.
   Avoid changing URDF semantics unless a Cho-specific mount, TCP, or sensor requires it.
2. `cho_ur_bringup` will include installed UR simulation launch files from apt packages.
   It should not vendor `Universal_Robots_ROS2_GZ_Simulation`.
3. Real UR support will use `UniversalRobots/Universal_Robots_ROS2_Driver` under `extern/`
   and include its launch files from `cho_ur_bringup`.
4. UR controllers should not be implemented as Franka torque-controller variants.
   Reuse the action-server contract where possible, but implement UR controllers as
   6-DOF, position-command controllers.
5. Task manager behavior must become robot-type aware because controller availability
   differs between Franka and UR5e.

## Package Names

Planned new packages:

```text
cho_robots_description/cho_ur_description
cho_robots_bringup/cho_ur_bringup
cho_controller/cho_controller_ur
```

Planned external dependency:

```text
extern/Universal_Robots_ROS2_Driver
```

APT dependencies expected for simulation:

```bash
sudo apt install ros-humble-ur-simulation-gz ros-humble-gz-ros2-control ros-humble-ur-controllers
```

APT dependency expected if not source-building the driver:

```bash
sudo apt install ros-humble-ur-robot-driver
```

For this project, real robot integration should source-build the driver under `extern/`
as requested.

## Milestone 1: Add `cho_ur_description`

Owner: Agent A

Implementation:

1. Clone the Humble branch of `Universal_Robots_ROS2_Description`.
2. Place it at:

   ```text
   cho_robots_description/cho_ur_description
   ```

3. Rename the ROS package from `ur_description` to `cho_ur_description`.
4. Update:

   ```text
   package.xml
   CMakeLists.txt
   resource/*
   launch/*
   ```

5. Replace package references:

   ```text
   package://ur_description
   ```

   with:

   ```text
   package://cho_ur_description
   ```

6. Do not hand-edit UR5e kinematics, inertials, visual meshes, or joint limits at this stage.

Suggested extra wrapper:

```text
cho_robots_description/cho_ur_description/launch/visualize_ur5e.launch.py
```

Default arguments:

```text
ur_type:=ur5e
launch_rviz:=true
```

Completion checks:

```bash
colcon build --packages-select cho_ur_description --symlink-install
source install/setup.bash
ros2 launch cho_ur_description view_ur.launch.py ur_type:=ur5e
```

Expected result:

- RViz displays UR5e.
- Mesh paths resolve.
- Frames such as `base_link`, `shoulder_link`, `wrist_3_link`, `flange`, and `tool0` exist.

Risk:

- Package renaming may break mesh paths if any `package://ur_description` reference remains.

## Milestone 2: Add `cho_ur_bringup` Scaffold

Owner: Agent A

Implementation:

Create:

```text
cho_robots_bringup/cho_ur_bringup
cho_robots_bringup/cho_ur_bringup/package.xml
cho_robots_bringup/cho_ur_bringup/CMakeLists.txt
cho_robots_bringup/cho_ur_bringup/launch
cho_robots_bringup/cho_ur_bringup/config
cho_robots_bringup/cho_ur_bringup/config/gz
cho_robots_bringup/cho_ur_bringup/config/real
cho_robots_bringup/cho_ur_bringup/utils
```

Dependencies:

```text
ament_cmake
launch
launch_ros
controller_manager
cho_ur_description
cho_controller_ur
ur_simulation_gz
ur_robot_driver
```

Use `exec_depend` for apt/source packages that may come from underlay or `extern`.

Completion checks:

```bash
colcon build --packages-select cho_ur_bringup --symlink-install
```

Risk:

- `ur_simulation_gz` may not be installed. Keep failure messages clear in launch files.

## Milestone 3: GZ Simulation Bringup Wrapper

Owner: Agent A

Implementation:

Create:

```text
cho_robots_bringup/cho_ur_bringup/launch/bringup_gz_robot.launch.py
```

The launch file should:

1. Declare arguments:

   ```text
   ur_type
   description_file
   controller_name
   launch_rviz
   use_sim_time
   world_file
   ee_name
   ```

2. Find the installed package:

   ```python
   get_package_share_directory('ur_simulation_gz')
   ```

3. Include:

   ```text
   ur_simulation_gz/launch/ur_sim_control.launch.py
   ```

4. Pass `description_file` pointing to the Cho UR description entry point.
5. Spawn Cho controllers after the base simulation starts.
6. Keep official UR controllers and Cho controllers in separate YAML files.

Planned config:

```text
cho_robots_bringup/cho_ur_bringup/config/gz/controllers.yaml
```

Minimum controller set:

```yaml
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_space_controller:
      type: cho_controller_ur/JointSpaceController
    task_space_ik_controller:
      type: cho_controller_ur/TaskSpaceIKController
```

Completion checks:

```bash
ros2 launch cho_ur_bringup bringup_gz_robot.launch.py controller_name:=joint_space_controller
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Risk:

- The included UR simulation launch may already spawn a controller manager and default controllers.
  The Cho wrapper must not start a second controller manager for the same robot.

## Milestone 4: Add `cho_controller_ur` Scaffold

Owner: Agent B

Implementation:

Create:

```text
cho_controller/cho_controller_ur
cho_controller/cho_controller_ur/package.xml
cho_controller/cho_controller_ur/CMakeLists.txt
cho_controller/cho_controller_ur/cho_controller_ur.xml
cho_controller/cho_controller_ur/include/cho_controller_ur
cho_controller/cho_controller_ur/src
cho_controller/cho_controller_ur/src/servers
```

Use `cho_controller_franka` as a structural reference, but do not copy Franka-specific
hardware assumptions into UR code.

Initial classes:

```text
URBaseController
JointSpaceController
TaskSpaceIKController
```

The action-server code can be copied or moved later into `cho_controller_common`.
For the first implementation, copying the minimal server files is acceptable to reduce
cross-package refactoring.

Completion checks:

```bash
colcon build --packages-select cho_controller_ur --symlink-install
```

Risk:

- Existing Franka code uses fixed 7-DOF vector types in several places.
  UR code must use dynamic-size Eigen vectors or UR-specific 6-DOF aliases.

## Milestone 5: Implement UR Joint-Space Position Controller

Owner: Agent B

Implementation:

Controller name:

```text
joint_space_controller
```

Plugin type:

```text
cho_controller_ur/JointSpaceController
```

Requirements:

1. Use 6 DOF.
2. Command interface:

   ```text
   position
   ```

3. State interfaces:

   ```text
   position
   velocity
   ```

4. Do not assume joint names like `ur5e_joint1`.
5. Prefer a `joints` parameter:

   ```yaml
   joints:
     - shoulder_pan_joint
     - shoulder_lift_joint
     - elbow_joint
     - wrist_1_joint
     - wrist_2_joint
     - wrist_3_joint
   ```

6. Reuse `cho_interfaces/action/JointSpace.action`.
7. Action server name:

   ```text
   /controller_action_server/joint_space_controller
   ```

8. Interpolate from current joint position to target joint position over goal duration.
9. Clip commands using UR joint limits if available from parameters.

Completion checks:

```bash
ros2 action list | grep joint_space_controller
ros2 action send_goal /controller_action_server/joint_space_controller cho_interfaces/action/JointSpace ...
```

Risk:

- If the official UR simulation controller manager already owns command interfaces,
  Cho controller activation may fail. The bringup must deactivate conflicting controllers.

## Milestone 6: Implement UR Task-Space IK Position Controller

Owner: Agent B

Implementation:

Controller name:

```text
task_space_ik_controller
```

Plugin type:

```text
cho_controller_ur/TaskSpaceIKController
```

Requirements:

1. Use Pinocchio from `robot_description`.
2. Default `ee_name`:

   ```text
   tool0
   ```

3. Accept `ee_name` override:

   ```text
   flange
   custom_tcp
   ```

4. Use DLS IK:

   ```text
   dq = J^T (J J^T + lambda I)^-1 v_task
   q_cmd = q + dq * dt
   ```

5. Use position command interfaces only.
6. Reuse `cho_interfaces/action/TaskSpace.action`.
7. Action server name:

   ```text
   /controller_action_server/task_space_ik_controller
   ```

8. Remove 7-DOF nullspace assumptions from the Franka implementation.

Completion checks:

```bash
ros2 action list | grep task_space_ik_controller
ros2 action send_goal /controller_action_server/task_space_ik_controller cho_interfaces/action/TaskSpace ...
```

Risk:

- A 6-DOF arm has no redundant nullspace for arbitrary posture control.
  IK failure handling and velocity limiting matter more than in Franka 7-DOF.

## Milestone 7: GZ Sim Controller Integration Test

Owner: Agent A primary, Agent B support

Implementation:

1. Build:

   ```bash
   colcon build --packages-select cho_ur_description cho_controller_ur cho_ur_bringup --symlink-install
   ```

2. Launch:

   ```bash
   ros2 launch cho_ur_bringup bringup_gz_robot.launch.py controller_name:=joint_space_controller
   ```

3. Verify:

   ```bash
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ros2 action list
   ros2 topic echo /joint_states --once
   ```

4. Send a small joint-space action.
5. Send a small task-space action.

Completion criteria:

- Robot appears in GZ/RViz.
- Cho controller activates.
- Action server accepts goals.
- Joint motion is visible and bounded.
- No command-interface conflict remains.

Risk:

- Simulation package behavior may differ depending on installed UR package version.
  If include launch arguments differ, inspect the installed launch file and adapt wrapper args.

## Milestone 8: Make Task Manager Robot-Type Aware

Owner: Agent B

Implementation:

Add launch argument:

```text
robot_type
```

to:

```text
cho_task_manager/launch/run_task_manager.launch.py
```

Add robot config files:

```text
cho_task_manager/config/robots/franka.yaml
cho_task_manager/config/robots/ur5e.yaml
```

Suggested `ur5e.yaml`:

```yaml
robot_type: ur5e
controllers:
  joint_space: joint_space_controller
  task_space: task_space_ik_controller
  gripper: null
  vla: null
actions:
  joint_space: /controller_action_server/joint_space_controller
  task_space: /controller_action_server/task_space_ik_controller
```

Suggested `franka.yaml` maps existing names:

```yaml
robot_type: franka
controllers:
  joint_space: joint_space_qp_controller
  task_space: task_space_qp_controller
  gripper: gripper_controller
  vla: vla_controller
```

Refactor:

```text
cho_task_manager/cho_task_manager/utils/controller_names.py
```

from fixed enum-only mapping to config-backed mapping.

Task split:

- Keep existing Franka task behavior unchanged by default.
- Add UR5e task variants only where joint targets or poses differ.

Possible layout:

```text
cho_task_manager/cho_task_manager/tasks/franka
cho_task_manager/cho_task_manager/tasks/ur5e
```

Completion checks:

```bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_and_place robot_type:=franka
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_and_place robot_type:=ur5e
```

Risk:

- Existing behaviors assert valid controller names from the enum.
  That assertion must be updated before UR controller names can pass.

## Milestone 9: Add UR Driver Under `extern`

Owner: Agent A

Implementation:

Clone:

```bash
cd extern
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
```

If the repository has submodules or source dependencies, document the needed `vcs import`
or `rosdep` commands in `docs/installation.md` or a new UR setup doc.

Expected package:

```text
ur_robot_driver
ur_controllers
ur_calibration
ur_dashboard_msgs
```

Completion checks:

```bash
colcon build --packages-select ur_robot_driver ur_controllers ur_calibration --symlink-install
```

Risk:

- Source driver may conflict with apt-installed UR packages.
  Prefer one source of truth in the active workspace/underlay.

## Milestone 10: Real Robot Bringup Wrapper

Owner: Agent A

Implementation:

Create:

```text
cho_robots_bringup/cho_ur_bringup/launch/bringup_real_robot.launch.py
```

Launch arguments:

```text
ur_type:=ur5e
robot_ip
kinematics_params_file
controller_name:=joint_space_controller
launch_rviz:=false
use_tool_communication:=false
ee_name:=tool0
```

The launch file should:

1. Include:

   ```text
   ur_robot_driver/launch/ur_control.launch.py
   ```

2. Pass `ur_type`, `robot_ip`, and `kinematics_params_file`.
3. Disable unnecessary RViz by default.
4. Spawn Cho controllers after the UR driver controller manager is available.
5. Deactivate conflicting official command controllers before activating Cho controllers.

Calibration file location:

```text
cho_robots_bringup/cho_ur_bringup/config/real/ur5e_calibration.yaml
```

Do not commit lab-specific calibration or IPs unless repository policy explicitly allows it.

Completion checks:

```bash
ros2 launch cho_ur_bringup bringup_real_robot.launch.py robot_ip:=<ROBOT_IP> kinematics_params_file:=<CALIBRATION_YAML>
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Risk:

- Real UR motion requires External Control URCap and correct robot-side program setup.
- Calibration mismatch can produce large TCP pose error.

## Milestone 11: Real Robot Small-Motion Validation

Owner: Agent A primary, Agent B support

Implementation:

Validation order:

1. Start `ur_robot_driver` alone.
2. Confirm dashboard and RTDE connection.
3. Confirm calibration hash or warning state.
4. Confirm `/joint_states`.
5. Move with the official UR trajectory controller using a very small command.
6. Start Cho `joint_space_controller`.
7. Send a small Cho joint-space action.
8. Start Cho `task_space_ik_controller`.
9. Send a small Cartesian translation, preferably less than 1 cm at first.
10. Verify controller switching and task_manager routing.

Do not start with task-space IK on real hardware before joint-space validation passes.

Completion criteria:

- Real robot connects.
- Cho joint-space controller can command a bounded small motion.
- Cho task-space IK controller can command a bounded small motion.
- Task manager can select UR5e controller/action names via `robot_type:=ur5e`.

Risk:

- UR real hardware position control and controller switching behavior must be treated as safety-critical.
  Keep default motions tiny and require explicit launch arguments for real robot IP/calibration.

## Parallel Work Plan

### Agent A Track

Primary files:

```text
cho_robots_description/cho_ur_description
cho_robots_bringup/cho_ur_bringup
extern/Universal_Robots_ROS2_Driver
docs
```

Tasks:

1. Milestone 1: description clone/rename/visualize.
2. Milestone 2: bringup scaffold.
3. Milestone 3: GZ simulation include wrapper.
4. Milestone 7: simulation integration test.
5. Milestone 9: driver clone and source dependency notes.
6. Milestone 10: real bringup wrapper.
7. Milestone 11: real robot validation procedure.

Agent A should not modify `cho_controller_franka` except for shared utility extraction that is
explicitly coordinated with Agent B.

### Agent B Track

Primary files:

```text
cho_controller/cho_controller_ur
cho_task_manager
cho_interfaces only if action definitions must change
```

Tasks:

1. Milestone 4: controller package scaffold.
2. Milestone 5: joint-space position controller.
3. Milestone 6: task-space IK position controller.
4. Milestone 8: task-manager robot-type routing.
5. Support Milestone 7 and 11 by debugging controller activation/action behavior.

Agent B should avoid editing launch wrappers except to request controller YAML parameters.

## Shared Contracts Between Agents

### Controller Names

Use these names for the first UR implementation:

```text
joint_space_controller
task_space_ik_controller
```

### Action Names

Use:

```text
/controller_action_server/joint_space_controller
/controller_action_server/task_space_ik_controller
```

### Joint Names

Use official UR joint names unless `tf_prefix` is enabled:

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

Pass these as YAML parameters instead of hardcoding generated names.

### End-Effector Name

Default:

```text
tool0
```

Allow override:

```text
ee_name:=flange
```

### Control Mode

UR5e first implementation:

```text
position
```

Do not implement real-hardware torque/effort impedance controllers in the first pass.

## Suggested Branching

If using git branches:

```text
feature/ur5e-description-bringup
feature/ur5e-controller-task-manager
```

Merge order:

1. `feature/ur5e-description-bringup`
2. `feature/ur5e-controller-task-manager`
3. integration fixes

## Open Questions

1. Should `cho_ur_description` keep all UR robot variants or only UR5e?
   Keeping all variants is simpler and closer to upstream; UR5e-only is lighter but creates more
   divergence from upstream.
2. Should shared action-server code be moved to `cho_controller_common` immediately?
   Recommended answer: no for the first pass. Copy minimal code into `cho_controller_ur`, then
   refactor after UR simulation works.
3. Should UR task poses be separate task files or config-driven variants?
   Recommended answer: use robot-specific task files first if poses differ significantly.
4. Should `tf_prefix` be supported initially?
   Recommended answer: no unless multi-robot UR support is needed now.

