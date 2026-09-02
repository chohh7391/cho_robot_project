# FR5 Integration — Handover

Handover for continuing the FAIRINO FR5 integration into `cho_robot_project`.
Read this top to bottom; it assumes no prior context.

---

## 1. Project context (how this repo is organized)

`cho_robot_project` is a ROS 2 (Humble) workspace of custom `ros2_control` controllers
for several robot arms. Each robot is a **vertical stack of parallel packages**:

- `cho_description/cho_description_<robot>` — URDF/xacro, meshes, MuJoCo XML, USD (Isaac).
- `cho_controller/cho_controller_<robot>` — the `ros2_control` plugin controllers
  (namespace `cho_controller::<robot>`), each inheriting a `<Robot>BaseController`.
- `cho_bringup/cho_bringup_<robot>` — launch files + per-env `controllers.yaml`
  (mujoco / gz / isaac / real).
- `cho_robot_config/config/<robot>.yaml` — controller-role single source of truth.
- `cho_bringup/cho_bringup_isaac/isaac/robots/<robot>.json` — Isaac physics profile.

Existing robots: **franka** (torque-controlled, the most mature — QP/impedance/IK),
**ur** (position-controlled, 6-DOF), **openarm** (sim only). FR5 was added by
**mirroring the UR vertical** (position control, 6-DOF), then upgraded to the
**Franka control patterns**.

Shared code lives in `cho_controller/cho_controller_common` (Pinocchio FK/IK/dynamics,
the `TrajectoryEuclidianCubic` / `TrajectorySE3Cubic` used by all action servers).

Build alias (`~/.bashrc`): `cbp` = `colcon build ... --packages-select`.
Always `source ~/ros2_ws/install/setup.bash` after a build.

The FR5 task and plan: `todo/FR5_TODO.md`. Auto-memory with the running state:
`~/.claude/projects/-home-home-ros2-ws-src-cho-robot-project/memory/fr5-integration.md`.

---

## 2. The FR5 task

Add the FAIRINO FR5 (6-DOF, joints `j1..j6`, EE frame `wrist3_link`) as a new
vertical, mirroring UR. Support **all four bringups**: gazebo, mujoco, isaac, real.
Test in sim; the user tests real hardware and gives feedback.

Vendor / asset sources on this machine:
- `~/Downloads/frcobot_ros2/` — vendor ROS2 driver (`fairino_hardware_v3_9_9`,
  `fairino_msgs`, libfairino C++ SDK, `fairino_description`, moveit configs).
- `~/Downloads/fr5_position_control_bundle/` — FR5 URDF + STL meshes +
  MuJoCo `fr5_p.xml` + DLS `kinematics/solvers.py` (reference only).

---

## 3. What was created / changed

New packages (all build clean, `--symlink-install`):
- `cho_description/cho_description_fr5/` — `urdf/fr5.urdf.xacro` (single entry point;
  `hardware` arg switches **mujoco | isaac | gazebo | mock | fairino** in
  `fr5.ros2_control.xacro`), `fr5_macro.xacro` (arm), meshes, `xml/{scene,fr5}.xml`
  (MuJoCo), `config/initial_positions.yaml`.
- `cho_controller/cho_controller_fr5/` — ns `cho_controller::fr5`. `FR5BaseController`
  + `JointSpacePositionController` + `TaskSpaceIKController` + action servers. **No
  gripper.**
- `cho_bringup/cho_bringup_fr5/` — `bringup_{mujoco,gz,isaac,real}_robot.launch.py`
  + `config/{mujoco,isaac,gz,real}/controllers.yaml` + `config/real/fr5.config.yaml`.

Vendored (copied into `extern/`, patched — see §6):
- `extern/fairino_hardware_v3_9_9/` (+ `CHO_PATCHES.md`) and `extern/fairino_msgs/`.

Added to existing packages:
- `cho_robot_config/config/fr5.yaml`.
- `cho_bringup/cho_bringup_isaac/isaac/robots/fr5.json` (Isaac physics profile).
- `cho_control_tools` generic action client — added `fr5` robot type (home/reach poses).

Modified existing (explicitly requested):
- `cho_interfaces/CMakeLists.txt` + deleted `cho_interfaces/msg/JointLog.msg` — FR5 was
  its last user after the logging migration (below). **NB: after deleting a rosidl
  .msg you MUST clean-rebuild:** `rm -rf build/cho_interfaces install/cho_interfaces &&
  colcon build --packages-select cho_interfaces` — an incremental build leaves a
  dangling `..._joint_log__convert_from_py` symbol that breaks every `cho_interfaces`
  Python import.

Logging convention (matches the repo's ff71392 migration): controllers publish
`~/controller_state` (`control_msgs/JointTrajectoryControllerState`) and `~/ee_state`
(`cho_interfaces/PoseLog`). The old global `/log/joint_pos` + `/log/ee_pose` are gone.

---

## 4. Control architecture — THE key technical content

FR5 is **position-controlled**: controllers write joint position commands; the plant
(MuJoCo servo, or the real FR5's `ServoJ` stream) tracks them.

Two controllers, each wraps an action server that runs a cubic trajectory from
`cho_controller_common`:
- `JointSpacePositionController` ← `cho_interfaces/action/JointSpace`
  (`/controller_action_server/joint_space_position_controller`).
- `TaskSpaceIKController` (DLS differential IK) ← `cho_interfaces/action/TaskSpace`
  (`/controller_action_server/task_space_ik_controller`).

### The critical pattern: OPEN-LOOP IK (ported from Franka)

The UR clone originally commanded `q_cmd = q_measured + delta_q` (closed-loop on the
measured state). That is wrong when the state estimate / servo isn't exact: it feeds
observer noise + servo droop into the command → **drift and "sudden fast" bursts**,
especially near singularities. The **Franka** controllers
(`cho_controller_franka/src/{task_space_ik,joint_space_position}_controller.cpp`) solve
this with an **open-loop reference**, and FR5 now mirrors them:

`TaskSpaceIKController::update()`:
1. Maintain `q_ref_` — an **open-loop** joint reference, never rebuilt from `q_measured`.
2. Solve FK + local Jacobian **at `q_ref_`** (not measured) via
   `FR5BaseController::compute_arm_kinematics()`.
3. Sample the trajectory on a **jitter-free clock**: `traj_clock_ += nominal_period(period)`
   (NOT the jittery measured ROS time).
4. Local-frame DLS Newton step; per-joint clamp `dq` to `±max_delta_q`; `q_ref_ += dq`;
   `clamp_to_joint_limits(q_ref_)`.
5. **Freeze `q_ref_` when idle** (no goal) — no re-solving toward a measured hold pose.
6. Command `q_ref_` directly.
7. Seed `q_ref_ = held_command_position()` in `on_activate` (the value the previous
   controller left on the shared command interface, not measured), and re-seed the
   trajectory at `FK(q_ref_)` on the first running cycle (`if (!prev_running_)`).

The joint controller was fixed more simply (the action server seeds the trajectory from
the last **command** `q_ref`, not the drooped measured position) and the user confirmed
joint control is good. **Optional future work:** give the joint controller the same
jitter-free `nominal_period()` clock as Franka for the real 125 Hz `ServoJ` stream.

Four helpers were ported into `FR5BaseController` (`base_controller.{hpp,cpp}`):
`compute_arm_kinematics`, `nominal_period` (jitter-free dt; also fixes the Isaac 4x-slow
trajectory bug — see comment), `clamp_to_joint_limits`, `held_command_position`.

### Verified result (MuJoCo, measured with a probe)
- Joint goals: smooth, `‖err‖ ≈ 0.017 rad`, no start/end lurch.
- Task `reach`: smooth **0.2 rad/s** (was 11.5 rad/s), holds **6.7 mm steady, no drift**
  (was 7.5→15 mm creep), succeeds. Controller switching works. `action_client fr5` works.
- **GoalStatus 4 == STATUS_SUCCEEDED** (not aborted — easy to misread; 6 is aborted).

---

## 5. Gotchas already hit and solved (don't re-discover these)

- **MuJoCo model tuning** (`cho_description_fr5/xml/fr5.xml`): the raw bundle `fr5_p.xml`
  oscillated/jammed. Fixes: joint `armature=0.1`+`damping`, actuator `kv` + higher `kp`
  (3000/1000), **removed the joints' `actuatorfrcrange`** (it capped servo force so the
  arm saturated at ~0.19 rad and ran away), and **all geoms `contype/conaffinity=0`**
  (the bundle reused visual meshes as collision → self-collision jammed joints). Also
  **named the actuators** (an unnamed MuJoCo actuator makes `mj_id2name` return null →
  the mujoco_ros2_control plugin segfaults in `register_urdf_joints`).
- **Canonical simulation ready pose (updated 2026-09-02)** is
  `[0, -pi/4, -pi/2, pi/4, -pi/2, 0] rad` (`[0, -45, -90, 45, -90, 0] deg`).
  Gazebo reads it from `initial_positions.yaml`; MuJoCo direct and MoveIt bringups both
  select the `home1` keyframe by default; the Isaac `arm_home` file is aligned but Isaac
  has not been run. The same pose is action-client `home 1` and MoveIt SRDF `home1`.
  Explicit all-zero remains recorded as `home 0`, but normal action-client and MoveIt
  execution reject it because the wrist is at the floor. It is available only as the
  diagnostic MuJoCo keyframe `zero` (`mujoco_initial_keyframe:=zero`) or through a
  deliberately issued raw direct-controller goal. Zero is a wrist singularity (`j5=0`)
  and is not the default simulation spawn anymore. Real bringup still reads the physical
  robot state and never auto-moves to the ready pose.
- **Ready-pose validation evidence (2026-09-02):** Pinocchio FK placed `wrist3_link` at
  `z=0.731834 m`; Jacobian `sigma_min=0.085573`, condition number `21.106` (previous
  home1: `0.086316`/`20.418`; zero: `1.54e-17`/singular). An actual MoveIt
  `/check_state_validity` request after applying the 4 x 4 x 0.1 m floor at z=-0.05
  returned `valid=true`, no contacts (self + floor collision model). Static registry
  tests separately enforce joint order/limits, cross-file consistency, FK height, and
  Jacobian conditioning; they do not substitute for the MoveIt collision runtime gate.
- **`lambda=0.02`, `max_delta_q` ≈ 2.5 rad/s cap** in every `controllers.yaml`
  (real is 0.005 @125 Hz = 0.625 rad/s, conservative). The open-loop `dq` clamp is the
  hard joint-velocity limit.
- **Process hygiene when testing MuJoCo:** each `ros2 launch` starts a MuJoCo GUI node
  that can be hard to kill (GPU/D-state) and orphans to a subreaper. Two `/clock`
  publishers make sim time jump backwards ("Moved backwards in time") and stall
  activation. Use a fresh `ROS_DOMAIN_ID` per test and kill by matching the *executable*
  (`ps -eo pid,args | awk '$2 ~ /ros2_control_node$/'`), not `pgrep -f` (which matches
  your own shell). Verify `ps -eo args | awk '$1 ~ /ros2_control_node$/'` count is 0.

---

## 6. Real-robot integration (vendor HW) — implemented, UNTESTED (needs hardware)

`extern/fairino_hardware_v3_9_9` is the vendor `hardware_interface::SystemInterface`
(`fairino_hardware/FairinoHardwareInterface`, libfairino SDK). `write()` streams
`ServoJ(...,cmdT=0.008,...)` at 125 Hz. Patches applied (all marked `// cho patch`,
see `CHO_PATCHES.md`), in `src/fairino_hardware_interface.cpp`:
- **robot_ip param**: `on_init` reads `robot_ip` from the ros2_control `<param>`
  (upstream hardcodes `#define CONTROLLER_IP_ADDRESS`).
- **A2-velocity**: cho controllers require `position + velocity` state; upstream exports
  position only. `on_init` now expects 2 state interfaces; `export_state_interfaces`
  exports velocity; `read()` fills it from `GetActualJointSpeedsDegree(1, ...)`.

Real bringup: `config/real/fr5.config.yaml` (set `robot_ip`, default 192.168.58.2),
`controllers.yaml` `update_rate: 125` (matches ServoJ cmdT 8 ms). Launch composes
rsp + `controller_manager/ros2_control_node` + spawners directly (no vendor
control.launch). Safety: bring up `joint_state_broadcaster` only first (state read-back),
then joint low-speed, then task. `on_deactivate` calls the vendor `StopMotion()`.
The cho controllers own velocity/Δq limits (vendor `write()` does not clamp).

Note: the vendor packaging is a **copy into `extern/`**, not a git submodule fork
(the plan `todo/FR5_TODO.md §8.A` preferred a fork). Convert later if you want the
patches tracked upstream.

---

## 7. Isaac — implemented, UNTESTED (needs a one-time USD build + Isaac Sim)

`bringup_isaac_robot.launch.py` + `isaac/robots/fr5.json` (profile: joints, `arm_home`
= canonical ready/home1, armature 0.1, PD gains from the MuJoCo `kp`, solver iters).
The file is aligned with the other simulation backends, but this revised pose has not
yet been executed in Isaac. Uses
`topic_based_ros2_control/TopicBasedSystem` (the isaac branch of the xacro).
**Must build the USD once** before first run (the launch prints the exact command and
exits if missing): `~/isaacsim/python.sh <cho_bringup_isaac share>/isaac/convert_urdf_to_usd.py
--urdf <fr5.urdf.xacro> --usd-path <cho_description_fr5 share>/usd/fr5 --ros-package
cho_description_fr5:<share>`. `physics_rate` (default 250) MUST equal
`controller_manager.update_rate` in `config/isaac/controllers.yaml`. Give Isaac its own
`ROS_DOMAIN_ID` if anything else simulates.

## Gazebo — implemented, UNTESTED
`bringup_gz_robot.launch.py`, xacro `hardware:=gazebo` → `ign_ros2_control/IgnitionSystem`
+ `libign_ros2_control-system.so` (both confirmed present on this machine). No gripper.

---

## 8. Current status summary

| Backend | State | Notes |
|---|---|---|
| **MuJoCo** | ✅ verified end-to-end | joint + task both smooth/steady/succeed; switching; action_client `fr5` |
| **Gazebo** | implemented, not run | prereqs present |
| **Isaac**  | implemented, not run | needs one-time USD build + Isaac install |
| **Real**   | implemented + vendor patched, not run | needs the physical FR5 |

Everything builds clean:
`cbp fairino_msgs fairino_hardware_v3_9_9 cho_controller_fr5 cho_description_fr5 cho_bringup_fr5 cho_interfaces`
(after the cho_interfaces clean-rebuild note in §3).

---

## 9. How to run / test (MuJoCo — the verified path)

```bash
cd ~/ros2_ws && source install/setup.bash
# terminal 1 — spawns at the canonical non-singular ready/home1 pose
ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py controller_name:=joint_space_position_controller
# terminal 2 — joint control
ros2 run cho_control_tools debug_action_client --robot_type fr5 --control_space joint
#   (csuite) home 1     # optional: re-command the already active ready pose
# then switch to task space:
ros2 control switch_controllers --activate task_space_ik_controller --deactivate joint_space_position_controller
ros2 run cho_control_tools debug_action_client --robot_type fr5 --control_space task
#   (csuite) reach 0|1|2|3      # small relative moves; smooth + succeeds
```
`home N` needs `joint_space_position_controller` active; `reach N` needs
`task_space_ik_controller` active (the WARN about the inactive one is normal).
Direct one-liner also works: `ros2 action send_goal /controller_action_server/joint_space_position_controller cho_interfaces/action/JointSpace "{target_joints: {position: [...]}, duration: 4.0}"`.

To reproduce the legacy singular zero spawn for diagnostics, add
`mujoco_initial_keyframe:=zero`. The action-client `home 0` command is intentionally
disabled; use a deliberately issued raw direct-controller goal to leave zero, and move
back to `home 1` before direct task-space control.

---

## 10. Suggested next steps

1. Run Gazebo and Isaac in sim; tune `fr5.json` PD gains / MuJoCo `kp,kv` from behavior.
2. **FK validation gate** (`todo/FR5_TODO.md` Phase 0): compare cho URDF FK vs the robot's
   `GetActualTCPPose`/`GetForwardKin` before real motion.
3. Real bringup, incrementally (state-only → joint low-speed → task), user-driven.
4. Optional: give `JointSpacePositionController` the Franka jitter-free `nominal_period()`
   clock for the real 125 Hz stream; consider converting `extern/fairino_*` to a submodule fork.
5. Docs: add FR5 launch commands to `CLAUDE.md`; add the new packages to CI.

Reference implementations to imitate: `cho_controller_franka/src/task_space_ik_controller.cpp`
and `joint_space_position_controller.cpp` (the open-loop pattern, fully commented).
