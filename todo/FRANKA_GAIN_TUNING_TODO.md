# Franka FR3: tune and test every controller

Written 2026-10-23 on franka-real (d342068), with the real robot out of reach. Everything that
moves the arm needs the owner's go-ahead, stage by stage. The real-robot recordings and the scratch
probes and analysis scripts referred to here are on the lab PC in `~/franka_gain_tuning`
(see its README), not in the repo.

## Where each controller stands

| mode | controller | on the real FR3 | changed since |
| --- | --- | --- | --- |
| position | `joint_space_position`, `task_space_ik` | 2026-10-22, re-checked 10-23 with robot_time and the rate limiter | - |
| velocity | `joint_space_velocity`, `task_space_velocity` | 2026-10-22 only | 90f1bd5 (robot_time, `JointRateLimiter`) |
| torque | `joint_space_impedance` | 10-22 and 10-23 | a5aeb85, da77290 (gains), 1ed5332 (switch fade) |
| torque | `task_space_qp` | 10-22 (holds, 1 cm move, goal-end fade) | 1ed5332 |
| torque | `joint_space_qp`, `task_space_impedance`, `operational_space`, `gravity_compensation` | never | 1ed5332 |
| effort | `action_chunk_controller` | 10-23 (hold, relative circle) | 2b0a2b4, fac99fa, 01593c7, 2aa7110 |
| - | `gripper_controller` | 10-23 (homing, chunk-driven moves) | - |

All of them pass `controller_check_{position,velocity,torque}` in MuJoCo on d342068.

## What the robot showed

- Joint friction sets how the torque laws move. They lag by F/kp: 2-3 mrad on j1-j4, and up to
  12 mrad on the wrist at kp 100. The wrist sticks and slips: ticks of up to 58 mrad/s on j6 and
  j7 at kp 100 / kd 5. The MJCF's frictionloss (1.137 Nm on j1-j4, 0.763/0.44/0.248 on j5-j7)
  matches the real lag. MuJoCo has no static friction above the kinetic, so it never slips.
- j2 and j4 ripple through every `joint_space_impedance` move at ~40 cycles/rad, a disturbance of
  the joints themselves that low damping amplifies. kd 15 -> 30-45 (damping ratio ~0.7 at the task
  home, with the joint inertia plus the MJCF armature) cut it by ~40 %. What is left sets the TCP's
  3-20 Hz vibration in motion: 0.04 mm rms, peaks of 0.23-0.65 mm, the same as the position-mode
  circle's. Holds stay within 0.013 mm.
- Heavy damping, not stiffness alone, holds a slip back
  (`~/franka_gain_tuning/jsi/stickslip.py`). On the robot, wrist kp 250 / kd 20 cut the wrist's
  3-20 Hz motion by 24-54 % and its end error by 3x. j7 still slips at 34-42 mrad/s.
- kd multiplies velocity noise into the torque command. At kd 20-45, `joint_space_impedance`
  changes its command by up to 175 Nm/s in a period at rest; the arm does not respond. Its D term
  takes `dq_filtered_` with alpha 0.99, which is the measurement.
- The null space of `task_space_impedance`, `operational_space` and the effort action-chunk law is
  free (`use_nullspace_posture: false`). On 10-23, j1 and j3 moved 0.19 rad on their own over one
  effort circle.
- Desk's end-effector setting changed on 10-23. The FCI's O_T_EE sat 35 mm further along the tool
  axis than on 10-22 and than the model's `fr3_hand_tcp`, at the same joints. Every torque law and
  `gravity_compensation` ride on the robot's own gravity model, so check the end effector's mass,
  CoM and TCP in Desk before any torque test.

## Plan

### 0. Without the robot

1. Add a `controller_tour` task to the task registry. It is a core task, generic across robots.
   - It takes its waypoints and segment duration from `task_params:=` and sends one JointSpace or
     TaskSpace goal per segment to the named controller.
   - It logs each goal's outcome and the error at each waypoint.
   - Joint-space controllers get the joint waypoints; task-space controllers get their TCP poses.
   - Start from the five waypoints in `~/franka_gain_tuning/tour`: W1 left, W2 far, W3 right,
     W4 high, W5 wrist tilt. The TCP never goes below 0.34 m, reaches ±0.385 m sideways and
     0.645 m forward, and each segment takes 10 s.
2. Add a recorder and a run report to `cho_control_tools`.
   - The recorder writes FrankaRobotState plus the controller's `controller_state` and `ee_state`.
   - Per run, the report gives:
     - tracking error along the path and at each waypoint;
     - 3-20 Hz velocity per joint and TCP vibration;
     - slips over 15 mrad/s;
     - the largest one-period torque change, outside switches and at each switch;
     - the command success rate.
   - The scratch versions are `day3/tremor2.py`, `day4/wrist_compare.py`, `day3/switch_detail.py`
     and `jsi/ripple.py`.
3. Derive starting gains for every controller from the model: kd for a chosen damping ratio from
   the joint inertia plus the armature, and the friction lag F/kp. Run them on the tour in MuJoCo
   with the real config, with the full friction and a fifth of it, so a set that is unstable or
   saturates is dropped before the robot.
4. Settle with the owner:
   - the null-space posture of TSI, OSC, TSQP and the action-chunk law (on or off, and its gains);
   - a velocity filter for the D terms;
   - friction feedforward (needs phase 1 step 5);
   - whether the MuJoCo, Gazebo and Isaac configs follow the real gains.

### 1. Baseline on the robot (one session)

1. Check the end effector in Desk. Clear the tour volume: ±40 cm sideways, 65 cm forward, and above
   and behind the base, where the elbow passes 0.61-0.66 m up and ±17 cm fore and aft.
2. Position (JSP, TSIK): run the tour. It is the reference for path accuracy.
3. Velocity (JSV, TSV), the first run since 90f1bd5: hold, one segment, then the tour.
4. Torque:
   - JSI and TSQP: the tour.
   - JSQP, TSI and OSC: hold, one segment, then the tour.
   - `gravity_compensation`: a 3 s hold.
   - Switch at rest between each pair, which exercises the 1ed5332 fade.
5. Friction: slow constant-velocity JSI moves per joint, both ways, for each joint's Coulomb and
   breakaway torque.
6. Run `controller_check_{position,velocity,torque}` on the robot.

### 2. Tuning (one or two sessions)

Per controller, run two or three candidate sets from phase 0 on the tour, and keep or drop each on
its report. Targets:
- no slip over 20 mrad/s;
- TCP 3-20 Hz vibration under 0.03 mm rms;
- waypoint error under 2 mm / 2 mrad for the position laws, and within the friction lag for the
  impedance laws;
- one-period torque change under 300 Nm/s outside switches;
- command success rate of 0.99 or better, and no reflex.

Make one commit per controller, with its before and after numbers, as a5aeb85 and da77290 did.

### 3. Close

1. With the tuned set, run `controller_check_*` and the guarded missions' switch to the hold
   controller on the robot.
2. Update the simulator configs if decided.
3. Merge franka-real into main (the owner's call).
