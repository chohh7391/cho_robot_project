# Controller Stability — Remaining TODO

Follow-up items from the controller stability review (2026-07). The safety-critical
and low-risk fixes were applied and pushed (commits "Controller stability fixes
(batch 1/2/3)"). The items below were **deliberately deferred** because they either
require a threading redesign or change real-robot control behavior and therefore need
hardware validation. Do NOT apply blind — test on the real FR3.

Line numbers are approximate (as of batch 3) — re-locate before editing.

---

## HIGH — action-server ↔ RT concurrency (redesign, hardware validation)

Root cause: `goal_handle_` is a plain (non-atomic) `std::shared_ptr` shared between the
1 kHz RT `compute()` thread and the ROS executor threads, and `compute()` calls
`rclcpp_action` methods every cycle.

- `servers/base_action_server.hpp:95` — `goal_handle_` shared between threads.
- RT `compute()` calls `goal_handle_.reset()` / `is_active()` / `succeed()` / `abort()`
  every cycle (task/joint/gripper action servers). These take internal mutexes,
  allocate, and serialize messages → priority inversion / cycle overrun; concurrent
  copy+reset of the same shared_ptr from two threads is a data race (refcount
  corruption / use-after-free); `reset()` can run the goal-handle destructor in the RT
  thread.

**Fix direction:** RT `compute()` should only (a) read a lock-free command snapshot
(`realtime_tools::RealtimeBuffer` / atomics) and (b) set an atomic status/result code.
A non-RT context (a timer or the executor) owns the `goal_handle_` lifecycle and calls
`succeed()/abort()/reset()`. No `rclcpp_action` calls from the RT thread.

**RESOLVED for ALL four action servers (2026-07-17):**
The atomic goal-phase state machine now lives in `BaseActionServer` (kIdle → kActive →
kFinish\* → kIdle): the executor stages the goal payload and calls `activate_goal()`,
the RT `compute()` resets per-goal state on epoch change (`rt_new_goal_epoch()`) and
only flips the phase atomically on cancel/success/timeout (`finish_from_rt()`), and a
5 ms non-RT finisher timer performs the actual `succeed()/abort()/canceled()` calls
and handle release (`on_goal_finished()` hook for per-server follow-ups, e.g. VLA's
BT notification). `goal_handle_` was removed from the base class entirely; no server's
RT path touches rclcpp_action or logs per-cycle. Verified in MuJoCo: VLA (four command
representations, cancel, success trigger, re-acceptance), joint+gripper (full
pick_place_position behavior-tree run), task-space (succeed status=4, mid-motion
cancel status=5, immediate re-accept).
**Known trait (pre-existing, unchanged):** an action server accepts goals even while
its controller is INACTIVE; compute() never runs, so the goal hangs until cancelled.
The behavior trees always switch the controller active before sending goals, which
avoids this — but direct CLI/action clients should do the same.

## HIGH/MED — VLA action-chunk buffering (concurrency) — RESOLVED (2026-07-17)

- ~~`process_vla_action` calling `vla_cmd_buffer_.readFromRT()` from non-RT~~ →
  replaced with an executor-owned shadow copy of the last written command.
- ~~`chunk_size_` written non-RT / read RT as a divisor~~ → the RT side now reads a
  precomputed `waypoint_dt` carried inside the buffered `VLACommand`; `chunk_size_` is
  executor-only and `chunk_size <= 0` messages are rejected.
- ~~`inference_dt_` unvalidated~~ → `handle_goal` rejects non-finite / non-positive
  `inference_frequency`.
- ~~"waiting for first chunk" branch skips cancel/success/timeout → wedge~~ → terminal
  checks moved before target processing; goal timeout anchored to goal start.
- Two non-RT writers to `vla_cmd_buffer_` — benign: `writeFromNonRT` serializes
  writers on its internal mutex, and both callbacks live in the node's mutually
  exclusive callback group.

---

## MED — behavior-changing (hardware validation)

- **Null-space posture in ACTION mode** — `task_space_qp switch_to_action_control()`
  (~:207) removes `task-posture`, leaving only the 6-DoF SE3 task on the 7-DoF arm.
  The elbow (redundant DoF) can drift toward a joint limit over long motions. Add a
  low-weight posture task in the null space.
- **IK absolute joint-limit clamp** — `task_space_ik_controller` integrates `q_ref_`
  open-loop (~:140) with no clamp to the model's position limits. Clamp `q_ref_` to
  `model.lowerPositionLimit / upperPositionLimit`. (The per-tick slew `max_delta_q` was
  already tightened to 0.0025 rad in batch 1.)

## LOW — behavior-changing / tuning (hardware validation)

- **Torque rate limiter** — no dτ/dt limit anywhere; the FR3 enforces ~1000 Nm/s.
  A mode switch or the QP-fail→zero-accel step can produce a torque discontinuity that
  trips the rate reflex. Add a per-cycle Δτ clamp (parameterized) in `clip_torque` or
  after it. Needs a stored previous torque + tuning.
- **joint_trajectory_controller gains underdamped** — e.g. real config
  `kd=[15,15,15,15,2,2,1]` vs `kp=[400,500,500,400,100,100,30]` (ζ≈0.1 on the wrist).
  Acceptable with feedforward, but re-tune `kd` upward on hardware if oscillation shows.
- **Minor RT preallocations** (marginal; TSID returns by value anyway) —
  `task_space_qp:176` `VectorXd ddq = getAccelerations()`, `:234`
  `TrajectorySample sample_posture_full(model_na)`; `task_space_ik:102`
  `Eigen::VectorXd q_full = state_.q`. Make members to drop per-cycle construction.

---

## Deliberately NOT changing (owner decision)

- `M_modified` wrist inertia inflation (`task_space_qp:126-129`,
  `joint_space_qp:114-118`) — intentional practical compensation.
- `kAlpha = 0.99` velocity blend — keep as is.
- gazebo hand-gravity `= -1.0 m/s²` (`base_controller.cpp:270`) — intentional; matches
  the gazebo setup.
- Payload/hand gravity NOT added to the QP torque on real — Desk compensates it.
