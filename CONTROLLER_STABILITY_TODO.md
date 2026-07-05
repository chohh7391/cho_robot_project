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
  every cycle (task/joint/gripper/vla action servers). These take internal mutexes,
  allocate, and serialize messages → priority inversion / cycle overrun; concurrent
  copy+reset of the same shared_ptr from two threads is a data race (refcount
  corruption / use-after-free); `reset()` can run the goal-handle destructor in the RT
  thread.

**Fix direction:** RT `compute()` should only (a) read a lock-free command snapshot
(`realtime_tools::RealtimeBuffer` / atomics) and (b) set an atomic status/result code.
A non-RT context (a timer or the executor) owns the `goal_handle_` lifecycle and calls
`succeed()/abort()/reset()`. No `rclcpp_action` calls from the RT thread.

## HIGH/MED — VLA action-chunk buffering (concurrency)

- `servers/vla_action_server.cpp:232` — `process_vla_action` (non-RT subscription cb)
  calls `vla_cmd_buffer_.readFromRT()`, violating `RealtimeBuffer`'s single-RT-reader
  contract vs RT `compute()` `readFromRT()` (:81) → torn/garbage pose to the RT loop.
- `vla_action_server.hpp:77` — `chunk_size_` uninitialized `int`, written non-RT (:233)
  / read RT (:119), used as a divisor → possible divide-by-zero and unsynchronized read.
- `vla_action_server.cpp:42` — `inference_dt_ = 1 / goal->inference_frequency` with no
  validation (zero/negative/NaN → inf).
- `vla_action_server.cpp:87-91` — "waiting for first chunk" branch returns early and
  skips cancel/success/timeout handling → goal can wedge if no chunk arrives.
- Two non-RT writers to `vla_cmd_buffer_` (`:69` and `:309`) — unsafe on a
  multi-threaded executor.

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
