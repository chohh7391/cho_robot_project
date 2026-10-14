# OpenArm MIT core contract boundary

`cho_openarm_mit_core` is the dependency root for the OpenArm MIT tuple and
safety protocol.  It owns the five-field joint tuple, generation/lease/SAFE
state-machine primitives, paired ownership contract, canonical joint ordering
and claim layout (`complete_claims()`), and fail-closed safety-profile loading.

It deliberately exports no controller plugin and no `ros2_control`
`SystemInterface`.  Controllers consume this contract to decide *what* tuple to
produce; hardware adapters consume it to decide *how* a validated tuple reaches
a simulator or actuator.  This keeps the CAN adapter dependent on this
package rather than on MuJoCo or controller code. The protocol itself is
[docs/openarm_mit_contract_v1.md](../../docs/openarm_mit_contract_v1.md).

## Safety profiles

`load_safety_profile_yaml()` / `load_safety_profile_file()` (`mit_protocol.hpp`)
are how every backend gets its profile. There is no default: the file's
`default_profile` must be null, and the caller names the profile and the arm
side (`single`, `left` or `right`, which selects the mount's joint-position
window). The loader rejects missing or unknown keys, wrong scalar types and
enums, null required simulation values, a backend mismatch, and non-finite or
misordered limits. The torque `ordering`, `slew_reference` and SAFE `normal_policy` enums are
per backend: each names what that backend's consumer does. The torque section's
`safe_hold_torque_band` (seven values, `0 <= band <= tau_ff_magnitude`) and
`safe_hold_motion_velocity` (`> 0`) bound the SAFE hold (below). Of the four profiles in
`cho_description_openarm/config/mit_safety_profiles_v1.yaml`,
`real_conservative_unapproved` is always rejected; `real_conservative_commissioning`
and `real_return_to_zero_commissioning` are the only real profiles that load.
A real adapter loads its profile and validates its CAN interface before it
opens CAN or enables a motor. What the numbers are and why is in that YAML's
comments.

## SwitchGate

`SwitchGate` is the controller-switch rule every backend applies (the real
adapter, MuJoCo and the test fake), one instance per arm, so the fake-based
controller integration tests run against the same rule the real adapter
enforces:

- `prepare_command_mode_switch()`: a switch that claims only part of an arm's
  46 command interfaces (`complete_claims()`: the five fields per joint, the four
  protocol handles, the seven `hold_torque`) is refused. A switch that starts or
  stops the arm is accepted whether or not the arm is SAFE. The backend's next
  `write()` puts the arm in measured SAFE (a new `safe_generation`,
  `SAFE_TRANSITION` until submitted); an arm already in SAFE keeps its hold and
  gets no new generation. Until perform, no producer SAFE request or commit is
  evaluated.
- `perform_command_mode_switch()`: the backend discards the commit the outgoing
  producer left unacknowledged (`ArmConsumer::discard_commit`): it is never
  evaluated, and `mit_ack_generation` advances to it. Humble calls perform from
  the control loop, inside `update()` and before `write()`, and activates the
  incoming controllers right after it, so the incoming producer reads the
  advanced ack in `on_activate()` and commits above it. The next `write()`
  enters SAFE again if anything ran since prepare, then evaluates commits.
- The first write after perform is flagged (`Cycle::performed`): a SAFE request
  still pending then belongs to the outgoing producer, and the switch's own
  SAFE consumes it; a commit with a new generation can only be the incoming
  producer's and is evaluated in that same write.
- A switch the controller_manager abandons after a successful prepare never
  performs. After `expiry_cycles` `write()` cycles (the real and MuJoCo
  backends set it to the profile's `update_rate_hz`, one second; the test fake
  keeps the default 1000) the gate opens by itself, with the same discard.
- On the bimanual simulation backends, an ownership change between direct and
  paired additionally needs the old owner to have completed an acknowledged
  SAFE with aligned generations: the pair transaction needs both arms'
  generations equal, which a hardware SAFE does not provide.

The class only decides; each backend owns its SAFE tuple and commit bookkeeping
(the real adapter and the fake through `ArmConsumer`, MuJoCo through its
limiter).

## SAFE hold

`ArmConsumer`'s SAFE hold is `q_des = q_measured, dq_des = 0`, the per-joint
safe gains, and `tau_ff = tau_measured`: the joint torque measured in the same
read as the pose (`observe(position, velocity, effort)`), clamped per joint to
the hold's effort limit (the profile's `tau_ff_magnitude` on the real adapter).
A new session (`configure()`) is seeded with the torque measured at the seed
read. The `ArmConsumer` class comment in `mit_protocol.hpp` gives the reasons.

A hold latched in motion is bounded. Every commit carries the producer's
gravity estimate per joint, `hold_torque` (`ArmCommand::hold_torque`, the
`<joint>/hold_torque` command interfaces at claim indices 39-45; NaN is no
estimate; a finite `|h|` over the `tau_ff` bound, or an infinite one, rejects
the commit like an out-of-range `tau_ff`). The consumer keeps `h` from the last
accepted commit of the session (a rejected or discarded commit leaves it;
`configure()` and `cleanup()` forget it). Wherever a hold is built (SAFE
request, rejected commit, lease expiry, switch, stop; `hold_effort_now()` for
an adapter's own fallback), with `band = torque.safe_hold_torque_band` and the
threshold `torque.safe_hold_motion_velocity`:

```
in_motion = any joint of the latch read with |dq| >= threshold (non-finite dq: in motion)
tau_ff    = in_motion and h finite ? clamp(tau_measured, h - band, h + band) : tau_measured
tau_ff    = clamp(tau_ff, -effort_limit, effort_limit)
```

(`hold_in_motion()`, `bounded_hold_torque()`). Moving, the measured torque
carries the motion's inertial and contact torque, which the hold would keep
as a constant push; the band keeps only what the producer's model explains.
At rest the measured torque stays, however far from `h`: it supports a payload
the model does not know. In motion such a payload is supported only up to the
band. The real profiles' band is the joints' identified static friction
(`friction_level` of `cho_bringup_openarm`'s `config/real/controllers_mit.yaml`),
so a hold error inside it stays still only while below the breakaway friction,
and a latch finds the arm moving (the sag bound:
[Hold torque](../../docs/openarm_mit_contract_v1.md#hold-torque));
`mujoco_sim_safe`'s is a bound, not a friction. The constructors without a band
never bound (band and threshold `+inf`). `test_safe_hold_torque` checks the rule on
every path.

The MuJoCo `Limiter` holds the same tuple, its measured torque being the
torque it applied in the previous cycle (the simulator's measured joint
torque), clamped to `tau_ff_magnitude`; `reset()` seeds a new session from
it the same way and forgets `h`. `submit()` takes the commit's `hold_torque`
(rejected past the joint's `tau_ff_magnitude`), and the latch bounds the
torque by the rule above with that cycle's `dq`. A latch (`request_safe()` or
a lease expiry) sets `q_des` to
the measured pose, which removes the spring's share `kp * (q_des - q)` in that
cycle, so the latched `tau_ff` applies in that cycle too, past the `tau_ff`
slew: at rest the output continues from the torque before the latch. The
gains still slew to the safe gains, and the final torque keeps its saturation
and slew. `test_mit_limiter` checks the torque across the latch at 1 kHz;
`test_limiter_vs_consumer` that both consumers bound a hold in motion alike.
`mit_command_v1.yaml`'s `safe_seed` (`tau_ff: measured_torque`, with its
`tau_ff_bound`) names this hold for both adapters.
