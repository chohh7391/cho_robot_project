# OpenArm MIT core contract boundary

`cho_openarm_mit_core` is the dependency root for the OpenArm MIT tuple and
safety protocol.  It owns the five-field joint tuple, generation/lease/SAFE
state-machine primitives, paired ownership contract, canonical joint ordering,
and fail-closed safety-profile loading.

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
misordered limits. Of the four profiles in
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
  five fields and four protocol handles is refused. A switch that starts or
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
read as the pose (`observe(position, effort)`), clamped per joint to the hold's
effort limit (the profile's `tau_ff_magnitude` on the real adapter). A new
session (`configure()`) is seeded with the torque measured at the seed read.
The `ArmConsumer` class comment in `mit_protocol.hpp` gives the reasons.
