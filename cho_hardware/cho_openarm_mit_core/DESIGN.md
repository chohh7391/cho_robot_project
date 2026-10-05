# OpenArm MIT core contract boundary

`cho_openarm_mit_core` is the dependency root for the OpenArm MIT tuple and
safety protocol.  It owns the five-field joint tuple, generation/lease/SAFE
state-machine primitives, paired ownership contract, canonical joint ordering,
and fail-closed safety-profile loading.

It deliberately exports no controller plugin and no `ros2_control`
`SystemInterface`.  Controllers consume this contract to decide *what* tuple to
produce; hardware adapters consume it to decide *how* a validated tuple reaches
a simulator or actuator.  This keeps a future CAN adapter dependent on this
package rather than on MuJoCo or controller code.

The default real profile is unapproved and is rejected before an adapter can
open a transport. `real_conservative_commissioning` is a deliberately narrow,
untested envelope. A real adapter validates its explicit commissioning profile
and CAN interface before opening CAN or enabling a motor.

`SwitchGate` is the controller-switch rule every backend applies (the real
adapter, MuJoCo and the test fake), so the fake-based controller integration
tests run against the same rule the real adapter enforces: a partial claim of
an arm is refused; a switch that starts or stops an arm is accepted whether or
not the arm is SAFE, the backend's next `write()` puts the arm in measured SAFE,
and no producer input is evaluated until perform; at perform the backend
discards the commit the outgoing producer left unacknowledged
(`ArmConsumer::discard_commit`), so it never runs and the incoming producer
continues above it; an abandoned switch opens after `expiry_cycles`. The
first write after perform is flagged (`Cycle::performed`): a SAFE request still
pending then belongs to the outgoing producer. The class
only decides; each backend owns its SAFE tuple and commit bookkeeping (the real
adapter and the fake through `ArmConsumer`, MuJoCo through its limiter).

`ArmConsumer`'s SAFE hold is `q_des = q_measured, dq_des = 0`, the per-joint safe gains, and
`tau_ff = tau_measured`: the joint torque measured in the same read as the pose
(`observe(position, effort)`), clamped per joint to the hold's effort limit (the profile's
`tau_ff_magnitude` on the real adapter). A new session (`configure()`) is seeded with the torque
measured at the seed read, so a reactivation out of a hold keeps the arm's gravity torque. It used
to retain the producer's last `tau_ff`, which is not the gravity torque whenever the producer carried
part of the support in its `kp*(q_des - q)` spring, and to start every session at zero; see the
class comment for why the measured torque and not a gravity model.
