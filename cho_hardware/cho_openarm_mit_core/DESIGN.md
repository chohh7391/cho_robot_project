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
