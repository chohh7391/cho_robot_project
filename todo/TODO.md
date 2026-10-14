# TODO

- OpenArm real hardware: run the checks in
  [docs/openarm_real_bringup.md, Re-check on hardware](../docs/openarm_real_bringup.md#re-check-on-hardware),
  in particular the SAFE hold latched in motion (sag at most
  `(|payload torque| + band + |model error|) / safe_hold_stiffness` per joint) and each joint's
  `|dq|` at rest, which must stay well below `torque.safe_hold_motion_velocity` (0.1 rad/s).
- CI flake: `test_task_space_return_to_zero` `Fixture.FrictionSourceDecidesWhichVelocitySignIsUsed`
  once failed on CI waiting for the controller to become ready (`Access::ready()`, 900 cycles;
  passes locally, also under CPU load). If it repeats, wait on the condition with a longer bound.
