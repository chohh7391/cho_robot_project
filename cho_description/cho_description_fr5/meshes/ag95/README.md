# DH Robotics AG-95 meshes

Vendored from [ian-chuang/dh_ag95_gripper_ros2](https://github.com/ian-chuang/dh_ag95_gripper_ros2)
(`humble`, `dh_ag95_description/meshes/`), MIT licensed — see `LICENSE`,
MIT © 2024 Ian Chuang. Unmodified; still in millimetres, so the xacro applies
`scale="0.001 0.001 0.001"`.

Only the two links that are RIGID with respect to the mounting flange are taken:

- `base_link.stl` — the mounting plate
- `gripper_body.stl` — the gripper body

The upstream model drives the jaws as a four-bar linkage per side: one actuated
`left_outer_knuckle_joint` (revolute, 0 open to 0.93 closed) plus seven `mimic`
joints, and the jaw separation is trigonometric in that angle. `cho_controller_gripper`
drives ONE joint through a LINEAR width mapping, and the real gripper is commanded
as a stroke percentage over RS485, so adopting that linkage would mean changing a
controller contract shared with franka, openarm and ur. Instead `fr5_macro.xacro`
keeps its single prismatic finger joint and covers the moving parts with one box
sized to their swept volume over the whole stroke — conservative, so the collision
model always encloses the real one.

Measured from these meshes, in the mounting-flange frame (`+z` away from the flange):

| part | x | y | z |
|---|---|---|---|
| static (these two meshes) | ±63.5 mm | ±33.5 mm | −3.5 … +134.5 mm |
| jaws, swept over 0…0.93 | ±84.1 mm | ±25.4 mm | +90.4 … +200.8 mm |

The upstream `grasp_link` sits at `z = +190 mm`; the full envelope reaches
`+200.8 mm` with the jaws closed. Recompute with
`scratchpad/ag95_envelope.py` if the upstream model changes.
