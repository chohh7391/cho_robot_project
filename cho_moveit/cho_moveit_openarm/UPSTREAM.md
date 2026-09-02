# OpenArm MoveIt configuration provenance

There was no upstream OpenArm v1.0 MoveIt configuration in this workspace when
this package was created. This package therefore does not vendor an external
MoveIt config.

The robot model is loaded from the canonical Cho description entry point:
`cho_description_openarm/robots/openarm_v10/openarm_v10.urdf.xacro`. Joint
names, bounds, the `openarm_link0` base, and the `openarm_hand_tcp` tip are
derived from that model.

The SRDF was generated for the single-arm chain only. Its allowed-collision
matrix conservatively disables rigidly adjacent link pairs. The additional
left/right finger exception represents the mechanically coupled gripper's valid
closed contact; without it every closed-gripper arm state is reported in self
collision. Non-adjacent arm geometry remains collision checked.

Bimanual planning is intentionally unsupported until a separately reviewed
dual-arm SRDF, controller mapping, and inter-arm collision matrix are available.
