# Upstream provenance

The initial semantic model, collision-disable matrix, KDL settings, and FAIRINO
joint-limit values in this package were adapted from FAIRINO's
`fairino5_v6_moveit2_config` package in the `frcobot_ros2` distribution.

- Upstream project: FAIRINO `frcobot_ros2`
- Referenced package: `fairino5_v6_moveit2_config`
- Upstream package version observed during adaptation: `0.3.0`
- Upstream declared license: BSD
- Upstream maintainer/author: litao, FAIRINO Technology

No upstream URDF or mesh is copied into this package. The robot model remains
owned by `cho_description_fr5`; this package contains only the adapted MoveIt
semantic and planning configuration. Consult the upstream repository for its
complete license text and history. New cho-specific launch and safety-gate code
in this package is licensed under Apache-2.0 as declared in `package.xml`.
