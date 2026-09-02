# Upstream provenance

The semantic model, joint limits, KDL settings, OMPL settings, and controller
mapping were adapted from `extern/Universal_Robots_ROS2_Driver/ur_moveit_config`,
the official Universal Robots ROS 2 Driver MoveIt package. The files are pinned
to the vendored source in this repository and adapted to `cho_description_ur`,
the Cho action bridge, and the Cho Gazebo `joint_trajectory_controller`.

Initial support is deliberately limited to UR5e with an empty TF prefix and no
attached gripper. This prevents silent URDF/SRDF and controller-name mismatches.
