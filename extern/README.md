# Vendor source policy

Directories in this folder are pinned third-party sources.  They are not the
owners of Cho launch files, robot descriptions, or controllers.

## OpenArm

Only these packages are part of the OpenArm real-hardware dependency boundary:

- `openarm_can` from `extern/openarm_can`
- `openarm_hardware` from `extern/openarm_ros2/openarm_hardware`

Do not use `openarm`, `openarm_bringup`, or
`openarm_bimanual_moveit_config` from the upstream repository in Cho launch
files.  Their package names do not collide with the `cho_*` names, so the
boundary is enforced with an explicit package allowlist instead of modifying
the upstream checkout with `COLCON_IGNORE` files:

Use the checked helper, which verifies both gitlinks and applies the allowlist:

```bash
~/ros2_ws/src/cho_robot_project/tools/build_openarm_vendor.sh
```

The helper accepts no pass-through arguments: package selection, discovery
paths, and all other colcon options are rejected rather than being able to
override the allowlist.  `--check-only` is its sole diagnostic option.  It
also rejects a mismatch among its reviewed SHA constants, each checked-out
HEAD, and each superproject index gitlink, as well as tracked, staged, or
non-ignored untracked source changes inside either vendor checkout. Ignored
build/package artifacts are allowed and are not treated as source changes. Run its negative tests with
`tools/test_build_openarm_vendor.sh`.

The equivalent manual build is `colcon build --symlink-install
--packages-select openarm_can openarm_hardware` from `~/ros2_ws`.

`openarm_can` 1.3.4 builds its optional command-line tools unconditionally and
therefore needs CLI11.  On Ubuntu 22.04 install it before building:

```bash
sudo apt-get install libcli11-dev
```

The upstream `openarm_can/package.xml` does not currently declare CLI11, so
`rosdep` alone does not install this system dependency.  The hardware library
itself uses Linux SocketCAN and has no additional ROS package dependency.
`openarm_hardware` directly needs `hardware_interface`, `pluginlib`, `rclcpp`,
`rclcpp_lifecycle`, and the CMake target exported by `openarm_can`.
Upstream finds and links `rclcpp_lifecycle` in CMake but omits it from
`openarm_hardware/package.xml`; this is a second upstream manifest caveat and
may prevent `rosdep` from provisioning a minimal non-ROS-desktop environment.

Initialize the exact vendor revisions recorded by the superproject with:

```bash
git submodule update --init --recursive \
  extern/openarm_ros2 extern/openarm_can
git submodule status extern/openarm_ros2 extern/openarm_can
```

To evaluate an upstream update, fetch in the relevant submodule, check out the
candidate commit, build only the two packages above, and commit the updated
gitlink in the superproject.  Never leave `main` as a floating dependency.
Cho-specific policy belongs in a Cho wrapper or adapter; do not patch vendor
source unless the design review records why a wrapper is insufficient.
