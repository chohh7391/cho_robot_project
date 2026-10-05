# Installation

This guide assumes ROS 2 Humble on Ubuntu 22.04.

## Clone Repository

```bash
cd ~/
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone --recursive git@github.com:chohh7391/cho_robot_project.git
```

## ROS Dependencies

```bash
cd ~/ros2_ws/src/cho_robot_project
bash install_dependencies.bash
```

The dependency script installs three packages by apt directly and resolves
everything else with `rosdep`, running it over the whole ROS workspace source
directory.

- `libfranka` and Pinocchio are taken from the ROS distribution as an
  ABI-compatible pair, and their rosdep keys are skipped so that selection stays
  in one place.
- `libcli11-dev` covers an upstream manifest omission. `extern/openarm_can`
  builds its `openarm-can-cli` tool unconditionally and so requires CLI11 at
  CMake configure time, but its `package.xml` declares no dependencies, so
  `rosdep` has nothing to resolve. Without it a real OpenArm MIT build fails to
  configure. Installing it here means the manual `apt install libcli11-dev` step
  quoted in [OpenArm real bringup](openarm_real_bringup.md) and in
  `extern/README.md` is already done.

The script also excludes a checked-out `extern/mujoco_vendor` source package, if
present, so rosdep can install `ros-humble-mujoco-vendor` for the vendored
`mujoco_ros2_control` packages.

It first checks out any `extern/` submodule that is not checked out yet, and
only those (an initialized submodule is never moved). `git pull` does not do
that for a submodule added since your clone - `extern/bota_driver_ros2_example`
was added that way - and a package that is not checked out is an unknown key to
`rosdep --ignore-src`: `bota_driver`, `bota_driver_example`, `openarm_can` and
the `openarm_ros2` packages have no rosdep rule at all. To do it by hand after
a pull:

```bash
git submodule update --init --recursive
```

`SKIP_SUBMODULE_UPDATE=1 bash install_dependencies.bash` leaves the submodules
alone; the rosdep keys of every one still missing are then skipped (the list is
`SUBMODULE_PACKAGES` in the script), and the packages that need them will not
build. `--simulate` never touches the checkout and reports what is missing.

## MuJoCo

```bash
sudo mkdir -p /opt/mujoco
cd ~/Downloads
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.4/mujoco-3.3.4-linux-x86_64.tar.gz
sudo tar -xvf mujoco-3.3.4-linux-x86_64.tar.gz -C /opt/mujoco/

echo 'export MUJOCO_VERSION=3.3.4' >> ~/.bashrc
echo 'export MUJOCO_DIR=/opt/mujoco/mujoco-3.3.4' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib' >> ~/.bashrc
echo 'export PATH=$PATH:$MUJOCO_DIR/bin' >> ~/.bashrc
source ~/.bashrc
```

## OpenArm

`cho_description_openarm` is a vendored fork (URDF/xacro, meshes and MJCF all
live in this repository). Simulation needs no vendor SDK. Real MIT bringup uses
the pinned `extern/openarm_can` and `extern/openarm_ros2` submodules instead of
modifying vendor code.

Isaac Sim additionally needs the USD asset built once per variant — see
`cho_description_openarm/usd/README.md`. The assets are generated and gitignored.

> Real OpenArm MIT bringup is commissioning-only and physically untested.
> Invoking it starts the selected hardware component; see the required
> commissioning procedure in [OpenArm real bringup](openarm_real_bringup.md).

## Build

```bash
cd ~/ros2_ws

# for simulation
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install

# for real
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After building:

```bash
source ~/ros2_ws/install/setup.bash
```
