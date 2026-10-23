# Installation

This guide assumes ROS 2 Humble on Ubuntu 22.04.

## Clone Repository

```bash
cd ~/
mkdir -p fire_ws/src
cd fire_ws/src
git clone --recursive git@github.com:chohh7391/cho_robot_project.git
```

## ROS Dependencies

```bash
cd ~/fire_ws/src/cho_robot_project
bash install_dependencies.bash
```

The dependency script installs two packages by apt directly and resolves
everything else with `rosdep`, running it over the whole ROS workspace source
directory.

- `libfranka` and Pinocchio are taken from the ROS distribution as an
  ABI-compatible pair, and their rosdep keys are skipped so that selection stays
  in one place.
- CLI11 (`libcli11-dev`), which `extern/openarm_can` needs at configure time but
  does not declare, comes from the `cli11` key that
  `cho_hardware_openarm_mit_real/package.xml` declares for it.

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

`extern/mujoco_ros2_control` links the `libmujoco` of `ros-humble-mujoco-vendor`. After
that package is upgraded to another MuJoCo version, rebuild the two packages that link
it. Their tests do not compile against MuJoCo 3.12 (`-Werror=conversion`), so build
without tests:

```bash
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install \
  --packages-select mujoco_ros2_control mujoco_ros2_control_plugins --cmake-args -DBUILD_TESTING=OFF
```

## OpenArm

`cho_description_openarm` is a vendored fork (URDF/xacro, meshes and MJCF all
live in this repository). Simulation needs no vendor SDK. Real MIT bringup uses
the pinned `extern/openarm_can` and `extern/openarm_ros2` submodules instead of
modifying vendor code.

`extern/openarm_can` needs CLI11 at configure time. Without it `openarm_can` fails
to configure, so `cho_hardware_openarm_mit_real` and `cho_bringup_openarm` are not
built and the OpenArm launch golden cases are skipped. `install_dependencies.bash`
installs it through rosdep (above); without the script:

```bash
sudo apt install libcli11-dev
```

Isaac Sim additionally needs the USD asset built once per variant — see
[cho_description_openarm/usd/README.md](../cho_description/cho_description_openarm/usd/README.md).
The assets are generated and gitignored.

> Real OpenArm MIT bringup is at the commissioning stage and starts the selected
> hardware component: [OpenArm real bringup](openarm_real_bringup.md).

## Build

```bash
cd ~/fire_ws

# for simulation
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install

# for real
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After building:

```bash
source ~/fire_ws/install/setup.bash
```
