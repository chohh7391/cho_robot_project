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

The dependency script updates and installs the ROS distribution's matching
`libfranka` and Pinocchio packages together, then uses `rosdep` from the whole
ROS workspace source directory. It skips their rosdep keys to keep that package
selection centralized, and excludes a checked-out
`extern/mujoco_vendor` source package, if present, so rosdep can install
`ros-humble-mujoco-vendor` for the vendored `mujoco_ros2_control` packages.

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

## qpOASES

```bash
cd ~/ros2_ws/src/cho_robot_project/extern/qpOASES
mkdir build && cd build
cmake ..
sudo make install
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
