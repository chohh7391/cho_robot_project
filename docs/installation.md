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

The dependency script uses `rosdep` from the whole ROS workspace source directory, skips the `libfranka` rosdep key, and excludes a local `extern/mujoco_vendor` source package if present.

This avoids installing the apt version of `libfranka` while allowing `mujoco_vendor` to resolve to the apt package.

## libfranka

```bash
cd ~/Downloads
wget https://github.com/frankarobotics/libfranka/releases/download/0.20.3/libfranka_0.20.3_jammy_amd64.deb
sudo dpkg -i libfranka_0.20.3_jammy_amd64.deb
```

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

Nothing extra to install. `cho_description_openarm` is a vendored fork (URDF/xacro,
meshes and MJCF all live in this repository), and the bringup is simulation-only, so
it builds from the workspace with no submodule and no vendor SDK.

Isaac Sim additionally needs the USD asset built once per variant — see
`cho_description_openarm/usd/README.md`. The assets are generated and gitignored.

> Real OpenArm hardware is **not** supported yet. The description carries a
> `hardware:='real'` branch pointing at `openarm_hardware/OpenArmHW`, but the
> `openarm_ros2` / `openarm_can` submodules are not vendored and there is no
> `config/real/` or real launch file, so `hardware:=real` does not build. Use
> `hardware:=mock` for a driver-free smoke test. Tracked in `todo/OPENARM_TODO.md`.

## Build

```bash
cd ~/ros2_ws

# for simulation
colcon build --symlink-install

# for real
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

After building:

```bash
source ~/ros2_ws/install/setup.bash
```
