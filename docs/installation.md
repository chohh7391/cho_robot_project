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

## cuRobo / cuMotion GPU planning (optional)

Only needed to pass `cumotion:=true` to an FR5 MoveIt entry point (see
`cho_moveit/README.md`). Nothing else in this repository uses it, and with the
flag off the stack behaves as if none of this were installed. Skip this whole
section otherwise.

> cuRobo v0.7.x is under the NVIDIA License with a **non-commercial** use
> limitation -- research or evaluation only. The Humble path is pinned to it
> because `isaac_ros_cumotion` release-3.2 imports the v1 API. Details, the
> pinned SHA and four install traps are in `extern/VENDORED_CUROBO.md`.

Prerequisites: an NVIDIA GPU, and a CUDA toolkit whose version matches the
PyTorch wheel below. This machine has 12.8 installed alongside a newer default;
`CUDA_HOME` is set explicitly further down for exactly that reason.

### 1. Vendor sources

Three of them are submodules, so the `git clone --recursive` at the top of this
guide already fetched them. If you cloned without `--recursive`:

```bash
cd ~/ros2_ws/src/cho_robot_project
git submodule update --init extern/curobo extern/isaac_ros_cumotion
```

| Submodule | Pinned at |
|---|---|
| `extern/curobo` | tag `v0.7.8` (`d64c4b00…`) -- the last cuRobo v1 release |
| `extern/isaac_ros_cumotion` | `release-3.2` -- the last branch that supports Humble |

`isaac_ros_common` is deliberately absent. The cuMotion packages reach into it
at build time for two version-stamping resources, but its own package
hard-requires NVIDIA VPI, which nothing here uses. `cho_moveit_curobo_deps`
supplies those two resources instead, so the upstream repository is not needed.

`nvblox_msgs` is the exception: it is **not** a submodule. `cumotion_planner.py`
imports it unconditionally, but the full `isaac_ros_nvblox` tree carries ten
packages plus its own nested submodules, and `nvblox_ros` needs the nvblox CUDA
library that nothing here uses. A submodule cannot express "this one package",
so take a sparse checkout:

```bash
cd ~/ros2_ws/src/cho_robot_project/extern
mkdir -p nvblox_msgs_src && cd nvblox_msgs_src
git init -q .
git remote add origin https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
git config core.sparseCheckout true
echo "nvblox_msgs/*" > .git/info/sparse-checkout
git fetch -q --depth 1 origin release-3.2 && git checkout -q FETCH_HEAD
```

### 2. colcon boundary -- do not skip this

`extern/isaac_ros_cumotion` carries eleven packages; this project builds five.
One of the six it does not, `curobo_core`, **fails** a plain workspace build: it
wants torch in the system interpreter and its own `curobo/` submodule is empty.
`extern/curobo` is worse -- it has no `package.xml`, so colcon identifies it by
`setup.py` and tries to build the pip tree.

colcon's only discovery-level opt-out is a `COLCON_IGNORE` file inside the
package directory, and those directories belong to submodules, whose contents
are upstream's. So the markers cannot be committed -- but their creation is
scripted and verified:

```bash
cd ~/ros2_ws/src/cho_robot_project
tools/setup_curobo_vendor.sh
```

```
colcon boundary OK: 6 packages (isaac_ros_cumotion isaac_ros_cumotion_interfaces
isaac_ros_cumotion_moveit isaac_ros_cumotion_python_utils
isaac_ros_cumotion_robot_description nvblox_msgs)
```

It is idempotent, and `--check-only` verifies without writing. Re-run it after
re-checking out either submodule -- `git submodule update` restores upstream's
tree and takes the markers with it.

### 3. Python environment

cuRobo runs inside the ROS node, so it needs an interpreter that can import both
it and `rclpy`. **Python 3.10 is mandatory**: Humble's rclpy ships
`_rclpy_pybind11.cpython-310-*.so`, which 3.11 and 3.12 cannot load. rclpy comes
from `/opt/ros/humble` on `PYTHONPATH`, not from the venv.

```bash
python3.10 -m venv ~/ros2_ws/.venv-curobo
# Ubuntu 22.04 seeds setuptools 59, which predates PEP 660 and refuses the
# editable install below.
~/ros2_ws/.venv-curobo/bin/pip install -U pip wheel "setuptools>=70,<81" setuptools_scm
~/ros2_ws/.venv-curobo/bin/pip install torch==2.7.0 \
  --index-url https://download.pytorch.org/whl/cu128
```

```bash
cd ~/ros2_ws/src/cho_robot_project/extern/curobo
# The toolkit must match the wheel's CUDA (12.8), not whatever /usr/local/cuda
# points at. Pin the architecture rather than relying on GPU autodetection.
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST="12.0"    # sm_120 / RTX 50-series; set yours
export MAX_JOBS=4
~/ros2_ws/.venv-curobo/bin/pip install -e . --no-build-isolation

# cuRobo 0.7.8 declares no upper bounds, and current warp-lang removed the
# wp.torch accessor it uses. Pin the combination that works.
~/ros2_ws/.venv-curobo/bin/pip install "warp-lang==1.10.0" "trimesh==4.9.0"
```

Check that the kernels were built for your GPU and that they load:

```bash
for so in src/curobo/curobolib/*.so; do
  $CUDA_HOME/bin/cuobjdump -lelf "$so" | grep -oE 'sm_[0-9]+' | sort -u
done
~/ros2_ws/.venv-curobo/bin/python -c \
  "import curobo, torch; print(curobo.__version__, torch.cuda.get_device_name(0))"
```

### 4. Build order

`cho_moveit_curobo_deps` must be built **and sourced** before the cuMotion
packages: colcon runs `setup.py` during package identification, and
`isaac_ros_cumotion_python_utils` looks up an `isaac_ros_common` resource at
import time. One invocation cannot satisfy that.

```bash
cd ~/ros2_ws
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install \
  --packages-select cho_moveit_curobo_deps nvblox_msgs
source install/setup.bash
MAKEFLAGS='-j2 -l2' colcon build --parallel-workers 2 --symlink-install \
  --packages-select isaac_ros_cumotion_interfaces isaac_ros_cumotion_python_utils \
                    isaac_ros_cumotion_robot_description isaac_ros_cumotion \
                    isaac_ros_cumotion_moveit
```

After that the normal build below covers everything, and

```bash
ros2 launch cho_bringup_fr5 bringup_gz_moveit.launch.py cumotion:=true
```

should log `Planning pipelines: joint=ompl, task=isaac_ros_cumotion` followed by
`cuMotion is ready for planning queries!`.

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
