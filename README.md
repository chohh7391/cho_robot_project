# Cho Robot Project
This repository provides a General Robot Control Framework for ROS2 Humble.

# Installation

- dependencies
```bash
sudo apt update && sudo apt install -y \
libpoco-dev libignition-gazebo6-dev ros-humble-joint-state-broadcaster \
ros-humble-xacro ros-humble-ament-cmake-clang-format ros-humble-eigenpy \
ros-humble-hardware-interface ros-humble-pinocchio ros-humble-eiquadprog \
ros-humble-controller-manager ros-humble-moveit-core \
ros-humble-ros2-control-test-assets ros-humble-franka-description \
ros-humble-ros-gz-sim ros-humble-ros2-control ros-humble-ros2-controllers \
ros-humble-ign-ros2-control ros-humble-gz-ros2-control \
ros-humble-py-trees ros-humble-py-trees-ros ros-humble-ros-gz-bridge \
ros-humble-ros2-control-cmake
```

- libfranka
```bash
cd ~/Downloads
wget https://github.com/frankarobotics/libfranka/releases/download/0.20.3/libfranka_0.20.3_jammy_amd64.deb
sudo dpkg -i libfranka_0.20.3_jammy_amd64.deb
```

- mujoco
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

- download repository
```bash
cd ~/
mkdir -p ros2_ws/src
git clone --recursive git@github.com:chohh7391/cho_robot_project.git
```

- qpOASES
```bash
cd cho_robot_project/extern/qpOASES
mkdir build && cd build
cmake ..
sudo make install
```

- build
```bash
cd ~/ros2_ws
# for simulation
colcon build --symlink-install
# for real
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

# Run

## Bringup

- real (change **robot ip** in cho_franka_bringup/config/real/franka.config.yaml)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_franka_bringup bringup_real_robot.launch.py
```

- gazebo
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py
```

- mujoco
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_franka_bringup bringup_mujoco_robot.launch.py
```

## Client
- run general action client
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/action_client.py
```

- run vla action client
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_client.py
```

- run Behavior Tree
```bash
# real
source ~/ros2_ws/install/setup.bash
ros2 launch cho_task_manager run_task_manager.launch.py use_sim_time:=false

# simulation
source ~/ros2_ws/install/setup.bash
ros2 launch cho_task_manager run_task_manager.launch.py use_sim_time:=true
```

## Log
- log desired & current pose
```bash
source ~/ros2_ws/install/setup.bash
ros2 bag record /log/ee_pose
```

- plot /log/ee_pose
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/plot_pose_log.py
```

# Trouble Shooting
- if gazebo screen is black long time
```bash
export IGN_IP=127.0.0.1
```

- initial build error
```bash
cd ~/ros2_ws
# repeat this
source install/setup.bash
colcon build --symlink-install
```
