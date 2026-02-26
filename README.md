# Version
- libfranka: 0.20.3
- franka_ros2: v2.2.0

# Installation

- dependencies
```bash
sudo apt update
sudo apt install libpoco-dev libignition-gazebo6-dev ros-humble-joint-state-broadcaster ros-humble-xacro ros-humble-ament-cmake-clang-format ros-humble-eigenpy ros-humble-hardware-interface ros-humble-pinocchio ros-humble-eiquadprog ros-humble-controller-manager ros-humble-moveit-core ros-humble-ros2-control-test-assets ros-humble-franka-description ros-humble-ros-gz-sim ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control ros-humble-gz-ros2-control ros-humble-py-trees ros-humble-py-trees-ros ros-humble-ros-gz-bridge ros-humble-ros2-control-cmake
```

- libfranka
```bash
wget https://github.com/frankarobotics/libfranka/releases/download/0.20.3/libfranka_0.20.3_jammy_amd64.deb
sudo dpkg -i libfranka_0.20.3_jammy_amd64.deb
```

- qpOASES
```bash
cd qpOASES
rm -rf build
mkdir build && cd build
cmake ..
sudo make install
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

- build
```bash
# for simulation
colcon build --symlink-install
# for real
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

# Run
- bringup real
```bash
ros2 launch cho_franka_bringup bringup_real_robot.launch.py
```

- bringup gazebo
```bash
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py
```

- bringup mujoco
```bash
ros2 launch cho_franka_bringup bringup_mujoco_robot.launch.py
```

- bringup mujoco
```bash
export MUJOCO_DIR=$HOME/mujoco-3.5.0

```

- run action client
```bash
python3 /home/home/ros2_ws/src/cho_robot_project/cho_task_manager/python/action_client.py
```
```bash
python3 /home/home/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_client.py
```

- run Behavior Tree
```bash
# real
ros2 launch cho_task_manager run_task_manager.launch.py use_sim_time:=false

# gazebo
ros2 launch cho_task_manager run_task_manager.launch.py use_sim_time:=true
```

- log desired * current pose
```bash
ros2 bag record /log/ee_pose
```

- plot /log/ee_pose
```bash
python3 /home/home/ros2_ws/src/cho_robot_project/cho_task_manager/python/plot_pose_log.py
```

# Trouble Shooting
- if gazebo screen is black long time
```bash
export IGN_IP=127.0.0.1
```

- mock_franka_gripper.py: Executable not found
```bash
chmod +x cho_robots_bringup/cho_franka_bringup/scripts/mock_franka_gripper.py
```

- build error
```bash
source install/setup.bash
colcon build --symlink-install
```