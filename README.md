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

```bash
pip3 install pandas
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
cd ros2_ws/src
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

# PC1 & PC2 Setting
- PC1: run controller with action server
- PC2: call action server

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up  # login with same account
```
- check IP of two PCs
```bash
tailscale ip -4
```


# Run

## Bringup

### Available controllers

- position controllers
    - ik_controller (TODO: make this controller stable)

- torque controllers
    - gravity_compensation_controller
    - joint_space_impedance_controller
    - task_space_impedance_controller
    - operational_space_controller
    - joint_space_qp_controller
    - task_space_qp_controller



### Bringup Robot in Real or Simulation

- real (change **robot ip** in cho_franka_bringup/config/real/franka.config.yaml)
```bash
source ~/ros2_ws/install/setup.bash

# position control mode
ros2 launch cho_franka_bringup bringup_real_robot.launch.py control_mode:=position controller_name:={controller_name}

# torque control mode
ros2 launch cho_franka_bringup bringup_real_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_franka_bringup bringup_real_robot.launch.py vla:=true control_mode:=position
ros2 launch cho_franka_bringup bringup_real_robot.launch.py vla:=true control_mode:=torque
```

- gazebo
```bash
source ~/ros2_ws/install/setup.bash

# position control mode
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py control_mode:=position controller_name:={controller_name}

# torque control mode
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py vla:=true control_mode:=position
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py vla:=true control_mode:=torque
```

- mujoco
```bash
source ~/ros2_ws/install/setup.bash

# position control mode
ros2 launch cho_franka_bringup bringup_mujoco_robot.launch.py control_mode:=position controller_name:={controller_name}

# torque control mode
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_franka_bringup bringup_mujoco_robot.launch.py vla:=true control_mode:=position
ros2 launch cho_franka_bringup bringup_mujoco_robot.launch.py vla:=true control_mode:=torque
```

## Test
- run general action client
```bash
source ~/ros2_ws/install/setup.bash
# change code for using desired controller before run
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/action_client.py
```

- run vla action client
```bash
source ~/ros2_ws/install/setup.bash
# real
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_client.py
# simulation
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_client.py --ros-args -p use_sim_time:=true
```

- run vla action with csv
```bash
source ~/ros2_ws/install/setup.bash
# real
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_with_csv.py --csv_path <CSV_PATH> --hz <INFERENCE_HZ> --chunk_size <CHUNK_SIZE>
# simulation
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_with_csv.py --csv_path <CSV_PATH> --hz <INFERENCE_HZ> --chunk_size <CHUNK_SIZE> --ros-args -p use_sim_time:=true
```

- run Behavior Tree
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=<YOUR_TASK>
```

## Run with 2 PC
- PC1: run controller with action server
- PC2: call action server


1. run server (PC2), don't quit this terminal
```bash
fastdds discovery --server-id 0 -l <PC2_IP> -p 11811
```

2. bringup (PC1)
```bash
export ROS_DISCOVERY_SERVER="<PC2_IP>:11811"
source ~/ros2_ws/install/setup.bash
ros2 launch cho_franka_bringup bringup_gazebo_robot.launch.py control_mode:={control_mode} controller_name:={controller_name}
```

3. call control command (PC2)
```bash
export ROS_DISCOVERY_SERVER="<PC2_IP>:11811"
source ~/ros2_ws/install/setup.bash
python3 cho_task_manager/python/action_client.py
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
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/plot_pose_log.py --path <DB3_PATH>
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