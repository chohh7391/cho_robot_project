# Cho Robot Project
This repository provides a General Robot Control Framework for ROS2 Humble.

# Installation

See [docs/installation.md](docs/installation.md).

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
    (TODO: make this controller stable in REAL)
    - joint_space_position_controller
    - task_space_ik_controller
    

- torque controllers
    - gravity_compensation_controller
    - joint_space_impedance_controller
    - task_space_impedance_controller
    - operational_space_controller
    - joint_space_qp_controller
    - task_space_qp_controller



### Bringup Robot in Real or Simulation

- real (change **robot ip** in cho_bringup_franka/config/real/franka.config.yaml)
```bash
source ~/ros2_ws/install/setup.bash

# position control mode
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=position controller_name:={controller_name}

# torque control mode
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=position vla:=true
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque vla:=true
```

- gazebo
```bash
source ~/ros2_ws/install/setup.bash

# position control mode
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position controller_name:={controller_name}

# torque control mode
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position vla:=true
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=torque vla:=true
```

- mujoco
```bash
source ~/ros2_ws/install/setup.bash

# position control mode
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=position controller_name:={controller_name}

# torque control mode
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=position vla:=true
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque vla:=true
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

Use the same `ROS_DOMAIN_ID` on both PCs. The examples below use domain `25`.

1. Run FastDDS discovery server (PC2). Do not quit this terminal.
```bash
fastdds discovery --server-id 0 -l 0.0.0.0 -p 11811
```

2. Bringup robot or simulation (PC1).
```bash
export ROS_DOMAIN_ID=25
export ROS_DISCOVERY_SERVER="<PC2_IP>:11811"
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:={control_mode} controller_name:={controller_name}
```

3. Call control command (PC2).
```bash
export ROS_DOMAIN_ID=25
export ROS_DISCOVERY_SERVER="<PC2_IP>:11811"
source ~/ros2_ws/install/setup.bash
python3 cho_task_manager/python/action_client.py
```

4. Check ROS graph with `ros2` CLI (PC2).

FastDDS Discovery Server clients can communicate correctly while `ros2 topic list`
only shows `/parameter_events` and `/rosout`. For graph inspection commands such
as `ros2 topic list` and `ros2 node info`, run the CLI as a FastDDS
`SUPER_CLIENT`.

Create a profile file on PC2. Replace `<PC2_IP>` with the same IP used above.

```bash
cat > /tmp/fastdds_super_client.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="super_client_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
          <discoveryServersList>
            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
              <metatrafficUnicastLocatorList>
                <locator>
                  <udpv4>
                    <address><PC2_IP></address>
                    <port>11811</port>
                  </udpv4>
                </locator>
              </metatrafficUnicastLocatorList>
            </RemoteServer>
          </discoveryServersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF
```

Then run ROS CLI commands with this profile.

```bash
export ROS_DOMAIN_ID=25
unset ROS_DISCOVERY_SERVER
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_super_client.xml
source ~/ros2_ws/install/setup.bash

ros2 topic list --no-daemon
ros2 node list --no-daemon
```

Make sure `FASTRTPS_DEFAULT_PROFILES_FILE` points to the XML file itself, not the
directory:

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE
ls -l $FASTRTPS_DEFAULT_PROFILES_FILE
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