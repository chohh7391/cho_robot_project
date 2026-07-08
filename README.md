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

#### Franka (`cho_bringup_franka`)

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

#### UR (`cho_bringup_ur`)

- joint_space_position_controller
- task_space_ik_controller
- gripper_controller — Robotiq 2F-85 (`cho_controller_ur/GripperController`); functional in **gazebo**. Real hardware needs the `robotiq_driver` hardware interface (see note in the UR bringup section).



### Bringup Robot in Real or Simulation

#### Franka

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
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:={controller_name}

# vla control mode (you can use both of position and torque) - (position: ik), (torque: task space impedance)
# before execution, change control_mode in controller config
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=position vla:=true
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque vla:=true
```

#### UR (Universal Robots)

UR arms run position-based controllers (`joint_space_position_controller`,
`task_space_ik_controller`); there is no `control_mode` argument. The Robotiq
2F-85 gripper is attached with `load_gripper:=true` (gazebo only).

- gazebo (with Robotiq 2F-85 gripper)
```bash
source ~/ros2_ws/install/setup.bash

# arm only
ros2 launch cho_bringup_ur bringup_gz_robot.launch.py controller_name:=joint_space_position_controller load_gripper:=false

# arm + Robotiq 2F-85 gripper (spawns gripper_controller)
ros2 launch cho_bringup_ur bringup_gz_robot.launch.py controller_name:=task_space_ik_controller load_gripper:=true
```

- mujoco (arm only; the native MuJoCo scene has no gripper)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_ur bringup_mujoco_robot.launch.py controller_name:=joint_space_position_controller
```

- real (set **robot ip** via `robot_ip:=<UR_IP>`)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_ur bringup_real_robot.launch.py robot_ip:=<UR_IP> controller_name:=joint_space_position_controller
```

> **Real Robotiq gripper:** `robotiq_description`/`robotiq_controllers` are installed
> via apt, but the real hardware interface (`robotiq_driver`,
> `RobotiqGripperHardwareInterface`) is not available as a Humble binary. To drive a
> real 2F-85, clone `PickNik/ros2_robotiq_gripper` into the workspace, build
> `robotiq_driver`, then add `gripper_controller` to `cho_bringup_ur/config/real/controllers.yaml`
> and spawn it from the real launch.

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

- run Behavior Tree (Task Manager)

The task manager builds a py_trees behavior tree and dispatches it by `robot_type`,
so Franka and UR implementations are fully separated under
`cho_task_manager/tasks/franka/` and `cho_task_manager/tasks/ur/`.
Controller roles per robot are read from `cho_task_manager/config/robots/<robot>.yaml`
(single source of truth).

Launch arguments:

| arg | default | description |
| --- | --- | --- |
| `task` | `pick_and_place` | task name (must exist for the given `robot_type`) |
| `robot_type` | `franka` | `franka` or `ur5e` |
| `use_sim_time` | `false` | set `true` when running against a simulator |

Available tasks per robot:

| robot_type | tasks | required bringup |
| --- | --- | --- |
| `franka` | `pick_and_place`, `peg_insert` | `control_mode:=torque` (uses `*_qp` / impedance / vla controllers) |
| `ur5e` | `pick_and_place`, `multi_move` | UR bringup; **`pick_and_place` needs `load_gripper:=true`** (it opens/closes the Robotiq 2F-85). `multi_move` needs no gripper. |

> Bring the robot up first (see Bringup above), then run the matching task.
> `peg_insert`/`pick_and_place` on Franka need a `control_mode:=torque` bringup;
> launching the task against a `position`-mode bringup fails with
> `no controller with this name exists`.
> UR `pick_and_place` includes gripper steps, so bring UR up with `load_gripper:=true`.

```bash
source ~/ros2_ws/install/setup.bash

# Franka pick & place (requires a control_mode:=torque bringup)
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_and_place robot_type:=franka use_sim_time:=true

# Franka peg_insert (VLA flow; requires control_mode:=torque vla:=true bringup)
ros2 launch cho_task_manager run_task_manager.launch.py task:=peg_insert robot_type:=franka use_sim_time:=true

# UR5e pick & place
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_and_place robot_type:=ur5e use_sim_time:=true

# UR5e multi-location move example (visits several absolute waypoints)
ros2 launch cho_task_manager run_task_manager.launch.py task:=multi_move robot_type:=ur5e use_sim_time:=true
```

## Run with 2 PC
- PC1: run controller with action server
- PC2: call action server

Use the same `ROS_DOMAIN_ID` on both PCs. The examples below use domain `25`.

1. Run FastDDS discovery server (PC2). Do not quit this terminal.
```bash
fastdds discovery --server-id 0 -l 0.0.0.0 -p 11811
```
ㅇv
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