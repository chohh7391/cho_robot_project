# UR5e Integration Research

## Scope

This note summarizes the official UR5e description, ROS 2 driver interface, calibration workflow, model parameters, and simulation sources needed to integrate UR5e into the Cho Robot Project alongside the existing Franka stack.

## Official URDF and Meshes

Use `ur_description` from `UniversalRobots/Universal_Robots_ROS2_Description`. The package contains:

- `urdf/ur.urdf.xacro`: top-level robot description entry point.
- `urdf/ur_macro.xacro`: serial-chain macro and frame definitions.
- `config/ur5e/default_kinematics.yaml`: nominal UR5e kinematic transforms.
- `config/ur5e/joint_limits.yaml`: joint ranges, velocity limits, and effort limits.
- `config/ur5e/physical_parameters.yaml`: mass, center of mass, and inertia parameters.
- `config/ur5e/visual_parameters.yaml`: mesh paths and visual/collision offsets.
- `meshes/ur5e/visual/*.dae` and `meshes/ur5e/collision/*.stl`.

Recommended acquisition:

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
```

Visualization smoke test:

```bash
ros2 launch ur_description view_ur.launch.xml ur_type:=ur5e
```

Key frame names to preserve when adapting to this repo:

- `base_link`: REP-103-aligned robot base.
- `base`: UR controller "Base" frame, rotated by pi about Z from `base_link`.
- `base_link_inertia`: dummy frame used because UR controller/internal frames differ from REP-103.
- `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`.
- `ft_frame`: fixed frame from `wrist_3_link`, rotated by `pi 0 0`.
- `flange`: EEF attachment frame.
- `tool0`: all-zero tool frame.

For Cho integration, mirror the Franka layout with a new description package, for example:

```text
cho_robots_description/cho_ur_description/
  robots/common/ur_robot.xacro
  robots/ur5e/ur5e.urdf.xacro
  config/ur5e/*.yaml
  meshes/ur5e/{visual,collision}/
```

## ROS 2 Driver and Hardware Interface

Use `ur_robot_driver` from `UniversalRobots/Universal_Robots_ROS2_Driver`. The repo includes:

- `ur_robot_driver`: hardware interface and launch files.
- `ur_controllers`: UR-specific controllers.
- `ur_calibration`: factory calibration extraction.
- `ur_moveit_config`: example MoveIt configuration.
- `ur_dashboard_msgs`: dashboard messages.

Install from apt when possible:

```bash
sudo apt install ros-${ROS_DISTRO}-ur
```

or source-build the driver repo branch matching the ROS distro.

Real robot startup:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=<ROBOT_IP> \
  launch_rviz:=true
```

Mock hardware startup:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_mock_hardware:=true
```

The `ur.ros2_control.xacro` macro selects:

- `mock_components/GenericSystem` when `use_mock_hardware:=true`.
- `ur_robot_driver/URPositionHardwareInterface` for real hardware.

Important hardware parameters include `robot_ip`, `ur_type`, RTDE recipe files, reverse/script/trajectory ports, `tf_prefix`, `non_blocking_read`, `servoj_gain`, `servoj_lookahead_time`, `kinematics/hash`, tool communication parameters, and `verify_robot_model`.

Unlike Franka, UR trajectory control is position-oriented. The official driver README states trajectory control currently supports position commands. Treat UR as a different command-capability class:

- Supported integration path: joint trajectory, forward position, forward velocity, force mode, freedrive, IO/status.
- Do not assume Franka-style joint torque command or model-based torque impedance can be reused on real UR5e.
- For Cho controllers, add a capability layer before loading controller plugins.

Default active controllers in `ur_control.launch.py` include:

- `joint_state_broadcaster`
- `io_and_status_controller`
- `speed_scaling_state_broadcaster`
- `force_torque_sensor_broadcaster`
- `tcp_pose_broadcaster`
- `ur_configuration_controller`
- `friction_model_controller`
- initially selected joint controller, usually `joint_trajectory_controller`

Inactive command controllers include:

- `forward_velocity_controller`
- `forward_position_controller`
- `forward_effort_controller`
- `force_mode_controller`
- `freedrive_mode_controller`
- `passthrough_trajectory_controller`
- `motion_primitive_forward_controller`

## UR5e Kinematics, Limits, and Inertial Data

Official UR DH parameters for UR5e/UR7e:

| Joint | a [m] | d [m] | alpha [rad] |
| --- | ---: | ---: | ---: |
| 1 | 0 | 0.1625 | pi/2 |
| 2 | -0.425 | 0 | 0 |
| 3 | -0.3922 | 0 | 0 |
| 4 | 0 | 0.1333 | pi/2 |
| 5 | 0 | 0.0997 | -pi/2 |
| 6 | 0 | 0.0996 | 0 |

Nominal `ur_description` kinematics uses the same main geometry:

- shoulder z: `0.1625`
- upper arm roll: `1.570796327`
- forearm x: `-0.425`
- wrist_1 x/z: `-0.3922`, `0.1333`
- wrist_2 y: `-0.0997`
- wrist_3 y: `0.0996`

Joint limits from `config/ur5e/joint_limits.yaml`:

| Joint | Position | Velocity | Effort |
| --- | --- | --- | ---: |
| shoulder_pan | +/- 360 deg | 180 deg/s | 150 Nm |
| shoulder_lift | +/- 360 deg | 180 deg/s | 150 Nm |
| elbow | +/- 180 deg, artificially limited for planning | 180 deg/s | 150 Nm |
| wrist_1 | +/- 360 deg | 180 deg/s | 28 Nm |
| wrist_2 | +/- 360 deg | 180 deg/s | 28 Nm |
| wrist_3 | +/- 360 deg | 180 deg/s | 28 Nm |

Masses from `physical_parameters.yaml`:

| Link | Mass [kg] |
| --- | ---: |
| base | 4.0 |
| shoulder | 3.761 |
| upper_arm | 8.058 |
| forearm | 2.846 |
| wrist_1 | 1.37 |
| wrist_2 | 1.3 |
| wrist_3 | 0.365 |

The official UR technical sheet lists UR5e as 6-DOF, 5 kg payload, 850 mm reach, 20.7 kg robot weight, max joint speed 180 deg/s, 500 Hz system update frequency, and built-in force/torque sensor accuracy of 4 N.

## Real Robot Calibration

Run calibration before using a physical UR5e. The driver can operate without it, but UR documentation warns TCP pose can be off by centimeters.

Procedure:

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=<ROBOT_IP> \
  target_filename:="${HOME}/my_ur5e_calibration.yaml"
```

Then pass the generated file into the description/driver path as the `kinematics_parameters_file`. The driver checks the connected robot calibration hash against the configured kinematics. If the robot and file do not match, startup can report a calibration mismatch.

Integration rule: keep per-robot calibration YAML outside generic model defaults, for example:

```text
cho_robots_bringup/cho_ur_bringup/config/real/ur5e_calibration.yaml
```

Avoid committing lab-specific calibration unless the repository policy allows robot-specific configuration files.

## Gazebo and MuJoCo Model Sources

Gazebo Classic for Humble:

- Source: `UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation`
- Branch: `humble`
- Package: `ur_simulation_gazebo`
- Launch: `ros2 launch ur_simulation_gazebo ur_sim_control.launch.py`
- Note: Gazebo Classic is Humble-only in this repo; the README states it is not supported from ROS 2 Jazzy onward.

Gazebo/GZ:

- Source: `UniversalRobots/Universal_Robots_ROS2_GZ_Simulation`
- Branches include Humble/Jazzy/Kilted/Rolling support paths.
- Package: `ur_simulation_gz`
- Launch: `ros2 launch ur_simulation_gz ur_sim_control.launch.py`

MuJoCo:

- Source candidate: `google-deepmind/mujoco_menagerie/universal_robots_ur5e`
- This is a native MJCF model source, useful if Cho keeps MuJoCo XML assets separate from URDF/xacro.
- If using `mujoco_ros2_control`, prefer generating or adapting MJCF from `ur_description` only after comparing frames, joint axes, masses, and inertias against the menagerie model.

## Suggested Cho Integration Plan

1. Add `cho_ur_description` with a thin wrapper around official `ur_description` first. Do not rewrite UR kinematics by hand.
2. Add `cho_ur_bringup` with `bringup_real_robot.launch.py`, `bringup_gazebo_robot.launch.py`, and later `bringup_mujoco_robot.launch.py`.
3. Add a robot capability config:

```yaml
robot_type: ur5e
command_interfaces:
  joint_position: true
  joint_velocity: true
  joint_effort_real: false
  force_mode: true
state_interfaces:
  joint_position: true
  joint_velocity: true
  joint_effort: true
  tcp_pose: true
  ft_sensor: true
```

4. Keep Franka torque controllers separate from UR position/trajectory controllers until a proper abstraction exists.
5. Map task manager actions to `joint_trajectory_controller` first. Add velocity/force-mode behaviors only after verifying safety behavior in simulation.

## Sources

- Universal Robots ROS2 Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Universal Robots ROS2 Description: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- UR ROS2 usage docs: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html
- UR calibration docs: https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_calibration/doc/index.html
- UR DH parameter article: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
- UR5e technical specifications: https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/complianceUR5e/H_g5_sections/appendix_g5/tech_spec_sheet.htm
- UR Gazebo Classic simulation: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation
- UR Gazebo/GZ simulation: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation
- MuJoCo Menagerie UR5e: https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e
