# bota_ft_sensor

Runs the driver of a Bota Systems FT sensor mounted on one of the arms here.
Nothing of Bota's is copied: the node is `bota_driver_node` from `bota_driver`
(`extern/bota_driver_ros2`) and the default configuration is
`bota_driver_example`'s (`extern/bota_driver_ros2_example`), so both submodules
have to be checked out ([installation](../../docs/installation.md#ros-dependencies)).

```bash
ros2 launch bota_ft_sensor bota_ft_sensor_mounted.launch.py
```

Only the driver starts. The arm's robot description carries the sensor's links
(`cho_description_franka`'s `franka_robot.xacro`): the Franka real, MuJoCo and Gazebo
bringups with `load_ft_sensor:=true` (default false), Isaac always. So a second `robot_state_publisher` and RViz would conflict with the
bringup. For Bota's own
standalone views use their launches directly, e.g.
`ros2 launch bota_driver_example plotjuggler.launch.py`.

| argument | default | |
| --- | --- | --- |
| `bota_ft_sensor_link_name` | `bota_ft_sensor` | sensor link prefix; also the driver node name and the topic prefix below |
| `config_file` | `bota_driver_example`'s `bota_config/bota_binary.json` | Bota driver JSON configuration (their serial example); pass your own for another sensor |
| `output_rate` | `500.0` | driver output rate [Hz] |

## Interface

With the default name:

| | |
| --- | --- |
| `/bota_ft_sensor/wrench` | `geometry_msgs/WrenchStamped` |
| `/bota_ft_sensor/tare` | `std_srvs/Trigger`, zeroes the sensor |

The task manager reads both: `TareFTSensorServiceBehavior` calls the tare service
and `SafetyMonitorBehavior` subscribes to the wrench ([tasks](../../docs/tasks.md)).
The Isaac bringup serves the same two names from an emulated sensor
(`publish_ft:=true`), so a task does not change between hardware and simulation.
The driver also publishes `<name>/imu` and `<name>/bota_frame`.

## Frames

On the FR3 the sensor sits between `fr3_link8` and `fr3_hand`: in
`cho_description_franka`, `bota_ft_sensor_wrench` is `fr3_link8` shifted 0.035 m
along z with no rotation, and `fr3_hand` hangs off it at Rz(−π/4)
(`fr3_hand_joint`). The transform from the sensor's local frame to the TCP frame
is `R_tcp_ft = Rx(π) × Rz(−π/4)`. A real wrench is rotated into the world frame
with `q_ft_to_world = quat_mul(ee_quat, q_ft_to_tcp)`, where `ee_quat` is the
pose on `/ee_state/pose` (the `fr3_hand_tcp` frame).
