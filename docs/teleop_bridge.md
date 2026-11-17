# Teleoperation bridge

`teleop_bridge` (in `cho_control_tools`) drives the arms' action-chunk controllers from a teleoperation
device. The device's own driver, outside this repository, publishes what the operator does; the bridge maps it
onto the robot and streams it through the controller action contract
([CONTRACT.md](../cho_interfaces/CONTRACT.md)).

```text
Quest 3 / Pico 4 Ultra --CloudXR--> Isaac Capture teleop_ros2 (NVIDIA, mode:=controller_raw)
                                      /xr_teleop/controller_data  (std_msgs/ByteMultiArray, msgpack)
                                    teleop_bridge (this repository)
                                      per arm: ChunkStream goal + absolute task ActionChunk, gripper width_command
                                    each arm's controllers.action_chunk and controllers.gripper (cho_robot_config)
```

Every robot fact comes from cho_robot_config: `--arms` names registry profiles, each with its own
`controllers.action_chunk`. A profile that drives several arms (OpenArm `both`) is refused: name its arms
(`--arms left right`). The only device today is `--input xr_controller`
([inputs/xr_controller.py](../cho_control_tools/cho_control_tools/teleop_bridge/inputs/xr_controller.py)).

## Run

The arm's action-chunk controller must be active (Franka `action_chunk:=true`; OpenArm
`mit_controller_name:=action_chunk_mit_controller`), and TF must carry its `ee_link` in its absolute goal frame.

```bash
ros2 run cho_control_tools teleop_bridge --robot franka
ros2 run cho_control_tools teleop_bridge --robot openarm --arms left right
ros2 launch cho_control_tools teleop_bridge.launch.py robot_type:=openarm arms:='left right' use_sim_time:=true
```

Without a headset, a scripted stream of the same messages (each hand squeezes and draws a circle, then kill):

```bash
ros2 run cho_control_tools teleop_bridge_fake_xr
```

With a headset, NVIDIA's reference publisher at the tag that matches Isaac Lab's pin (`isaacteleop` 1.4.145):
`examples/teleop_ros2` of [NVIDIA/IsaacCapture](https://github.com/NVIDIA/IsaacCapture) at `v1.4.145`, whose
Docker image defaults to Humble (later tags dropped Humble). Its README builds and runs the container; run it
with `-p mode:=controller_raw`, which publishes only `controller_data`. The container needs the NVIDIA
Container Toolkit (`--gpus all`), host networking, and the same DDS settings as the robot's terminals
([multi_pc.md](multi_pc.md)); accepting the CloudXR EULA (`cloudxr_accept_eula`) is the operator's.

## Operating

- **Run toggle** (right A): starts, then stops. Starting holds each arm where TF puts its `ee_link` and opens one
  `ChunkStream` goal per arm; stopping cancels them, and the controllers bring the arms to rest and hold.
- **Kill** (right B): stops whenever it is down. A button already down when the bridge starts, or when input comes
  back after a gap, counts only after it is released.
- **Squeeze** (grip button) engages that hand's clutch: from that instant the hand's motion moves the arm,
  through `mapping.xr_to_base`. Engaging never moves the arm, wherever the hand is; released, the arm holds and
  the hand can be repositioned. A hand that loses its pose or jumps further than `clutch.max_input_step` /
  `max_input_rotation_step` in one frame lets go until the squeeze is released and pressed again.
- **Trigger** closes the gripper continuously (`~/width_command`, scaled by the gripper controller's
  `width_at_open` / `width_at_closed`). A gripper controller without those parameters (the Franka hand's) is
  left alone.
- Input older than `input.timeout_sec` lets every clutch go; input missing for `stop_after_input_lost_sec` while
  running stops. A goal that ends without the bridge ending it (rejected, aborted) stops every arm.
- If the bridge itself dies, the chunks stop and each controller holds after the goal's `stream_timeout`.

Each cycle sends the target as an absolute task-space chunk of `stream.horizon_steps` identical waypoints
`1 / rate_hz` apart, so every chunk outlasts the next one's arrival; a released clutch keeps sending the held
target, which keeps the goal alive. The controller's reference limiter, joint limits and chunk blend shape the
motion between them ([cho_action_chunk/DESIGN.md](../cho_controller/utils/cho_action_chunk/DESIGN.md)).

## Settings

[config/teleop_bridge/xr_controller.yaml](../cho_control_tools/config/teleop_bridge/xr_controller.yaml) holds
every setting, its default and its meaning. `--config` (launch: `teleop_config:=`) takes a file under the same
top-level key that names only what it changes; it is merged over the defaults and an unknown key is refused.
What a robot usually sets:

- `mapping.xr_to_base` and `mapping.operator_yaw_deg`: XR world axes to the arm's absolute goal frame. The default
  is for an operator facing +x of that frame.
- `mapping.position_scale`, `mapping.follow_rotation`.
- `arms.<profile>.hand` (which hand drives which arm) and `arms.<profile>.workspace` (a clamp box in the goal frame).

## In a project repository

A project whose robot lives in its own registry runs the bridge the way its tasks run the task manager: its launch
file sets `CHO_ROBOT_CONFIG_DIR` to its registry directory and includes `teleop_bridge.launch.py` with its
`robot_type`, `arms` and `teleop_config`. Its arms need an action-chunk controller (`controllers.action_chunk`).

## Not yet verified

- Run with a headset: the XR world frame's axes as teleop_ros2 publishes them (the default `xr_to_base` assumes
  the OpenXR stage convention), the Docker image, and the DDS settings across the container.
- On a real robot.
