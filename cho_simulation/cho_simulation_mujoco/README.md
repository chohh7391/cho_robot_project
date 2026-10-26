# cho_simulation_mujoco

Plugins for `mujoco_ros2_control` (`extern/mujoco_ros2_control`) that read a MuJoCo scene: a bringup given
`mujoco_plugins:=<file>` loads them into the simulation, and the simulation calls each with its model and data every
step.

## BodyPosePublisher

`cho_simulation_mujoco/BodyPosePublisher` publishes the pose of chosen bodies, the simulation's ground truth: one
`geometry_msgs/PoseStamped` per body on `<topic_prefix><body>`, and with `publish_tf` the same poses on `/tf`, child
frame the body's name.

```yaml
# mujoco_plugins:= file
/**:
  ros__parameters:
    mujoco_plugins:
      body_poses:
        type: cho_simulation_mujoco/BodyPosePublisher
        bodies: [box]                   # empty or absent: every body with a free joint
        frame_id: fr3_link0             # what the MuJoCo world is in ROS
        rate: 50.0                      # [Hz] of simulated time
        topic_prefix: /sim/body_pose/
        publish_tf: false
```

| parameter | default | |
| --- | --- | --- |
| `bodies` | `[]` | body names; empty takes every body whose first joint is free, a scene's loose objects |
| `frame_id` | `world` | the frame the poses are stamped in. They are in the MuJoCo world frame, which a robot attached at its origin shares with its root link: `fr3_link0` for the Franka |
| `rate` | `50.0` | samples per second of simulated time |
| `topic_prefix` | `/sim/body_pose/` | absolute; the body's name is appended |
| `publish_tf` | `false` | also put the poses on `/tf` |

The stamp is the simulation time, the same as `/clock`. A body the model does not have makes the plugin fail to
initialise, which stops the simulation's start-up with its name.

## Writing a plugin of a project's own

A project reads or drives a scene its own way with a plugin of its own, in its own package: a class deriving from
`mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase` (`init(node, model, data)`, `update(model, data)`,
`cleanup()`), exported with `pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)`, and
named in the project's `mujoco_plugins:=` file. `include/cho_simulation_mujoco/` is an example. Two things the plugin
interface leaves open:

- The plugin's node is a sub-node named after its entry: topics and services are prefixed with that name,
  parameters are not, so a plugin reads its own as `mujoco_plugins.<name>.<parameter>`.
- `update()` runs in the control loop: publish through a realtime publisher (`trylock()` / `unlockAndPublish()`), never
  block.

The upstream base class and its two plugins: `extern/mujoco_ros2_control/mujoco_ros2_control_plugins`.
`test/test_scene_bodies.cpp` checks which bodies, their poses and the publishing schedule on a model built in the test.
