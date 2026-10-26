# Project template

A research project's own code lives in a repository of its own, built as an overlay workspace on the core (this
repository), never in the core's packages. What a project needs from the core goes into main as a general extension
point, and the project moves its pin to the core tag that has it; code a second project needs moves into the core.

This directory is such a repository's starting point:

- `project_tasks/` — a package that registers one task, `example_joint_wave`, with `cho_task_manager`: its tree
  (`project_tasks/joint_wave.py`), its parameters file (`config/joint_wave.yaml`) and its tests. How a task is
  registered, takes parameters and is tested: [cho_task_manager README](../../cho_task_manager/README.md).
- `project.repos` — the core version the project builds on.
- `COLCON_IGNORE` — keeps the template out of the core's own build; a copy does not take it.
  `cho_task_manager/test/test_task_registry.py` loads the template's task from here, so a core change that breaks it
  fails there.

## Starting a project

1. Copy this directory's contents into the new repository, without `COLCON_IGNORE`.
2. Rename the package: its directory, `name` in `package.xml` and `setup.py`, `resource/project_tasks`, the Python
   package directory and the imports. No `cho_` prefix: the core's packages have it, and the core's launch golden
   test needs a case of its own for every installed `cho_bringup_*`.
3. Rename the task: the entry point in `setup.py` and `TASK_NAME`. A task name is global across every installed
   package, so start it with the project's name, and a core task added later cannot collide with it.
4. Pin the core: `version` in `project.repos`, a core tag.

## The workspaces

The core is built in a workspace of its own, at the pinned version; the project's workspace overlays it:

```text
~/core_ws/src/cho_robot_project      vcs import --recursive src < project.repos
~/<project>_ws/src/<project>         this repository
```

```bash
# The core: once, and again when the pin moves
mkdir -p ~/core_ws/src && cd ~/core_ws
vcs import --recursive src < ~/<project>_ws/src/<project>/project.repos
```

Then install the core's dependencies and build it as [installation.md](../../docs/installation.md) says, in
`~/core_ws` instead of `~/fire_ws`. A core workspace already built serves as well, checked out at the pin.

```bash
# The project, on top of it
cd ~/<project>_ws
source ~/core_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

A terminal that runs the project sources the project's `install/setup.bash`, which brings the core's with it. A
task's entry point is read from the package's install, written when it is built: adding, renaming or removing a task
in `setup.py` takes a `colcon build`, also under `--symlink-install`; changing a tree does not.

## Running the example

The core's Franka bringup in MuJoCo, position mode, then the task from the project's workspace:

```bash
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=position controller_name:=joint_space_position_controller
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka task:=example_joint_wave \
  control_mode:=position use_sim_time:=true \
  task_params:=$(ros2 pkg prefix project_tasks)/share/project_tasks/config/joint_wave.yaml
```

It switches to the position hold controller, goes to the registry's task home, turns joint 7 out by 0.15 rad and back
twice, and comes home. Without `task_params:=` it takes its defaults (`DEFAULTS` in `joint_wave.py`).

## An arm controller of the project's own

A project controller that claims the arm's command interfaces must be taken down by every exclusive switch, the one
`guarded_mission()` aborts to the hold controller with included. The switch finds the arm's controllers in the
registry's `controllers.additional_arm`, and a project adds its own there without copying the registry: it lists
them in a file of its own,

```yaml
# <bringup package>/config/robot_config_additions.yaml
franka:
  additional_arm: [<controller>]
```

and the package that installs the file adds it to `CHO_ROBOT_CONFIG_ADDITIONS` with an environment hook. Every
registry load reads the files listed there; the format and what is refused: the docstring of
[registry.py](../../cho_robot_config/cho_robot_config/registry.py). In an ament_cmake package:

```cmake
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_config_additions.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_config_additions.sh.in")
```

```text
# env-hooks/robot_config_additions.dsv.in
prepend-non-duplicate;CHO_ROBOT_CONFIG_ADDITIONS;share/@PROJECT_NAME@/config/robot_config_additions.yaml
# env-hooks/robot_config_additions.sh.in
ament_prepend_unique_value CHO_ROBOT_CONFIG_ADDITIONS "$AMENT_CURRENT_PREFIX/share/@PROJECT_NAME@/config/robot_config_additions.yaml"
```

A terminal that sourced the project's workspace then switches the project's controllers like the core's.

## A world and scene data of the project's own

In MuJoCo (the Franka bringup, `bringup_mujoco_robot.launch.py`), a project brings the scene its robot works in and
reads what happens in it, without touching the core's models:

- `mujoco_world:=<file>`: an MJCF file of the project's world, anywhere on disk, with its assets beside it -- a floor,
  a bench, the objects, cameras, lights. The bringup composes it with the robot's model at launch
  ([mujoco_scene.py](../../cho_bringup/cho_bringup_common/cho_bringup_common/mujoco_scene.py)); the world replaces the
  robot's own floor, its names stay as written, and it sets no `<option>` (physics stays the robot model's). The
  robot stands at the world's origin, so a pose in the world is a pose in its root link (`fr3_link0`).
- `mujoco_plugins:=<file>`: `mujoco_ros2_control` plugins to load into the simulation, each handed the MuJoCo model
  and data every step. The core's `cho_simulation_mujoco/BodyPosePublisher` publishes bodies' poses as
  `PoseStamped`, which a task latches with `PoseTargetBehavior`; a project writes its own plugin for anything else
  ([cho_simulation_mujoco README](../../cho_simulation/cho_simulation_mujoco/README.md)).

```yaml
# <bringup package>/config/mujoco/plugins.yaml
/**:
  ros__parameters:
    mujoco_plugins:
      body_poses:
        type: cho_simulation_mujoco/BodyPosePublisher
        bodies: [box]
        frame_id: fr3_link0
```
