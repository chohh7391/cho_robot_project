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
