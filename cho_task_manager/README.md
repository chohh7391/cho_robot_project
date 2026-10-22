# cho_task_manager

The py_trees task manager. `task_manager_node` builds the behavior tree of one task for one robot from the robot's
registry entry and ticks it. Running a task, every task this package ships and how a tree is put together:
[docs/tasks.md](../docs/tasks.md). This file is how a task reaches the node, so that a package outside this one can
add its own.

- `behaviors/` — the leaves: action clients, controller switches, waits, topic latches, the safety monitor.
- `subtrees/` — shared fragments: `home_subtree()`, `guarded_mission()`, `tare_ft_children()`.
- `tasks/<robot>/` — this package's trees; `tasks/core.py` registers them.
- `testing.py` — the checks a task's tests run ([Testing a task](#testing-a-task)).
- `utils/` — the registry's names and roles (`controller_names.py`), blackboard, clock, `task_params.py`.

## How a task is found

A task is an entry point in the `cho_task_manager.tasks` group, declared in the setup.py of the package that has it:

```python
entry_points={
    'cho_task_manager.tasks': [
        'myproject_wave = myproject_tasks.wave:TASK',
    ],
},
```

- The entry point's name is the task's, what `task:=` selects.
- Its object is a mapping `{robot_type: builder}`. A builder takes `robot_config`, the robot's registry entry for the
  arm profile completed with the node's arguments (`control_mode:=` and the others of the
  [Run table](../docs/tasks.md#run)), and returns the tree's root.
- The node takes the tasks of every installed package, this one's included: `tasks/core.py` holds one such mapping
  per task, and setup.py names each.
- A task name is global. Two packages registering one name refuse to start, and so does a registration that does not
  import or is not such a mapping: the node logs the task and the package and exits before anything moves. Start a
  project's task names with the project's.
- Entry points are read from the package's install, written when it is built: adding, renaming or removing a task in
  setup.py takes a `colcon build` of the package, also under `--symlink-install`. Changing a builder does not.

## A task's parameters

`task_params:=<file.yaml>` names one YAML file of the task's own parameters. Its mapping reaches the builder as
`robot_config['task_params']`, `{}` without the file. A file that cannot be read, is not YAML or holds anything but a
mapping is refused before the tree is built ([utils/task_params.py](cho_task_manager/utils/task_params.py)).

The builder checks the values. A `ValueError` it raises is logged and ends the node before anything moves, so a key
the task does not take should raise one too, or a misspelt key runs the task on its default: the template's
`joint_wave_params()` does both
([joint_wave.py](../templates/overlay_project/project_tasks/project_tasks/joint_wave.py)).

This package's tasks take theirs as node parameters, from before the file existed. A new task takes its own from the
file, not from a new node parameter.

## Testing a task

`cho_task_manager.testing` builds a task's tree from the registry alone, as the node builds it, and checks each goal
the tree sends against the robot's registry entry:

```python
from cho_task_manager.testing import goal_problems, registry_tree


def test_every_goal_names_the_registry_frame_and_joints():
    tree, config = registry_tree('myproject_wave', 'franka', task_params={'amplitude': 0.1})
    assert goal_problems(tree, config) == []
```

- `registry_tree(task, robot_type, profile='single', task_params=None)` returns the tree and the config it was built
  from, and raises what the builder raises.
- `goal_leaves(tree)` returns the tree's JointSpace and TaskSpace goal leaves.
- `goal_problems(tree, robot_config)` lists, as messages, each goal that does not name what the registry hands out: a
  TaskSpace goal the profile's goal frame for its relativity, a JointSpace goal the arm's joints in the registry's
  order ([CONTRACT.md, Frames](../cho_interfaces/CONTRACT.md#frames)). A TaskSpace goal whose pose comes from the
  blackboard carries that pose's frame, set at run time, and is the task's own test to check.

The task is found through its entry point, so these run against the installed package (`colcon test`).

## A task in a project

A research project's tasks live in the project's own repository, built as an overlay workspace on the core, not in
this package. [templates/overlay_project/](../templates/overlay_project/README.md) is a project to start from: one
task registered as above, its parameters file and its tests.
