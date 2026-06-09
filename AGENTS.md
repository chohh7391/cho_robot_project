# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble workspace source tree. Core C++ controllers live in `cho_controller/`, with package code split into `src/` and `include/`. Robot descriptions, xacro files, generated URDFs, meshes, and RViz launch files are in `cho_description/cho_description_franka/`. Runtime launch files and controller YAMLs are in `cho_bringup/cho_bringup_franka/launch` and `config`. Python task orchestration and behavior-tree code is in `cho_task_manager/cho_task_manager`, with helper scripts under `cho_task_manager/python`. ROS messages and actions are defined in `cho_interfaces/msg` and `cho_interfaces/action`. External dependencies are vendored under `extern/`; avoid editing them unless the change is intentionally upstream-related.

## Build, Test, and Development Commands

Run commands from the workspace root, typically `~/ros2_ws`:

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select cho_task_manager cho_description_franka
colcon test-result --verbose
```

Use package-scoped builds while iterating, for example `colcon build --packages-select cho_controller_franka`. Launch real hardware only after checking config values such as `cho_bringup/cho_bringup_franka/config/real/franka.config.yaml`:

```bash
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position controller_name:=task_space_ik_controller
ros2 launch cho_task_manager run_task_manager.launch.py task:=<task_name>
```

## Coding Style & Naming Conventions

Use ROS 2 package conventions. C++ files use `.cpp`/`.hpp`, namespaces matching package names, and descriptive controller names such as `task_space_qp_controller`. Python code should follow the local `setup.cfg`: max line length is 120, selected flake8/pep257 rules are relaxed, and scripts install through `setup.py`. Keep launch arguments and xacro properties in `snake_case`; keep ROS link, joint, topic, and controller names stable because configs and tests depend on them.

## Testing Guidelines

Python lint tests live in `cho_task_manager/test` and URDF/xacro checks live in `cho_description/cho_description_franka/test`. Name new tests `test_*.py`. For controller or robot-description changes, at minimum build the affected package and run the relevant package tests. For xacro or frame changes, also inspect generated URDF transforms or validate in RViz/Gazebo before using hardware.

## Commit & Pull Request Guidelines

Recent history does not enforce a strict commit format. Prefer concise imperative subjects with a package scope, for example `cho_bringup_franka: update FT sensor transform`. PRs should describe the behavior change, list affected packages, include test commands/results, and call out hardware, simulation, or frame-convention impacts. Include screenshots or RViz/Gazebo evidence for visual or robot-description changes.

## Safety & Configuration Tips

Do not commit machine-specific robot IPs, secrets, or local network settings. Treat real robot launch changes as safety-critical: verify simulation paths first, document controller defaults, and preserve existing frame semantics unless the PR explicitly changes them.
