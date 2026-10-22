# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A project task: one arm joint out by an amplitude and back.

On the robot's hold controller for the bringup's control mode, from and to the
registry's task home: the smallest tree that uses what a project task needs,
the registry's roles and poses, ``task_params`` and the standard guarded root.
It is registered in setup.py, so cho_task_manager runs it like its own::

    ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka \\
        task:=example_joint_wave control_mode:=position use_sim_time:=true \\
        task_params:=$(ros2 pkg prefix project_tasks)/share/project_tasks/config/joint_wave.yaml
"""

import py_trees

from cho_task_manager.behaviors.action import JointSpaceActionBehavior
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.subtrees import guarded_mission
from cho_task_manager.utils.controller_names import (
    arm_joint_names,
    hold_controllers,
    resolve_control_mode,
    task_home_positions,
)
from cho_task_manager.utils.msg_utils import make_joint_state

TASK_NAME = 'example_joint_wave'

#: What task_params may set, and its value when it does not.
DEFAULTS = {
    'joint': 7,          # 1-based arm joint
    'amplitude': 0.2,    # [rad], out and back by this much
    'duration': 2.0,     # [s] per leg, a minimum (cho_interfaces/CONTRACT.md)
    'repeats': 1,
}


def joint_wave_params(robot_config):
    """The task's parameters: DEFAULTS, overridden by ``robot_config['task_params']``.

    Raises ValueError, which the node reports before anything moves, for a
    parameter it does not take or a value it cannot run.
    """
    given = robot_config['task_params']
    unknown = sorted(set(given) - set(DEFAULTS))
    if unknown:
        raise ValueError(f'{TASK_NAME} does not take {unknown}; it takes {sorted(DEFAULTS)}')
    params = {**DEFAULTS, **given}
    joints = len(arm_joint_names(robot_config))
    if not isinstance(params['joint'], int) or not 1 <= params['joint'] <= joints:
        raise ValueError(f"{TASK_NAME}: joint must be 1..{joints}, got {params['joint']!r}")
    if not 0.0 < float(params['amplitude']) <= 0.5:
        raise ValueError(f"{TASK_NAME}: amplitude must be in (0, 0.5] rad, got {params['amplitude']!r}")
    if not float(params['duration']) > 0.0:
        raise ValueError(f"{TASK_NAME}: duration must be > 0 s, got {params['duration']!r}")
    if not isinstance(params['repeats'], int) or params['repeats'] < 1:
        raise ValueError(f"{TASK_NAME}: repeats must be an integer >= 1, got {params['repeats']!r}")
    return params


def create_joint_wave_tree(robot_config):
    """Switch to the hold controller, go home, wave the joint, come home."""
    params = joint_wave_params(robot_config)
    control_mode = resolve_control_mode(robot_config, default='position')
    controller = hold_controllers(robot_config, control_mode)[0]
    names = arm_joint_names(robot_config)
    home = task_home_positions(robot_config)
    away = list(home)
    away[params['joint'] - 1] += float(params['amplitude'])

    def move(name, target):
        return JointSpaceActionBehavior(
            name=name,
            target_joints=make_joint_state(target, names),
            controller_name=controller,
            duration=float(params['duration']),
            joint_names=names,
        )

    mission = py_trees.composites.Sequence(name='Joint_Wave', memory=True)
    mission.add_child(SwitchControllerServiceBehavior(
        name=f'Switch_{controller}', activate=[controller], robot_config=robot_config))
    mission.add_child(move('Home', home))
    for leg in range(params['repeats']):
        mission.add_child(move(f'Out_{leg + 1}', away))
        mission.add_child(move(f'Back_{leg + 1}', home))
    return guarded_mission(mission, robot_config, control_mode)


#: The registration setup.py points at: {robot_type: tree builder}.
TASK = {'franka': create_joint_wave_tree}
