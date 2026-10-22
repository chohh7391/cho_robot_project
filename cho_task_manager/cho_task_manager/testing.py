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

"""Checks of a task's trees, for this package's tests and a project package's.

A project that registers a task (cho_task_manager/tasks/__init__.py) runs them
from its own pytest. Every goal a tree sends must name the frame and the joints
the robot's registry hands out, the rule each tree here keeps
(cho_interfaces/CONTRACT.md, Frames)::

    from cho_task_manager.testing import goal_problems, registry_tree

    def test_my_task_sends_registry_goals():
        tree, config = registry_tree('my_task', 'franka', task_params={'amplitude': 0.1})
        assert goal_problems(tree, config) == []
"""

from cho_task_manager.behaviors.action import JointSpaceActionBehavior, TaskSpaceActionBehavior
from cho_task_manager.tasks import build_task_tree
from cho_task_manager.utils.controller_names import arm_joint_names, goal_frame, load_robot_config


def registry_tree(task, robot_type, profile='single', task_params=None):
    """*task*'s tree on *robot_type*'s *profile*, and the config it was built from.

    Built from the registry alone and *task_params*, as the node builds it with
    ``task_params:=`` a file holding them. Raises what the builder raises.
    """
    config = load_robot_config(robot_type, profile)
    config['task_params'] = dict(task_params or {})
    return build_task_tree(task, config), config


def goal_leaves(tree):
    """The JointSpace and TaskSpace goal leaves of *tree*."""
    return [node for node in tree.iterate()
            if isinstance(node, (TaskSpaceActionBehavior, JointSpaceActionBehavior))]


def goal_problems(tree, robot_config):
    """What the goals of *tree* get wrong against *robot_config*, as messages.

    A TaskSpace goal names the registry's goal frame for its relativity
    (goal_frame()); a JointSpace goal names the arm's joints in the registry's
    order, in the leaf and in a fixed target. A TaskSpace goal whose pose comes
    from the blackboard carries that pose's own frame, set at run time, and is
    the task's own test to check. Empty when every goal keeps the rule.
    """
    problems = []
    joints = arm_joint_names(robot_config)
    for leaf in goal_leaves(tree):
        if isinstance(leaf, TaskSpaceActionBehavior):
            if leaf.target_key is not None:
                continue
            frame = goal_frame(robot_config, leaf.relative)
            if leaf.frame_id != frame:
                problems.append(f"{leaf.name}: frame '{leaf.frame_id}', the registry's '{frame}'")
            continue
        if leaf.joint_names != joints:
            problems.append(f"{leaf.name}: joints {leaf.joint_names}, the registry's {joints}")
        if leaf.target_joints is not None and list(leaf.target_joints.name) != joints:
            problems.append(
                f"{leaf.name}: target names {list(leaf.target_joints.name)}, the registry's {joints}")
    return problems
