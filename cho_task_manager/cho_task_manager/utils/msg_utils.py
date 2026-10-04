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

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from typing import List


def make_pose(position: List, orientation: List = [1.0, 0.0, 0.0, 0.0]):
    """
    position: [x, y, z]
    orientation: [qx, qy, qz, qw]
    """
    p = Pose()
    p.position.x, p.position.y, p.position.z = position[0], position[1], position[2]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
        orientation[0], orientation[1], orientation[2], orientation[3])
    return p

def make_down_pose(height):
    p = Pose()
    p.position.x, p.position.y, p.position.z = 0.0, 0.0, float(height)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = 0.0, 0.0, 0.0, 1.0
    return p

def make_up_pose(height):
    p = Pose()
    p.position.x, p.position.y, p.position.z = 0.0, 0.0, float(-height)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = 0.0, 0.0, 0.0, 1.0
    return p

def make_joint_state(position, names=None):
    """A JointState target; with *names*, one per position, in the same order.

    Pass the registry's ``model.joints`` (``arm_joint_names(robot_config)``):
    a JointSpace server matches named positions by name and rejects a goal
    that names an unknown joint or misses one (cho_interfaces/CONTRACT.md), so
    a target meant for one arm of a bimanual robot cannot drive the other.
    Unnamed, the positions are taken in the server's own joint order.
    """
    js = JointState()
    js.position = [float(p) for p in position]
    if names is not None:
        names = [str(name) for name in names]
        if len(names) != len(js.position):
            raise ValueError(
                f'{len(js.position)} joint positions but {len(names)} joint names {names}')
        js.name = names
    return js


def named_joint_state(target, names):
    """*target* with its ``name`` filled from *names*, or as given when it already has one.

    Returns a copy; raises ValueError when the counts differ, which is a target
    written for another robot or arm.
    """
    if target.name:
        return target
    named = JointState()
    named.header = target.header
    named.position = list(target.position)
    named.velocity = list(target.velocity)
    named.effort = list(target.effort)
    names = [str(name) for name in names]
    if len(names) != len(named.position):
        raise ValueError(
            f'{len(named.position)} joint positions but this arm has {len(names)} '
            f'joints {names}')
    named.name = names
    return named
